# 导入支持库
import yaml                # 从 YAML 文件读取配置
import torch               # 张量运算和模型加载
import mujoco              # MuJoCo 物理引擎 Python 绑定
import mujoco.viewer       # 启动 MuJoCo 视觉窗口
import time                # 计时工具
import numpy as np         # 数值辅助
from pynput import keyboard  # 监听键盘输入
from cmd_keyboard import get_cmd  # 从键盘获取速度命令的辅助函数

# ------------------ 工作部分 ------------------
# 将配置文件读入字典
with open("config.yaml", "r") as f:
    cfg = yaml.safe_load(f)  # 解析 YAML 配置
# 选择使用的设备（GPU 或 CPU）
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 从配置中提取常用字段
paths = cfg["paths"]              # 包含 scene_xml 和 policy_path
joint_names = cfg["joint_names"]  # 机器人关节名称列表
wheel_ids = cfg["wheel_ids"]      # 在关节列表中代表车轮的索引

# 默认和蹲伏的关节位置，复制到四条腿
default_dof_pos = torch.tensor(cfg["default_dof_pos"] * 4, dtype=torch.float32, device=device)
crouch_dof_pos  = torch.tensor(cfg["crouch_dof_pos"] * 4, dtype=torch.float32, device=device)

# PD 控制增益，同样复制四条腿
p_gains = torch.tensor(cfg["p_gains"] * 4, dtype=torch.float32, device=device)
d_gains = torch.tensor(cfg["d_gains"] * 4, dtype=torch.float32, device=device)
actions_scale = cfg["actions_scale"]  # RL 动作缩放因子
vel_scale = cfg["vel_scale"]          # 车轮速度缩放
yaw_kp = cfg["yaw_kp"]                # 偏航比例增益

scale_factors = cfg["scale_factors"]  # 观测量的归一化比例

# 加载 MuJoCo 场景 XML 并创建模型与数据
m = mujoco.MjModel.from_xml_path(paths["scene_xml"])
d = mujoco.MjData(m)

# 按名称读取传感器数据的辅助函数
def get_sensor_data(name):
    id_ = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
    if id_ == -1:
        raise ValueError(f"Sensor {name} not found")
    adr, dim = m.sensor_adr[id_], m.sensor_dim[id_]
    return torch.tensor(d.sensordata[adr:adr+dim], device=device, dtype=torch.float32)

# 将世界坐标系向量转换到机器人自身坐标系
def world2self(quat, v):
    q_w, q_vec = quat[0], quat[1:]
    v_vec = torch.tensor(v, device=device, dtype=torch.float32)
    a = v_vec * (2.0 * q_w**2 - 1.0)
    b = torch.linalg.cross(q_vec, v_vec) * q_w * 2.0
    c = q_vec * torch.dot(q_vec, v_vec) * 2.0
    return a - b + c

# 构造策略所需的观测向量
def get_obs(actions, default_dof_pos, commands):
    sf = scale_factors
    commands_scale = torch.tensor([sf["scale_lin_vel"], sf["scale_lin_vel"], sf["scale_ang_vel"]], device=device)
    base_quat = get_sensor_data("imu_quat")
    projected_gravity = world2self(base_quat, torch.tensor([0., 0., -1.], device=device))
    imu_gyro = get_sensor_data("imu_gyro")

    # 得到关节位置并将车轮设为零
    dof_pos = torch.zeros(16, device=device)
    for i, n in enumerate(joint_names):
        dof_pos[i] = get_sensor_data(n + "_pos")[0]
    dof_pos[wheel_ids] = 0.0

    # 得到关节速度
    dof_vel = torch.zeros(16, device=device)
    for i, n in enumerate(joint_names):
        dof_vel[i] = get_sensor_data(n + "_vel")[0]

    cmds = torch.tensor(commands, device=device)
    return torch.cat([
        imu_gyro * sf["scale_ang_vel"],
        projected_gravity,
        cmds * commands_scale,
        (dof_pos - default_dof_pos) * sf["scale_dof_pos"],
        dof_vel * sf["scale_dof_vel"],
        actions
    ], dim=-1)

# 主运行函数

def main():
    global control_mode
    control_mode = 0 # 初始为阻尼模式（0=阻尼 1=PD 2=RL）

    # 尝试加载训练好的策略网络
    try:
        policy = torch.jit.load(paths["policy_path"])
        policy.eval().to(device)
        print("Success to load policy network")
    except Exception as e:
        policy = None
        print("Fail to load policy network", e)

    # 将机器人关节放到蹲伏位置
    for i, name in enumerate(joint_names):
        jnt_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        d.qpos[m.jnt_qposadr[jnt_id]] = crouch_dof_pos[i].item()
    for _ in range(200):
        mujoco.mj_step(m, d)
    print("Damping mode......")
    print("1 -> PD   2 -> RL")

    actions = torch.zeros(16, device=device)            # 保存上一次动作向量
    obs_buffer = torch.zeros((6, 57), device=device)    # RNN 风格策略的历史缓冲

    # 键盘回调，用于切换控制模式
    def on_press(key):
        global control_mode
        try:
            if key.char == '1' and control_mode == 0:
                control_mode = 1
                print(" PD mode ......")
            elif key.char == '2' and control_mode == 1 and policy is not None:
                control_mode = 2
                print(" RL mode ......")
        except AttributeError:
            pass

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # 使用 viewer 运行仿真循环
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            dof_pos = torch.cat([get_sensor_data(n+"_pos") for n in joint_names]).to(device)
            dof_vel = torch.cat([get_sensor_data(n+"_vel") for n in joint_names]).to(device)
            dof_err = default_dof_pos - dof_pos

            # RL 控制分支
            if control_mode == 2:
                kb_cmd = get_cmd()
                commands = kb_cmd
                base_quat = get_sensor_data("imu_quat")
                q_w, q_x, q_y, q_z = base_quat
                yaw_now = torch.atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y*q_y + q_z*q_z))
                yaw_err = torch.atan2(torch.sin(commands[2] - yaw_now), torch.cos(commands[2] - yaw_now))
                commands[2] = yaw_kp * yaw_err
                print(f"\rRL cmd: vx={commands[0]:+4.1f}  vy={commands[1]:+4.1f}  wz={commands[2]:+4.1f}  yaw_now={yaw_now:+4.2f}", end='')
            else:
                commands = [0., 0., 0.]
            # PD 控制分支
            if control_mode == 1:
                act = torch.zeros(16, device=device)
                for i in range(16):
                    if i in wheel_ids:
                        act[i] = -d_gains[i]*dof_vel[i]
                    else:
                        act[i] = (1.2 * 1.25 * p_gains[i]*dof_err[i] - d_gains[i]*dof_vel[i])
                d.ctrl[:] = torch.clip(act, -100, 100).cpu().numpy()

            elif control_mode == 2 and policy is not None:

                obs_now = get_obs(actions, default_dof_pos, commands)
                obs_now = torch.clip(obs_now, -100, 100)
                obs_buffer = torch.cat([obs_now.unsqueeze(0), obs_buffer[:-1]], dim=0)
                obs_seq = obs_buffer.flatten()
                actions = policy(obs_seq)
                actions_scaled = actions * actions_scale
                vel_ref = torch.zeros_like(actions_scaled)
                vel_ref[wheel_ids] = actions[wheel_ids] * vel_scale
                act = p_gains * (actions_scaled + dof_err) + d_gains * (vel_ref - dof_vel)
                d.ctrl[:] = torch.clip(act, -100, 100).detach().cpu().numpy()
            else:
                d.ctrl[:] = 0.0

            step_start = time.time()
            for _ in range(cfg["sim_steps_per_loop"]):
                mujoco.mj_step(m, d)
            # 相机对准机器人底座
            viewer.cam.lookat[:] = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base')]
            viewer.sync()
            time_until_next_step = m.opt.timestep*4 - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
