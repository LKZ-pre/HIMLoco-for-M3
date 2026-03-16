#!/bin/bash

# --- 1. 环境自检 ---
if [ ! -d ".git" ]; then
    echo "❌ 错误：当前目录不是 Git 仓库，请在工作空间根目录运行！"
    exit 1
fi

# 检查 X11 连接数，预防你之前的“界面打不开”问题
X_CLIENTS=$(ls -l /proc/*/fd 2>/dev/null | grep -c "anon_inode:\[x11\]")
if [ "$X_CLIENTS" -gt 200 ]; then
    echo "⚠️ 警告：当前 X11 连接数高达 $X_CLIENTS，系统图形界面可能即将卡死！"
    echo "建议运行 'killall -9 rviz2 gazebo' 释放资源。"
fi

# --- 2. 状态识别 ---
BRANCH=$(git rev-parse --abbrev-ref HEAD)
echo "🚀 当前分支: [$BRANCH]"

# 强制检查是否误加了 build/install (即使有 .gitignore 也双重保险)
STAGED_BIG_FILES=$(git diff --cached --name-only | grep -E "^(build|install|log)/")
if [ ! -z "$STAGED_BIG_FILES" ]; then
    echo "🛑 拦截：检测到你正尝试提交编译产物 (build/install/log)！"
    echo "正在自动执行 reset 并移除这些文件..."
    git reset HEAD build/ install/ log/ 2>/dev/null
    echo "✅ 已拦截大文件上传，请检查 .gitignore 配置。"
fi

# --- 3. 提交信息处理 ---
# 如果有未暂存的修改，提示用户
if [ -z "$(git status --porcelain)" ]; then
    echo "✨ 当前代码没有改动，无需同步。"
    exit 0
fi

git add .
echo "📝 请输入提交说明 (直接回车将使用默认描述):"
read -r msg

if [ -z "$msg" ]; then
  msg="update: dev on $BRANCH at $(date +'%H:%M:%S')"
fi

# --- 4. 执行本地操作 ---
if git commit -m "$msg"; then
    echo "✅ 本地提交成功"
else
    echo "❌ 提交失败，可能没有可提交的内容"
    exit 1
fi

# --- 5. 远程同步 (关键步骤) ---
echo "📥 同步远程代码 (Pulling with Rebase)..."
# 设置 10 秒超时，防止网络卡死
if timeout 15s git pull origin "$BRANCH" --rebase; then
    echo "✅ 远程代码已同步"
else
    echo "❌ 同步失败：可能是网络问题或代码冲突。"
    echo "👉 若是冲突，请在 VS Code 中处理后执行: git rebase --continue"
    exit 1
fi

# --- 6. 推送更新 ---
echo "📤 推送至 GitHub..."
if timeout 20s git push origin "$BRANCH"; then
    echo "---------------------------------------"
    echo "🎉 同步完成！Robocon 2026 加油！旅行者！！！"
    echo "时间: $(date +'%Y-%m-%d %H:%M:%S')"
    echo "---------------------------------------"
else
    echo "❌ 推送失败：请检查 GitHub Token/SSH 权限或网络。"
    exit 1
fi