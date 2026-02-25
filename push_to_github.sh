#!/bin/bash
# 将本仓库推送到你自己的 GitHub
# 用法：
#   1. 在 GitHub 新建空仓库，复制其 SSH 或 HTTPS 地址
#   2. 把下面 GITHUB_REPO 改成你的地址，保存后执行: bash push_to_github.sh

set -e
cd "$(dirname "$0")"

# ========== 请改成你的 GitHub 仓库地址 ==========
# 示例 SSH: git@github.com:你的用户名/bingda_ros1_noetic.git
# 示例 HTTPS: https://github.com/你的用户名/bingda_ros1_noetic.git
GITHUB_REPO="git@github.com:YOUR_USERNAME/bingda_ros1_noetic.git"

# ========== 以下无需修改 ==========
if [[ "$GITHUB_REPO" == *"YOUR_USERNAME"* ]]; then
  echo "请先编辑本脚本，把 GITHUB_REPO 改成你的 GitHub 仓库地址后再运行。"
  exit 1
fi

if git remote get-url github &>/dev/null; then
  echo "远程 github 已存在，更新为: $GITHUB_REPO"
  git remote set-url github "$GITHUB_REPO"
else
  echo "添加远程 github: $GITHUB_REPO"
  git remote add github "$GITHUB_REPO"
fi

echo "当前分支: $(git branch --show-current)"
echo "正在推送到 GitHub ..."
git push -u github "$(git branch --show-current)"

echo "完成。若 GitHub 上想用 main 分支，可在网页端改默认分支后再执行: git push github nano:main"
