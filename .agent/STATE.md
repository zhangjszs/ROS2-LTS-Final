# STATE
- 更新时间：2026-10-06T09:07Z
- 当前 Issue：#56（blocked，ccache 已装、mold 缺失待用户安装；验收 1/3 通过）
- 分支：main（零文件变更，无临时分支；`build/` 缓存未动，他人 `AGENTS.md` 脏改动未碰）
- 未完成工作：#56 恢复点 = 用户手动 `sudo apt-get install -y mold` 后 Planner 打回 ready/in-progress，下一棒直接重跑 `bash scripts/check_cpp20.sh` → `SKIP_BUILD=1 drive_gates.sh`（cpp20 门 + 附带 mpc-reject-smoke 门）；#55 待 2 次 post-fix Nightly（下一次约今日 20:xxZ）
- 最近提交：无代码提交（本轮仅 2 条 #56 comment + 标签流转 + `.agent/` 状态提交）
- 已知环境限制：ccache 4.12.3 已就绪；mold 缺失（`which` 空，stale CMakeCache 要求 `CMAKE_LINKER_TYPE=MOLD`）；无免密 sudo；本机 ROS=lyrical（CI Jazzy），多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`
