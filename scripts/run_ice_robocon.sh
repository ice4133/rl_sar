#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIMULATOR="${PROJECT_DIR}/cmake_build/bin/rl_sim_mujoco"

if [[ ! -x "$SIMULATOR" ]]; then
    echo "Build the simulator first: INFERENCE_RUNTIME=libtorch ./build.sh -mj" >&2
    exit 1
fi

cd "$PROJECT_DIR"
SCENE="${1:-scene}"
if (( $# > 0 )); then shift; fi
echo "请点击 MuJoCo 窗口，在窗口内使用键盘（终端不接收运动按键）。"
echo "手动启动：0 起立 → Enter 开始仿真 → 等待 2 秒 → 1 启用策略。"
echo "按住 W/S 前后、A/D 横移、Q/E 转向：直接达到配置速度上限，松开归零。"
echo "手柄摇杆推满达到同一速度上限；空格归零；Enter 暂停；R 复位。"
exec "$SIMULATOR" ice/robocon "$SCENE" "$@"
