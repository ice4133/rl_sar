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
MODEL="${PROJECT_DIR}/src/rl_sar_zoo/ice/robocon_description/mjcf/${SCENE}.xml"
if [[ ! -f "$MODEL" ]]; then
    echo "Missing MuJoCo scene: $MODEL" >&2
    echo "Generate the model using scripts/import_ice_robocon.py --model-only --run-dir <saved-run> --asset-root <robocon-assets>" >&2
    exit 1
fi
echo "请点击 MuJoCo 窗口，在窗口内使用键盘（终端不接收运动按键）。"
echo "手动启动：0 起立 → Enter 开始仿真 → 等待 2 秒 → 1 启用策略。"
echo "按住 W/S 前后、A/D 横移、Q/E 转向：以配置上限为目标平滑加速，松开平滑减速。"
echo "0/9 先减速再切换站姿/趴下；1 取消普通停车请求；P 直接进入阻尼。"
echo "手柄摇杆推满使用同一目标上限；空格平滑停车；Enter 暂停；R 复位。"
exec "$SIMULATOR" ice/robocon "$SCENE" "$@"
