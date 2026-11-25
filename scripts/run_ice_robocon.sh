#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIMULATOR="${PROJECT_DIR}/cmake_build/bin/rl_sim_mujoco"

if [[ ! -x "$SIMULATOR" ]]; then
    echo "Build the simulator first: INFERENCE_RUNTIME=libtorch ./build.sh -mj" >&2
    exit 1
fi

cd "$PROJECT_DIR"
exec "$SIMULATOR" ice/robocon "${1:-scene}"
