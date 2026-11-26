#!/usr/bin/env bash
# Local Ubuntu dependencies, without modifying the training environment or sudo.
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
if [[ ! -x .venv-deploy/bin/python ]]; then
    uv venv --python "${ROBOCON_PYTHON:-3.11}" .venv-deploy
fi
uv pip install --python .venv-deploy/bin/python -r scripts/requirements-ice-robocon.txt

mkdir -p library/sysroot/debs library/inference_runtime library/mujoco/include library/mujoco/lib
if [[ ! -f library/sysroot/usr/lib/x86_64-linux-gnu/cmake/TBB/TBBConfig.cmake ]]; then
    (
        cd library/sysroot/debs
        apt-get download libyaml-cpp-dev libyaml-cpp0.8 libtbb-dev libtbb12 libtbbmalloc2 \
            libtbbbind-2-5 libglfw3-dev libglfw3 libgl-dev libglx-dev libopengl-dev libegl-dev
    )
    for package in library/sysroot/debs/*.deb; do
        dpkg-deb -x "$package" library/sysroot
    done
fi
.venv-deploy/bin/python - <<'PY'
from pathlib import Path
import mujoco, torch
torch_root = Path(torch.__file__).parent
mj_root = Path(mujoco.__file__).parent
for destination, source in (
    (Path('library/inference_runtime/libtorch'), torch_root),
    (Path('library/mujoco/include/mujoco'), mj_root / 'include/mujoco'),
    (Path('library/mujoco/lib/libmujoco.so.3.2.7'), mj_root / 'libmujoco.so.3.2.7'),
    (Path('library/mujoco/lib/libmujoco.so'), mj_root / 'libmujoco.so.3.2.7'),
):
    if not destination.exists():
        destination.symlink_to(source.resolve())
PY

# PyTorch 2.3 CPU wheels use ABI=0; yaml-cpp and every SDK object must agree.
if [[ ! -f library/yaml-cpp/lib/libyaml-cpp.so ]]; then
    if [[ ! -d library/yaml-cpp-src ]]; then
        git clone --depth 1 --branch 0.8.0 https://github.com/jbeder/yaml-cpp.git library/yaml-cpp-src
    fi
    cmake -S library/yaml-cpp-src -B library/yaml-cpp-build \
        -DYAML_CPP_BUILD_TESTS=OFF -DYAML_CPP_BUILD_TOOLS=OFF -DYAML_BUILD_SHARED_LIBS=ON \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-D_GLIBCXX_USE_CXX11_ABI=0 \
        -DCMAKE_INSTALL_PREFIX="$PROJECT_DIR/library/yaml-cpp"
    cmake --build library/yaml-cpp-build -j4
    cmake --install library/yaml-cpp-build
fi
cmake -S src/rl_sar -B cmake_build -DUSE_CMAKE=ON -DUSE_MUJOCO=ON -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$PROJECT_DIR/library/sysroot/usr" \
    -DPython3_EXECUTABLE="$PROJECT_DIR/.venv-deploy/bin/python" \
    -DCMAKE_CXX_FLAGS="-I$PROJECT_DIR/library/sysroot/usr/include -D_GLIBCXX_USE_CXX11_ABI=0" \
    -DCMAKE_EXE_LINKER_FLAGS="-L$PROJECT_DIR/library/yaml-cpp/lib -L$PROJECT_DIR/library/sysroot/usr/lib/x86_64-linux-gnu -L$PROJECT_DIR/library/mujoco/lib" \
    -DCMAKE_INSTALL_RPATH="$PROJECT_DIR/library/yaml-cpp/lib;$PROJECT_DIR/library/sysroot/usr/lib/x86_64-linux-gnu"
cmake --build cmake_build --target rl_sim_mujoco test_ice_robocon test_velocity_input -j4
echo "Ready. Import the selected robot_lab run, then run scripts/run_ice_robocon.sh for keyboard/gamepad control."
