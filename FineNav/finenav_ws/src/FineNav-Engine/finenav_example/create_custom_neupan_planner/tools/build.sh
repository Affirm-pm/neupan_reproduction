#!/usr/bin/env bash
set -eo pipefail
# Keep Conda's Python, fmt and yaml-cpp out of ROS's system toolchain. This
# changes only this script's environment; the caller's shell is unaffected.
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
plugin_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$(cd "$plugin_dir/../../../.." && pwd)"
deps="$workspace/neupan_dependencies"
neupan_source="${NEUPAN_SOURCE_DIR:-$workspace/../neupan_cpp_ws/src/neupan_cpp/libneupan}"
if [[ ! -f "$neupan_source/CMakeLists.txt" ]]; then
  echo "Missing libneupan source: $neupan_source" >&2
  echo "Keep finenav_ws and neupan_cpp_ws together under FineNav, or set NEUPAN_SOURCE_DIR." >&2
  exit 1
fi
neupan_source="$(cd "$neupan_source" && pwd)"
# Source snapshots shipped in neupan_dependencies/src retain their licenses.
cmake -S "$deps/src/osqp" -B "$deps/build/osqp" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$deps/install" \
  -DFETCHCONTENT_SOURCE_DIR_QDLDL="$deps/src/qdldl" -DOSQP_BUILD_DEMO_EXE=OFF
cmake --build "$deps/build/osqp" -j4
cmake --install "$deps/build/osqp"
cmake -S "$deps/src/osqp-eigen" -B "$deps/build/osqp-eigen" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$deps/install" \
  -DCMAKE_PREFIX_PATH="$deps/install" -DBUILD_TESTING=OFF
cmake --build "$deps/build/osqp-eigen" -j4
cmake --install "$deps/build/osqp-eigen"
source /opt/ros/humble/setup.bash
cd "$workspace"
export CMAKE_PREFIX_PATH="$deps/install:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS="${MAKEFLAGS:--j4 -l4}"
colcon build --executor sequential --base-paths src --packages-up-to finenav_example \
  --cmake-args -DBUILD_TESTING=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DOsqpEigen_DIR="$deps/install/lib/cmake/OsqpEigen" \
    -Dosqp_DIR="$deps/install/lib/cmake/osqp" \
    -DNEUPAN_SOURCE_DIR="$neupan_source"
