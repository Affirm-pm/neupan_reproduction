#!/usr/bin/env bash
set -eo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$root/finenav_ws/src/FineNav-Engine/finenav_example/create_custom_neupan_planner/tools/build.sh"
source "$root/finenav_ws/install/setup.bash"
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS="-j4 -l4"
cd "$root/finenav_ws"
colcon build --executor sequential --base-paths src --packages-select finenav_simulator \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
