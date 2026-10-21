#!/usr/bin/env bash
set -eo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$root/finenav_ws/install/finenav_simulator/share/finenav_simulator/package.xml" ]]; then
  echo 'Build the simulation first: bash tools/build_gazebo.sh' >&2
  exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-188}"
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${IGN_PARTITION:-finenav_neupan_$(id -u)}"
source "$root/finenav_ws/install/setup.bash"
exec ros2 launch finenav_example neupan_gazebo.launch.py "$@"
