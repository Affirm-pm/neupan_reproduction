#!/usr/bin/env bash
# Builds and installs OSQP and osqp-eigen, which have no Ubuntu/rosdep
# packages. Usage: install_osqp.sh [prefix]   (default /usr/local)
set -euo pipefail

PREFIX="${1:-/usr/local}"
OSQP_VERSION=v1.0.0
OSQP_EIGEN_VERSION=v0.11.2
WORK="$(mktemp -d)"
SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO=sudo; fi

git clone --depth 1 --branch "$OSQP_VERSION" https://github.com/osqp/osqp "$WORK/osqp"
cmake -S "$WORK/osqp" -B "$WORK/osqp/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$WORK/osqp/build" -j"$(nproc)"
$SUDO cmake --install "$WORK/osqp/build"

git clone --depth 1 --branch "$OSQP_EIGEN_VERSION" \
  https://github.com/robotology/osqp-eigen "$WORK/osqp-eigen"
cmake -S "$WORK/osqp-eigen" -B "$WORK/osqp-eigen/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$PREFIX"
cmake --build "$WORK/osqp-eigen/build" -j"$(nproc)"
$SUDO cmake --install "$WORK/osqp-eigen/build"

# Refresh the loader cache, or installed binaries (e.g. neupan_node, which
# has no RPATH to /usr/local/lib) fail with "libOsqpEigen.so: cannot open".
$SUDO ldconfig

rm -rf "$WORK"
