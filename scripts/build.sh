#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[roboarm_ik/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi

mkdir -p rbnx-build/ws/src
for sub in graspnet_msgs piper_msgs; do
    ln -snf "$PKG/src/$sub" "$PKG/rbnx-build/ws/src/$sub"
done

ROS_DISTRO="${ROS_DISTRO:-humble}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo "[roboarm_ik/build] colcon build (msgs only)"
cd "$PKG/rbnx-build/ws"
colcon build --symlink-install \
    --event-handlers console_direct+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

FLAGS=(--out-dir "$PKG/rbnx-build/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[roboarm_ik/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[roboarm_ik/build] done."
