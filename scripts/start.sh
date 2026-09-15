#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

OVERLAY_INSTALL="$PKG/rbnx-build/ws/install"
if [[ ! -f "$OVERLAY_INSTALL/setup.bash" ]]; then
    echo "[roboarm_ik/start] ERR: colcon overlay missing — run scripts/build.sh" >&2
    exit 2
fi

set +u
source "$OVERLAY_INSTALL/setup.bash"
set -u

_prepend_unique() {
    local var="$1" val="$2"
    local cur="${!var:-}"
    case ":${cur}:" in
        *":${val}:"*) ;;
        *) export "$var"="${val}${cur:+:${cur}}" ;;
    esac
}

for _prefix in "$OVERLAY_INSTALL"/*/; do
    _prefix="${_prefix%/}"
    [[ -d "$_prefix/share" ]] || continue
    _prepend_unique AMENT_PREFIX_PATH "$_prefix"
    _prepend_unique CMAKE_PREFIX_PATH "$_prefix"
    for _site in \
        "$_prefix"/local/lib/python*/dist-packages \
        "$_prefix"/lib/python*/site-packages \
        "$_prefix"/lib/python*/dist-packages
    do
        [[ -d "$_site" ]] && _prepend_unique PYTHONPATH "$_site"
    done
    for _libdir in "$_prefix"/lib "$_prefix"/local/lib; do
        [[ -d "$_libdir" ]] && _prepend_unique LD_LIBRARY_PATH "$_libdir"
    done
done
unset _prefix _site _libdir

if ! python3 -c "import graspnet_msgs.msg, piper_msgs.msg, kinpy, scipy" 2>/dev/null; then
    echo "[roboarm_ik/start] FATAL: required Python/ROS modules missing" >&2
    exit 3
fi

CODEGEN_PROTO="$PKG/rbnx-build/codegen/proto_gen"
if [[ ! -d "$CODEGEN_PROTO" ]]; then
    echo "[roboarm_ik/start] ERR: codegen output missing — run scripts/build.sh" >&2
    exit 2
fi

export PYTHONPATH="$CODEGEN_PROTO:$PKG:${PYTHONPATH:-}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

exec python3 -u -m roboarm_ik.main
