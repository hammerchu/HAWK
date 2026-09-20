#!/usr/bin/env bash
# One shot: export S-path assets, open Isaac + MoveIt, wait for Play, follow S.
# You still press Play in Isaac (and File>Add the window USDA if it is not in the scene).
# Do not start this from an Isaac-tainted env.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
MOVEIT_SETUP="${MOVEIT_SETUP:-$HOME/ws_moveit2/duco_arm_example/install/setup.bash}"
LOCAL_PKG="${LOCAL_PKG:-$ROOT/isaac_sim_test/gcr16_2000/v1/scripts/use_local_pkg.sh}"
ISAAC_DIR="${ISAAC_DIR:-$HOME/DEV/isaacsim/_build/linux-x86_64/release}"
ISAAC_SH="${ISAAC_SH:-}"
SCENE_USD="${SCENE_USD:-}"
WINDOW_USD="${WINDOW_USD:-$ROOT/isaac_sim_test/window_path/generated/window_frame.usda}"
RUN_CURVE="${RUN_CURVE:-1}"
FOLLOW_CURVE="${FOLLOW_CURVE:-$ROOT/isaac_sim_test/follow_s_path_moveit.py}"
EXPORT_ALIGNED="${EXPORT_ALIGNED:-$ROOT/isaac_sim_test/window_path/export_aligned.py}"

# Pick isaac-sim.sh if the caller did not set ISAAC_SH.
find_isaac_launcher() {
    if [ -n "${ISAAC_SH}" ] && [ -x "${ISAAC_SH}" ]; then
        echo "${ISAAC_SH}"
        return
    fi
    local candidate
    for candidate in \
        "${ISAAC_DIR}/isaac-sim.sh" \
        "${ISAAC_DIR}/isaacsim.sh" \
        "${HOME}/.local/share/ov/pkg/isaac_sim/isaac-sim.sh"
    do
        if [ -x "${candidate}" ]; then
            echo "${candidate}"
            return
        fi
    done
    echo ""
}

# Use the known GCR+windows stage if SCENE_USD was not passed.
find_default_scene() {
    if [ -n "${SCENE_USD}" ]; then
        echo "${SCENE_USD}"
        return
    fi
    local candidate
    for candidate in \
        "${HOME}/DEV/isaacsim/documents/duco_arm_windows_v1.usd" \
        "${HOME}/DEV/isaacsim/documents/duco_arm_windows_v1.usda"
    do
        if [ -f "${candidate}" ]; then
            echo "${candidate}"
            return
        fi
    done
    echo ""
}

# Open a titled terminal that stays up after the command (or drop to a shell).
open_shell() {
    local title="$1"
    local cmd="$2"
    local wrapped="echo '=== ${title} ==='; ${cmd}; echo; echo '[${title}] done. Shell stays open.'; exec bash"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="${title}" -- bash -lc "${wrapped}"
    elif command -v konsole >/dev/null 2>&1; then
        konsole --title="${title}" -e bash -lc "${wrapped}" >/dev/null 2>&1 &
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        xfce4-terminal --title="${title}" -e "bash -lc $(printf '%q' "${wrapped}")" &
    elif command -v tmux >/dev/null 2>&1; then
        if [ -z "${TMUX:-}" ]; then
            tmux new-session -d -s hawk_moveit -n "${title}" "bash -lc $(printf '%q' "${wrapped}")"
        else
            tmux new-window -n "${title}" "bash -lc $(printf '%q' "${wrapped}")"
        fi
    else
        echo "No gnome-terminal / konsole / xfce4-terminal / tmux. Run this by hand:"
        echo "${cmd}"
        return 1
    fi
}

# Shared Humble exports for every child shell (no Isaac LD_LIBRARY_PATH).
ros_prelude() {
    cat <<EOF
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
# drop leftover Isaac libs so ros2_control_node does not abort
unset LD_LIBRARY_PATH
source ${ROS_SETUP}
EOF
}

# Overlay duco_arm_example plus the local gcr16 mesh package.
moveit_prelude() {
    cat <<EOF
$(ros_prelude)
source $(printf '%q' "${MOVEIT_SETUP}")
if [ -f $(printf '%q' "${LOCAL_PKG}") ]; then
  source $(printf '%q' "${LOCAL_PKG}")
fi
EOF
}

main() {
    if [ ! -f "${ROS_SETUP}" ]; then
        echo "Missing ${ROS_SETUP}. Install Humble or set ROS_SETUP=..."
        exit 1
    fi
    if [ ! -f "${MOVEIT_SETUP}" ]; then
        echo "Missing ${MOVEIT_SETUP}. Set MOVEIT_SETUP=.../install/setup.bash"
        exit 1
    fi
    if [ ! -f "${FOLLOW_CURVE}" ]; then
        echo "Missing ${FOLLOW_CURVE}"
        exit 1
    fi

    echo "=== HAWK: export aligned window + S path ==="
    python3 "${EXPORT_ALIGNED}"

    local isaac_sh scene_usd
    isaac_sh="$(find_isaac_launcher)"
    if [ -z "${isaac_sh}" ]; then
        echo "Could not find isaac-sim.sh. Set ISAAC_DIR or ISAAC_SH."
        exit 1
    fi
    scene_usd="$(find_default_scene)"

    local isaac_cmd moveit_cmd curve_cmd
    isaac_cmd="$(ros_prelude)
cd $(printf '%q' "$(dirname "${isaac_sh}")")
echo
echo '*** ISAAC: ROS 2 Bridge ON, then PRESS PLAY ***'
echo 'If the green S loop / window is missing:'
echo '  File -> Add -> $(printf '%q' "${WINDOW_USD}")'
echo 'Parent path under /window_frame. Do not scale the curve.'
echo
"
    if [ -n "${scene_usd}" ]; then
        echo "Isaac scene: ${scene_usd}"
        isaac_cmd+="$(printf '%q' "${isaac_sh}") $(printf '%q' "${scene_usd}")"
    else
        echo "No default SCENE_USD. Open your GCR stage after Isaac starts."
        isaac_cmd+="$(printf '%q' "${isaac_sh}")"
    fi

    moveit_cmd="$(moveit_prelude)
ros2 daemon stop || true
echo 'Starting MoveIt demo. Wait until /controller_manager is up.'
ros2 launch duco_arm_example demo.launch.py
"

    open_shell "HAWK Isaac" "${isaac_cmd}"
    sleep 2
    open_shell "HAWK MoveIt" "${moveit_cmd}"

    if [ "${RUN_CURVE}" = "1" ]; then
        curve_cmd="$(moveit_prelude)
echo
echo '*** PRESS PLAY IN ISAAC, then this shell waits for /isaac_joint_states ***'
echo 'RViz: Add MarkerArray topic /hawk/s_path'
echo
python3 $(printf '%q' "${FOLLOW_CURVE}")
"
        open_shell "HAWK S-path" "${curve_cmd}"
    fi

    echo
    echo "Opened Isaac + MoveIt + S-path (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})."
    echo "You click: Isaac Play (and File>Add window_frame.usda if the pane is missing)."
    echo "The S-path shell waits, joint-approaches the pane, then traces S."
    echo "Stop:  python3 ${FOLLOW_CURVE} --stop"
    echo "Skip follow next time: RUN_CURVE=0 $0"
    if command -v tmux >/dev/null 2>&1 && [ -z "${TMUX:-}" ] && ! command -v gnome-terminal >/dev/null 2>&1; then
        echo "tmux session hawk_moveit — attach with: tmux attach -t hawk_moveit"
    fi
}

main "$@"
