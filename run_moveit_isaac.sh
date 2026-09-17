#!/usr/bin/env bash
# Open two (or three) shells: Isaac Sim + MoveIt demo, same ROS 2 radio.
# You still press Play in Isaac. Do not start this from an Isaac-tainted env.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
MOVEIT_SETUP="${MOVEIT_SETUP:-$HOME/ws_moveit2/panda_arm_example/install/setup.bash}"
ISAAC_DIR="${ISAAC_DIR:-$HOME/DEV/isaacsim/_build/linux-x86_64/release}"
ISAAC_SH="${ISAAC_SH:-}"
SCENE_USD="${SCENE_USD:-}"
RUN_CURVE="${RUN_CURVE:-0}"
FOLLOW_CURVE="${FOLLOW_CURVE:-$ROOT/isaac_sim_test/follow_curve_moveit.py}"

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

# Open a titled terminal that stays up after the command (or drop to a shell).
open_shell() {
    local title="$1"
    local cmd="$2"
    local wrapped="echo '=== ${title} ==='; ${cmd}; echo; echo '[${title}] done. Shell stays open.'; exec bash"

    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="${title}" -- bash -lc "${wrapped}"
    elif command -v konsole >/dev/null 2>&1; then
        konsole --title "${title}" -e bash -lc "${wrapped}" >/dev/null 2>&1 &
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

main() {
    if [ ! -f "${ROS_SETUP}" ]; then
        echo "Missing ${ROS_SETUP}. Install Humble or set ROS_SETUP=..."
        exit 1
    fi
    if [ ! -f "${MOVEIT_SETUP}" ]; then
        echo "Missing ${MOVEIT_SETUP}. Set MOVEIT_SETUP=.../install/setup.bash"
        exit 1
    fi

    local isaac_sh
    isaac_sh="$(find_isaac_launcher)"
    if [ -z "${isaac_sh}" ]; then
        echo "Could not find isaac-sim.sh. Set ISAAC_DIR or ISAAC_SH."
        exit 1
    fi

    local isaac_cmd moveit_cmd curve_cmd
    isaac_cmd="$(ros_prelude)
cd $(printf '%q' "$(dirname "${isaac_sh}")")
echo 'Launching Isaac. Load the saved scene (if needed) and press Play.'
"
    if [ -n "${SCENE_USD}" ]; then
        isaac_cmd+="$(printf '%q' "${isaac_sh}") $(printf '%q' "${SCENE_USD}")"
    else
        isaac_cmd+="$(printf '%q' "${isaac_sh}")"
    fi

    moveit_cmd="$(ros_prelude)
source $(printf '%q' "${MOVEIT_SETUP}")
ros2 daemon stop || true
echo 'Starting MoveIt demo. Wait until /controller_manager is up.'
ros2 launch panda_arm_example demo.launch.py
"

    open_shell "HAWK Isaac" "${isaac_cmd}"
    sleep 2
    open_shell "HAWK MoveIt" "${moveit_cmd}"

    if [ "${RUN_CURVE}" = "1" ]; then
        curve_cmd="$(ros_prelude)
source $(printf '%q' "${MOVEIT_SETUP}")
echo 'Waiting 25s for Isaac Play + MoveIt controllers...'
sleep 25
python3 $(printf '%q' "${FOLLOW_CURVE}")
"
        open_shell "HAWK Curve" "${curve_cmd}"
    fi

    echo "Opened Isaac + MoveIt shells (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})."
    echo "In Isaac: scene loaded, ROS 2 Bridge on, press Play."
    echo "Optional curve later:"
    echo "  python3 ${FOLLOW_CURVE}"
    echo "  python3 ${FOLLOW_CURVE} --stop"
    if command -v tmux >/dev/null 2>&1 && [ -z "${TMUX:-}" ] && ! command -v gnome-terminal >/dev/null 2>&1; then
        echo "tmux session hawk_moveit — attach with: tmux attach -t hawk_moveit"
    fi
}

main "$@"
