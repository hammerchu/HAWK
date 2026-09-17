#!/usr/bin/env bash
# Generate the URDF (if needed) and open MoveIt Setup Assistant with gcr16_2000_v1 on AMENT_PREFIX_PATH.
# Setup Assistant crashes with PackageNotFoundError if you launch it from a plain Humble shell.
set -euo pipefail

V1="$(cd "$(dirname "$0")/.." && pwd)"
URDF="${V1}/urdf/gcr16_2000.urdf"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
# shellcheck disable=SC1091
source "${V1}/scripts/use_local_pkg.sh"

if [ ! -f "${URDF}" ]; then
  echo "No ${URDF} — generating"
  "${V1}/scripts/generate_urdf.sh" "${URDF}"
fi

echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}"
echo "package share: $(ros2 pkg prefix gcr16_2000_v1)/share/gcr16_2000_v1"
echo "Load this URDF in Setup Assistant:"
echo "  ${URDF}"
echo "Planning group joints: gcr16_joint1 .. gcr16_joint6"
exec ros2 run moveit_setup_assistant moveit_setup_assistant
