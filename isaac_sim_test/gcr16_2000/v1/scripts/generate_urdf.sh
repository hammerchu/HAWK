#!/usr/bin/env bash
# Flatten the move xacro into urdf/gcr16_2000.urdf for MoveIt Setup Assistant.
set -euo pipefail

V1="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-${V1}/urdf/gcr16_2000.urdf}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
# shellcheck disable=SC1091
source "${V1}/scripts/use_local_pkg.sh"

mkdir -p "$(dirname "${OUT}")"
xacro "${V1}/urdf/gcr16_2000_move.urdf.xacro" use_cad_meshes:=true > "${OUT}"

echo "Wrote ${OUT}"
echo "Load this in MoveIt Setup Assistant (planning group: gcr16_joint1 .. gcr16_joint6)."
echo "Keep this shell's AMENT_PREFIX_PATH so package://gcr16_2000_v1/meshes resolves."
