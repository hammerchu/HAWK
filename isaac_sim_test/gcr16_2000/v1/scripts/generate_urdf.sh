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
echo "Next: ./scripts/run_setup_assistant.sh"
echo "Do not launch Setup Assistant from a plain Humble shell — it will crash looking for gcr16_2000_v1."
