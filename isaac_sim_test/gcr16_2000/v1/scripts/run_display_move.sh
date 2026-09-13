#!/usr/bin/env bash
# Rebuild per-link STLs if needed, then open RViz sliders (mode:=move).
set -euo pipefail

V1="$(cd "$(dirname "$0")/.." && pwd)"
HAWK="$(cd "${V1}/../../.." && pwd)"
LINK_DIR="${HAWK}/isaac_sim_test/3d"
MESH_DIR="${V1}/meshes"

need_export=0
for i in 0 1 2 3 4 5 6; do
  step=""
  if [ -f "${LINK_DIR}/GCR16-J${i}.step" ]; then
    step="${LINK_DIR}/GCR16-J${i}.step"
  elif [ -f "${LINK_DIR}/GCR16-J${i}.stp" ]; then
    step="${LINK_DIR}/GCR16-J${i}.stp"
  fi
  stl="${MESH_DIR}/GCR16-J${i}.stl"
  if [ -z "${step}" ]; then
    echo "Missing ${LINK_DIR}/GCR16-J${i}.step"
    exit 1
  fi
  if [ ! -f "${stl}" ] || [ "${step}" -nt "${stl}" ]; then
    need_export=1
  fi
done

if [ "${need_export}" -eq 1 ]; then
  echo "Link STEPs newer than STLs — exporting meshes"
  "${V1}/scripts/export_meshes.sh"
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${V1}/scripts/use_local_pkg.sh"
cd "${V1}"
exec python3 launch/display.launch.py mode:=move
