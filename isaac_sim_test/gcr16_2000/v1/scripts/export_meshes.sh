#!/usr/bin/env bash
# Build GCR16 STLs on the sim box. STEP is 21MB; do not commit the STLs.
set -euo pipefail

V1="$(cd "$(dirname "$0")/.." && pwd)"
HAWK="$(cd "${V1}/../../.." && pwd)"
STEP="${STEP_PATH:-${HAWK}/isaac_sim_test/3d/GCR16-2000.STEP}"
OUT="${MESH_OUT:-${V1}/meshes}"
SCRIPT="${V1}/scripts/export_meshes_freecad.py"

if [ ! -f "${STEP}" ]; then
  echo "STEP not found: ${STEP}"
  echo "Set STEP_PATH=/path/to/GCR16-2000.STEP"
  exit 1
fi

FC=""
for c in freecadcmd FreeCADCmd freecad.cmd; do
  if command -v "${c}" >/dev/null 2>&1; then
    FC="${c}"
    break
  fi
done
if [ -z "${FC}" ]; then
  echo "Install FreeCAD: sudo apt install -y freecad"
  exit 1
fi

echo "STEP: ${STEP}"
echo "OUT:  ${OUT}"
"${FC}" "${SCRIPT}" "${STEP}" "${OUT}"
ls -lh "${OUT}"/GCR16-J*.stl
