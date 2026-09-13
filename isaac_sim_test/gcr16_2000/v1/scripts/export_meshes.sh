#!/usr/bin/env bash
# Build GCR16 STLs on the sim box. Do not pass the STEP as a FreeCADCmd argument.
set -euo pipefail

V1="$(cd "$(dirname "$0")/.." && pwd)"
HAWK="$(cd "${V1}/../../.." && pwd)"
export GCR16_STEP="${STEP_PATH:-${HAWK}/isaac_sim_test/3d/GCR16-2000.STEP}"
export GCR16_OUT="${MESH_OUT:-${V1}/meshes}"
SCRIPT="${V1}/scripts/export_meshes_freecad.py"

if [ ! -f "${GCR16_STEP}" ]; then
  echo "STEP not found: ${GCR16_STEP}"
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

echo "STEP: ${GCR16_STEP}"
echo "OUT:  ${GCR16_OUT}"
echo "FC:   ${FC} (only the .py is passed on the command line)"
"${FC}" "${SCRIPT}"

echo "--- meshes ---"
ls -lh "${GCR16_OUT}"/*.stl 2>/dev/null || echo "No STL written."
