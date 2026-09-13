#!/usr/bin/env bash
# Build GCR16 STLs on the sim box. Do not pass a STEP as a FreeCADCmd argument.
# Prefers per-link Fusion files isaac_sim_test/3d/GCR16-J0.step .. J6.step.
set -euo pipefail

V1="$(cd "$(dirname "$0")/.." && pwd)"
HAWK="$(cd "${V1}/../../.." && pwd)"
export GCR16_OUT="${MESH_OUT:-${V1}/meshes}"
export GCR16_LINK_DIR="${LINK_DIR:-${HAWK}/isaac_sim_test/3d}"
export GCR16_STEP="${STEP_PATH:-${HAWK}/isaac_sim_test/3d/GCR16-2000.STEP}"

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

LINK_SCRIPT="${V1}/scripts/export_link_steps_freecad.py"
FALLBACK="${V1}/scripts/export_meshes_freecad.py"

if [ -f "${GCR16_LINK_DIR}/GCR16-J0.step" ] || [ -f "${GCR16_LINK_DIR}/GCR16-J0.stp" ]; then
  echo "Using per-link Fusion STEPs in ${GCR16_LINK_DIR}"
  echo "OUT: ${GCR16_OUT}"
  echo "FC:  ${FC} (only the .py is passed on the command line)"
  ls "${GCR16_LINK_DIR}"/GCR16-J[0-6].step "${GCR16_LINK_DIR}"/GCR16-J[0-6].stp 2>/dev/null || true
  "${FC}" "${LINK_SCRIPT}"
else
  if [ ! -f "${GCR16_STEP}" ]; then
    echo "No GCR16-J0.step in ${GCR16_LINK_DIR} and no full STEP at ${GCR16_STEP}"
    exit 1
  fi
  echo "No per-link STEPs — falling back to 7-solid split of ${GCR16_STEP}"
  echo "OUT: ${GCR16_OUT}"
  "${FC}" "${FALLBACK}"
fi

python3 "${V1}/scripts/fix_stls.py" "${GCR16_OUT}"

echo "--- meshes ---"
ls -lh "${GCR16_OUT}"/GCR16-*.stl 2>/dev/null || echo "No STL written."
