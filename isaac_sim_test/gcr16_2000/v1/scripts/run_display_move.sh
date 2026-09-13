#!/usr/bin/env bash
# Rebuild per-link STLs if needed, then open RViz sliders (mode:=move).
set -euo pipefail

V1="$(cd "$(dirname "$0")/.." && pwd)"
HAWK="$(cd "${V1}/../../.." && pwd)"
LINK_DIR="${HAWK}/isaac_sim_test/3d"
MESH_DIR="${V1}/meshes"

_link_step() {
  # Print path to GCR16-Jn.step or .stp if it exists.
  local i="$1"
  if [ -f "${LINK_DIR}/GCR16-J${i}.step" ]; then
    echo "${LINK_DIR}/GCR16-J${i}.step"
  elif [ -f "${LINK_DIR}/GCR16-J${i}.stp" ]; then
    echo "${LINK_DIR}/GCR16-J${i}.stp"
  fi
}

need_export=0
stamp="${MESH_DIR}/.link_local_v2"
j1="${MESH_DIR}/GCR16-J1.stl"
if [ "${FORCE_MESH:-0}" = "1" ]; then
  need_export=1
fi
if [ ! -f "${stamp}" ]; then
  echo "Mesh stamp missing (${stamp}) — will rebuild link-local STLs"
  need_export=1
fi
if [ -f "${j1}" ] && [ "$(wc -c < "${j1}")" -lt 1000000 ]; then
  echo "GCR16-J1.stl is tiny — looks like the old 7-solid split, rebuilding"
  need_export=1
fi
for i in 0 1 2 3 4 5 6; do
  step="$(_link_step "${i}")"
  stl="${MESH_DIR}/GCR16-J${i}.stl"
  if [ -z "${step}" ]; then
    echo "Missing ${LINK_DIR}/GCR16-J${i}.step"
    echo "Copy the seven Fusion link STEPs onto the sim box, then re-run."
    exit 1
  fi
  if [ ! -f "${stl}" ] || [ "${step}" -nt "${stl}" ]; then
    need_export=1
  fi
done

if [ "${need_export}" -eq 1 ]; then
  echo "Exporting per-link Fusion STEPs to link-local ROS STLs"
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
  export GCR16_LINK_DIR="${LINK_DIR}"
  export GCR16_OUT="${MESH_DIR}"
  echo "LINK_DIR ${GCR16_LINK_DIR}"
  echo "OUT      ${GCR16_OUT}"
  echo "FC       ${FC}"
  "${FC}" "${V1}/scripts/export_link_steps_freecad.py"
  python3 "${V1}/scripts/fix_stls.py" "${MESH_DIR}"
  date > "${stamp}"
  echo "--- meshes ---"
  ls -lh "${MESH_DIR}"/GCR16-J*.stl
fi

# Humble setup.bash reads optional unset vars; set -u would abort before RViz.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
# shellcheck disable=SC1091
source "${V1}/scripts/use_local_pkg.sh"
cd "${V1}"
echo "Launching RViz sliders: python3 launch/display.launch.py mode:=move"
exec python3 launch/display.launch.py mode:=move
