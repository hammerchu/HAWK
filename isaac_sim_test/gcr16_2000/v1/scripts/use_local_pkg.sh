#!/usr/bin/env bash
# Source this so package://gcr16_2000_v1/... resolves (RViz, xacro, MoveIt Setup Assistant).
#   source ./scripts/use_local_pkg.sh
V1="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${V1}/.ament_prefix"
SHARE="${PREFIX}/share/gcr16_2000_v1"
mkdir -p "${SHARE}"
for item in meshes urdf launch rviz scripts; do
  rm -rf "${SHARE}/${item}"
  if [ -e "${V1}/${item}" ]; then
    ln -sfn "${V1}/${item}" "${SHARE}/${item}"
  fi
done
cp -f "${V1}/package.xml" "${SHARE}/package.xml"
mkdir -p "${PREFIX}/share/ament_index/resource_index/packages"
: > "${PREFIX}/share/ament_index/resource_index/packages/gcr16_2000_v1"
export AMENT_PREFIX_PATH="${PREFIX}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
echo "AMENT_PREFIX_PATH prepended with ${PREFIX}"
echo "share -> ${SHARE} (meshes/urdf linked)"
ls -lh "${V1}/meshes"/GCR16-*.stl 2>/dev/null || echo "No STLs yet."
