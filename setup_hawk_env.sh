#!/usr/bin/env bash
# Recreate hawkEnv with rospy + sensor_msgs for publish_topic_to_arm.py
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
ENV_DIR="${ROOT}/hawkEnv"
PYTHON_BIN="${PYTHON_BIN:-python3.11}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  echo "Need ${PYTHON_BIN} on PATH. Install Python 3.11, or run: PYTHON_BIN=python3 $0"
  exit 1
fi

if [ -d "${ENV_DIR}" ]; then
  echo "Removing existing hawkEnv..."
  rm -rf "${ENV_DIR}"
fi

"${PYTHON_BIN}" -m venv "${ENV_DIR}"
# shellcheck disable=SC1091
source "${ENV_DIR}/bin/activate"

python -m pip install --upgrade pip
python -m pip install --extra-index-url https://rospypi.github.io/simple \
  rospy==1.15.11 \
  sensor-msgs==1.13.0.post3 \
  geometry-msgs==1.13.0.post2

python -c "import rospy; from sensor_msgs.msg import JointState; print('hawkEnv ready:', JointState)"
echo "Activate later with: source ${ENV_DIR}/bin/activate"