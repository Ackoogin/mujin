#!/usr/bin/env bash
# AME Dev Environment - one-time virtual environment setup
# Run from the repository root or from subprojects/AME/tools/devenv/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
VENV_PYTHON="${VENV_DIR}/bin/python"

if [[ -n "${ROS2_PYTHON:-}" && -x "${ROS2_PYTHON}" ]]; then
  BOOTSTRAP_PYTHON="${ROS2_PYTHON}"
elif command -v python3 >/dev/null 2>&1; then
  BOOTSTRAP_PYTHON="$(command -v python3)"
elif command -v python >/dev/null 2>&1; then
  BOOTSTRAP_PYTHON="$(command -v python)"
else
  echo "ERROR: python3 or python was not found on PATH."
  exit 1
fi

BOOTSTRAP_VERSION="$("${BOOTSTRAP_PYTHON}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"

# Recreate the venv if it was built against a different Python minor version.
if [[ -f "${VENV_DIR}/pyvenv.cfg" ]]; then
  VENV_VERSION="$(sed -n 's/^version = \([0-9][0-9]*\.[0-9][0-9]*\).*/\1/p' "${VENV_DIR}/pyvenv.cfg" | head -n 1)"
  if [[ -n "${VENV_VERSION}" && "${VENV_VERSION}" != "${BOOTSTRAP_VERSION}" ]]; then
    echo "Rebuilding virtual environment for Python ${BOOTSTRAP_VERSION}..."
    rm -rf "${VENV_DIR}"
  elif [[ -x "${VENV_PYTHON}" ]] &&
       ! "${VENV_PYTHON}" -m pip --version >/dev/null 2>&1; then
    echo "Rebuilding incomplete virtual environment..."
    rm -rf "${VENV_DIR}"
  fi
fi

# Create venv at subprojects/AME/tools/devenv/.venv
if [[ ! -d "${VENV_DIR}" ]]; then
  echo "Creating virtual environment..."
  "${BOOTSTRAP_PYTHON}" -m venv "${VENV_DIR}"
else
  echo "Virtual environment already exists."
fi

echo "Installing dependencies..."
"${VENV_PYTHON}" -m pip install --upgrade pip --quiet
"${VENV_PYTHON}" -m pip install -r "${SCRIPT_DIR}/requirements.txt" --quiet

echo
echo "Setup complete."
echo "Run subprojects/AME/tools/devenv/start_devenv.sh to launch the ROS2 backend."
echo "Run subprojects/AME/tools/devenv/start_devenv_pcl.sh to launch the PCL backend."
