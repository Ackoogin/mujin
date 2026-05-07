#!/usr/bin/env bash
# AME Dev Environment - launch script
# Usage: start_devenv.sh [--foxglove-url URL] [--no-ros2] [--width W] [--height H]
# Run setup_venv.sh once first if the .venv directory does not exist.
#
# Environment overrides:
#   ROS2_SETUP=/path/to/ros2/setup.bash
#   AME_INSTALL_SETUP=/path/to/workspace/install/setup.bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tools/devenv/ up 4 levels to repo root (devenv -> tools -> AME -> subprojects -> mujin)
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
PYTHON="${SCRIPT_DIR}/.venv/bin/python"

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

if [[ ! -x "${PYTHON}" ]]; then
  echo "Virtual environment not found. Running setup_venv.sh..."
  "${SCRIPT_DIR}/setup_venv.sh"
fi

NEEDS_ROS2=1
ORIG_ARGS=("$@")
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-ros2)
      NEEDS_ROS2=0
      ;;
    --backend)
      if [[ "${2:-}" == "none" || "${2:-}" == "pcl" ]]; then
        NEEDS_ROS2=0
      fi
      shift
      ;;
  esac
  shift || true
done
set -- "${ORIG_ARGS[@]}"

if [[ -z "${ROS2_SETUP:-}" ]]; then
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    ROS2_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
    ROS2_SETUP="/opt/ros/humble/setup.bash"
  elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    ROS2_SETUP="/opt/ros/jazzy/setup.bash"
  fi
fi

if [[ "${NEEDS_ROS2}" -eq 1 && ( -z "${ROS2_SETUP:-}" || ! -f "${ROS2_SETUP}" ) ]]; then
  echo "ERROR: ROS2 setup.bash was not found."
  echo
  echo "Install ROS2 or rerun with:"
  echo "  ROS2_SETUP=/path/to/ros2/setup.bash $0"
  echo
  echo "Use --no-ros2 for offline mode, or start_devenv_pcl.sh for the PCL backend."
  exit 1
fi

AME_INSTALL_SETUP="${AME_INSTALL_SETUP:-${REPO_ROOT}/install/setup.bash}"
if [[ "${NEEDS_ROS2}" -eq 1 && ! -f "${AME_INSTALL_SETUP}" ]]; then
  echo "ERROR: AME ROS2 install setup was not found:"
  echo "  ${AME_INSTALL_SETUP}"
  echo
  echo "Build and install ame_ros2 first, then rerun this script."
  exit 1
fi

# Pin CycloneDDS to one interface (avoids multi-NIC reply confusion).
export CYCLONEDDS_URI="file://${REPO_ROOT}/cyclonedds_localhost.xml"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${REPO_ROOT}/log/ros}"
mkdir -p "${ROS_LOG_DIR}"

if [[ "${NEEDS_ROS2}" -eq 1 ]]; then
  # Source ROS2 base environment and local ame_ros2 install.
  source_setup "${ROS2_SETUP}"
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 command is still unavailable after sourcing:"
    echo "  ${ROS2_SETUP}"
    exit 1
  fi

  source_setup "${AME_INSTALL_SETUP}"
fi

export PYTHONPATH="${REPO_ROOT}/build/python${PYTHONPATH:+:${PYTHONPATH}}"

cd "${REPO_ROOT}"
exec "${PYTHON}" -m subprojects.AME.tools.devenv "$@"
