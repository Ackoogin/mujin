#!/usr/bin/env bash
# Run ame_ros2 nodes (in-process mode)
# Usage:
#   run_ros2.sh [domain.pddl] [problem.pddl]
#
# Environment overrides:
#   ROS2_SETUP=/path/to/ros2/setup.bash
#   AME_INSTALL_SETUP=/path/to/workspace/install/setup.bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

DOMAIN="${1:-${REPO_ROOT}/subprojects/AME/domains/uav_search/domain.pddl}"
PROBLEM="${2:-${REPO_ROOT}/subprojects/AME/domains/uav_search/problem.pddl}"

if [[ -z "${ROS2_SETUP:-}" ]]; then
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    ROS2_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
    ROS2_SETUP="/opt/ros/humble/setup.bash"
  elif [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    ROS2_SETUP="/opt/ros/jazzy/setup.bash"
  fi
fi

if [[ -z "${ROS2_SETUP:-}" || ! -f "${ROS2_SETUP}" ]]; then
  echo "ERROR: ROS2 setup.bash was not found."
  echo
  echo "Install ROS2 or rerun with:"
  echo "  ROS2_SETUP=/path/to/ros2/setup.bash $0"
  echo
  echo "On Ubuntu 22.04, this project can use ROS2 Humble from /opt/ros/humble."
  exit 1
fi

AME_INSTALL_SETUP="${AME_INSTALL_SETUP:-${REPO_ROOT}/install/setup.bash}"
if [[ ! -f "${AME_INSTALL_SETUP}" ]]; then
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

# Source ROS2 base environment and local ame_ros2 install.
source_setup "${ROS2_SETUP}"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command is still unavailable after sourcing:"
  echo "  ${ROS2_SETUP}"
  exit 1
fi

source_setup "${AME_INSTALL_SETUP}"

echo
echo "Starting ame_combined (in-process mode)"
echo "  domain:  ${DOMAIN}"
echo "  problem: ${PROBLEM}"
echo

exec ros2 run ame_ros2 ame_combined \
  --ros-args \
  -p "domain.pddl_file:=${DOMAIN}" \
  -p "domain.problem_file:=${PROBLEM}"
