#!/usr/bin/env bash
# ============================================================================
# Start AME DevEnv with PCL backend (no ROS2 required)
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# tools/devenv/ up 4 levels to repo root (devenv -> tools -> AME -> subprojects -> mujin)
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
PYTHON="${SCRIPT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON}" ]]; then
  echo "Virtual environment not found. Running setup_venv.sh..."
  "${SCRIPT_DIR}/setup_venv.sh"
fi

export PYTHONPATH="${REPO_ROOT}/build/python${PYTHONPATH:+:${PYTHONPATH}}"

DOMAIN_PATH="${REPO_ROOT}/subprojects/AME/domains/uav_search/domain.pddl"
PROBLEM_PATH="${REPO_ROOT}/subprojects/AME/domains/uav_search/problem.pddl"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --domain)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --domain requires a path."
        exit 1
      fi
      DOMAIN_PATH="$2"
      shift 2
      ;;
    --problem)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --problem requires a path."
        exit 1
      fi
      PROBLEM_PATH="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

PY_VER="$("${PYTHON}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")')"
EXT_SUFFIX="$("${PYTHON}" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX") or ".so")')"
AME_PY="${REPO_ROOT}/build/python/_ame_py${EXT_SUFFIX}"

if [[ ! -f "${AME_PY}" ]]; then
  echo
  echo "ERROR: Python bindings not found for venv Python ${PY_VER}"
  echo
  echo "Please build with Python bindings enabled:"
  echo "  cmake --preset ame-devenv -DPython3_EXECUTABLE=\"${PYTHON}\""
  echo "  cmake --build --preset ame-devenv-release"
  echo
  exit 1
fi

echo
echo "Starting AME DevEnv with PCL backend..."
echo "  Python:  ${PYTHON} (${PY_VER})"
echo "  Domain:  ${DOMAIN_PATH}"
echo "  Problem: ${PROBLEM_PATH}"
echo

cd "${REPO_ROOT}"
exec "${PYTHON}" -m subprojects.AME.tools.devenv --backend pcl --domain "${DOMAIN_PATH}" --problem "${PROBLEM_PATH}"
