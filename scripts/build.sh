#!/usr/bin/env bash
# Build the colcon workspace. Re-run after editing source or pulling updates.
#
# Usage:
#   ./scripts/build.sh                 # full release build
#   ./scripts/build.sh crazyflie       # build a single package
#   LOW_MEM=1 ./scripts/build.sh       # serial build for low-RAM machines
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_ROOT}"

# resolve + source ROS
. /etc/os-release
ROS_DISTRO="${ROS_DISTRO:-}"
if [[ -z "${ROS_DISTRO}" ]]; then
  case "${VERSION_ID}" in 22.04) ROS_DISTRO=humble ;; 24.04) ROS_DISTRO=jazzy ;; esac
fi
if [[ -z "${ROS_DISTRO}" || ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ERROR: Cannot find a supported ROS 2 install. Source /opt/ros/<distro>/setup.bash first." >&2
  exit 1
fi
# ROS 2's setup.bash references unbound vars (e.g. AMENT_TRACE_SETUP_FILES);
# disable nounset across the source so `set -u` doesn't abort the build.
# shellcheck disable=SC1090
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
echo "==> Building against ROS 2 ${ROS_DISTRO}"

# A conda / venv Python on PATH makes CMake and rosidl build the Python message
# bindings for the WRONG interpreter (e.g. cpython-312 .so files on a 22.04 box
# whose `ros2` CLI runs /usr/bin/python3.10). The C++ nodes still work, but every
# `ros2 topic echo` of a workspace message then fails with
# "The message type '.../NamedPoseArray' is invalid". Build with the system Python.
if [[ -n "${CONDA_PREFIX:-}${VIRTUAL_ENV:-}" || "$(command -v python3)" != "/usr/bin/python3" ]]; then
  echo "WARN: python3 is $(command -v python3) (CONDA_PREFIX='${CONDA_PREFIX:-}', VIRTUAL_ENV='${VIRTUAL_ENV:-}')."
  echo "      Stripping conda/venv from PATH for this build so ROS bindings target /usr/bin/python3."
  PATH="$(echo "${PATH}" | tr ':' '\n' | grep -v -E 'conda|/\.?venv/' | paste -sd:)"; export PATH
  # keep ROS entries on PYTHONPATH (colcon's ament extensions need them); drop conda/venv ones
  PYTHONPATH="$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v -E '^$|conda|/\.?venv/' | paste -sd:)"; export PYTHONPATH
  unset CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV PYTHONHOME
fi
PY_ARGS=(-DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3)

COLCON_ARGS=(--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "${PY_ARGS[@]}")
if [[ -n "${LOW_MEM:-}" ]]; then
  COLCON_ARGS=(--symlink-install --parallel-workers 2 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release "${PY_ARGS[@]}")
  export MAKEFLAGS="-j2"
  echo "==> LOW_MEM mode (serial, -j2)"
fi

if [[ $# -gt 0 ]]; then
  colcon build --packages-select "$@" "${COLCON_ARGS[@]}"
else
  colcon build "${COLCON_ARGS[@]}"
fi

echo
echo "==> Build complete. Activate the workspace with:"
echo "    source ${WS_ROOT}/install/setup.bash"
