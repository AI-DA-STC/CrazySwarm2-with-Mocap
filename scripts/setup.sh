#!/usr/bin/env bash
# One-shot setup for the CrazySwarm2 workspace.
#
# The source is VENDORED in this repo (src/), so setup only needs to:
#   1. detect distro (Ubuntu 22.04/Humble or 24.04/Jazzy)
#   2. install apt + rosdep deps   -> scripts/install_deps.sh
#   3. build                       -> scripts/build.sh
#
# For the simulator (backend:=sim) also run scripts/setup_sim_firmware.sh once.
# Idempotent: safe to re-run.
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_ROOT}"

. /etc/os-release
ROS_DISTRO="${ROS_DISTRO:-}"
if [[ -z "${ROS_DISTRO}" ]]; then
  case "${VERSION_ID}" in
    22.04) ROS_DISTRO=humble ;;
    24.04) ROS_DISTRO=jazzy  ;;
    *) echo "ERROR: Unsupported Ubuntu ${VERSION_ID} (need 22.04 or 24.04)." >&2; exit 1 ;;
  esac
fi
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ERROR: ROS 2 ${ROS_DISTRO} not installed at /opt/ros/${ROS_DISTRO}." >&2
  echo "       See README 'Setup Step 1' for the ROS 2 install steps, then re-run." >&2
  exit 1
fi
echo "=================================================================="
echo " CrazySwarm2 setup  |  Ubuntu ${VERSION_ID}  |  ROS 2 ${ROS_DISTRO}"
echo "=================================================================="

if [[ ! -d src/crazyswarm2 ]]; then
  echo "ERROR: src/crazyswarm2 not found. This repo should contain the vendored" >&2
  echo "       source — did the clone complete?" >&2
  exit 1
fi

# --- 1. dependencies ----------------------------------------------------------
echo "==> [1/2] Installing dependencies"
ROS_DISTRO="${ROS_DISTRO}" bash "${WS_ROOT}/scripts/install_deps.sh"

# --- 2. build -----------------------------------------------------------------
echo "==> [2/2] Building workspace"
bash "${WS_ROOT}/scripts/build.sh"

cat <<EOF

==================================================================
 Done. To use the workspace in a new shell:

   source /opt/ros/${ROS_DISTRO}/setup.bash
   source ${WS_ROOT}/install/setup.bash

 Simulator (one-time extra step for backend:=sim):
   ./scripts/setup_sim_firmware.sh        # builds the cffirmware bindings
   ros2 launch crazyflie launch.py backend:=sim rviz:=True

 Hardware: see docs/RUNNING.md.
==================================================================
EOF
