#!/usr/bin/env bash
# One-shot setup for the CrazySwarm2 stack.
#   1. detect distro (Ubuntu 22.04/Humble or 24.04/Jazzy)
#   2. import upstream sources (vcs)               -> ./src
#   3. init git submodules (crazyflie_tools, ...)
#   4. install apt + rosdep deps                   -> scripts/install_deps.sh
#   5. apply the config overlay                    -> ./config -> crazyflie/config
#   6. build                                       -> scripts/build.sh
#
# Idempotent: safe to re-run. Each step can also be run on its own.
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
  echo "       See docs/SETUP.md for the ROS 2 install steps, then re-run." >&2
  exit 1
fi
echo "=================================================================="
echo " CrazySwarm2 setup  |  Ubuntu ${VERSION_ID}  |  ROS 2 ${ROS_DISTRO}"
echo "=================================================================="

# --- 1+2. import sources ------------------------------------------------------
mkdir -p src
echo "==> [1/5] Importing upstream sources (vcs import)"
sudo apt-get update -qq && sudo apt-get install -y -qq python3-vcstool >/dev/null
vcs import src < crazyswarm2.repos

# --- 3. submodules ------------------------------------------------------------
echo "==> [2/5] Initialising git submodules"
for d in src/*/; do
  if [[ -f "${d}.gitmodules" ]]; then
    git -C "${d}" submodule update --init --recursive
  fi
done

# --- 4. dependencies ----------------------------------------------------------
echo "==> [3/5] Installing dependencies"
ROS_DISTRO="${ROS_DISTRO}" bash "${WS_ROOT}/scripts/install_deps.sh"

# --- 5. config overlay --------------------------------------------------------
echo "==> [4/5] Applying config overlay"
CFG_DST="src/crazyswarm2/crazyflie/config"
if [[ -d "${CFG_DST}" ]]; then
  for f in crazyflies.yaml motion_capture.yaml server.yaml teleop.yaml; do
    if [[ -f "config/${f}" ]]; then
      cp -v "config/${f}" "${CFG_DST}/${f}"
    fi
  done
else
  echo "WARN: ${CFG_DST} not found; skipping overlay (did vcs import succeed?)."
fi

# --- 6. build -----------------------------------------------------------------
echo "==> [5/5] Building workspace"
bash "${WS_ROOT}/scripts/build.sh"

cat <<EOF

==================================================================
 Done. To use the workspace in a new shell:

   source /opt/ros/${ROS_DISTRO}/setup.bash
   source ${WS_ROOT}/install/setup.bash

 Try the simulator first (no hardware needed):
   ros2 launch crazyflie launch.py backend:=sim

 See docs/RUNNING.md for the hardware + mocap flow.
==================================================================
EOF
