#!/usr/bin/env bash
# Install system + ROS dependencies for the CrazySwarm2 stack.
# Distro-aware: supports Ubuntu 22.04 + ROS 2 Humble and Ubuntu 24.04 + ROS 2 Jazzy.
#
# Safe to re-run. Assumes ROS 2 is already installed (see docs/SETUP.md if not).
set -euo pipefail

# --- resolve ROS distro -------------------------------------------------------
. /etc/os-release   # provides VERSION_ID, e.g. "22.04" / "24.04"

detect_distro() {
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    echo "${ROS_DISTRO}"; return
  fi
  case "${VERSION_ID}" in
    22.04) echo "humble" ;;
    24.04) echo "jazzy"  ;;
    *) echo "" ;;
  esac
}

ROS_DISTRO="$(detect_distro)"
if [[ -z "${ROS_DISTRO}" ]]; then
  echo "ERROR: Unsupported Ubuntu ${VERSION_ID}. Supported: 22.04 (Humble), 24.04 (Jazzy)." >&2
  echo "       Or 'source /opt/ros/<distro>/setup.bash' first to set ROS_DISTRO." >&2
  exit 1
fi
if [[ ! -d "/opt/ros/${ROS_DISTRO}" ]]; then
  echo "ERROR: /opt/ros/${ROS_DISTRO} not found. Install ROS 2 ${ROS_DISTRO} first (docs/SETUP.md)." >&2
  exit 1
fi
echo "==> Target: Ubuntu ${VERSION_ID}, ROS 2 ${ROS_DISTRO}"

# --- apt build + runtime deps -------------------------------------------------
echo "==> Installing apt packages"
sudo apt-get update
sudo apt-get install -y \
  git cmake build-essential \
  python3-colcon-common-extensions python3-vcstool python3-rosdep python3-pip \
  libusb-1.0-0-dev libboost-program-options-dev libeigen3-dev \
  ros-"${ROS_DISTRO}"-tf-transformations \
  ros-"${ROS_DISTRO}"-motion-capture-tracking || {
    echo "WARN: ros-${ROS_DISTRO}-motion-capture-tracking not found via apt."
    echo "      Uncomment the motion_capture_tracking entry in crazyswarm2.repos and re-run setup.sh."
  }

# cflib is needed only for the Python (cflib) backend; harmless otherwise.
pip3 install --user cflib cfclient 2>/dev/null || pip3 install --user --break-system-packages cflib cfclient || true

# --- rosdep -------------------------------------------------------------------
echo "==> Running rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update
# Resolve declared package deps (incl. motion_capture_tracking_interfaces) from src/.
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -d "${WS_ROOT}/src" ]]; then
  rosdep install --from-paths "${WS_ROOT}/src" --ignore-src -r -y --rosdistro "${ROS_DISTRO}" || \
    echo "WARN: some rosdep keys unresolved; see output above."
fi

echo "==> Dependencies installed for ROS 2 ${ROS_DISTRO}."
