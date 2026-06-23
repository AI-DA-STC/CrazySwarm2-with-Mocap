#!/usr/bin/env bash
# Build + install the Crazyflie firmware Python bindings (cffirmware).
#
# The SIMULATOR backend (crazyflie_sim, i.e. `launch.py backend:=sim`) imports
# `cffirmware` for software-in-the-loop control. It is NOT a pip package and is
# NOT needed for hardware flight (backend:=cpp / cflib) — only for the simulator.
#
# Installs cffirmware into your user site-packages, so no PYTHONPATH is required.
# Idempotent. Clones crazyflie-firmware OUTSIDE the ROS workspace.
set -euo pipefail

FW_DIR="${CF_FIRMWARE_DIR:-$HOME/crazyflie-firmware}"
FW_TAG="2025.02"   # latest crazyswarm2-tested firmware release

echo "==> [1/4] Build dependencies (swig, gcc, python3-dev)"
sudo apt-get update
sudo apt-get install -y swig build-essential python3-dev git

echo "==> [2/4] Fetch crazyflie-firmware ${FW_TAG} -> ${FW_DIR}"
if [[ -d "${FW_DIR}/.git" ]]; then
  git -C "${FW_DIR}" fetch --tags
  git -C "${FW_DIR}" checkout "${FW_TAG}"
  git -C "${FW_DIR}" submodule sync
  git -C "${FW_DIR}" submodule update --init --recursive
else
  git clone --branch "${FW_TAG}" --single-branch --recursive \
    https://github.com/bitcraze/crazyflie-firmware.git "${FW_DIR}"
fi

echo "==> [3/4] Build Python bindings"
cd "${FW_DIR}"
make cf2_defconfig
make bindings_python

echo "==> [4/4] Install cffirmware into user site-packages"
cd build
python3 setup.py install --user \
  || pip3 install --user . \
  || pip3 install --user --break-system-packages .

echo
if python3 -c "import cffirmware" 2>/dev/null; then
  echo "==> cffirmware OK. The simulator (backend:=sim) can now run."
else
  echo "WARN: cffirmware not importable yet. Add this to your shell as a fallback:"
  echo "      export PYTHONPATH=${FW_DIR}/build:\$PYTHONPATH"
  exit 1
fi
