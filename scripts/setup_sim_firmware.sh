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

echo "==> [1/4] Build dependencies (swig, gcc, python3-dev, git-lfs)"
sudo apt-get update
# git-lfs is REQUIRED: the CMSIS submodule stores files via Git LFS, and without
# it the submodule checkout aborts, leaving CMSIS DSP sources missing.
sudo apt-get install -y swig build-essential python3-dev git git-lfs
git lfs install

echo "==> [2/4] Fetch crazyflie-firmware ${FW_TAG} -> ${FW_DIR}"
if [[ -d "${FW_DIR}/.git" ]]; then
  git -C "${FW_DIR}" fetch --tags
  git -C "${FW_DIR}" checkout "${FW_TAG}"
else
  git clone --branch "${FW_TAG}" --single-branch \
    https://github.com/bitcraze/crazyflie-firmware.git "${FW_DIR}"
fi
# --force re-checks-out submodules left broken by an earlier no-LFS attempt.
git -C "${FW_DIR}" submodule sync --recursive
git -C "${FW_DIR}" submodule update --init --recursive --force

echo "==> [3/4] Build Python bindings"
cd "${FW_DIR}"
make cf2_defconfig
make bindings_python

echo "==> [4/4] Expose cffirmware on PYTHONPATH"
# `make bindings_python` put cffirmware.py AND the compiled _cffirmware*.so in
# build/. `setup.py install` does NOT reliably bundle the .so (the egg ends up
# missing _cffirmware -> the sim dies with "No module named _cffirmware"), so we
# add the build dir to PYTHONPATH instead — the method crazyswarm2 documents.
BUILD_DIR="${FW_DIR}/build"
LINE="export PYTHONPATH=\"${BUILD_DIR}:\${PYTHONPATH:-}\""
grep -qF "${BUILD_DIR}" "${HOME}/.bashrc" 2>/dev/null || echo "${LINE}" >> "${HOME}/.bashrc"
export PYTHONPATH="${BUILD_DIR}:${PYTHONPATH:-}"

echo
# Verify from a NEUTRAL directory (not build/, which would shadow the real path).
if (cd "${HOME}" && python3 -c "import cffirmware" 2>/dev/null); then
  echo "==> cffirmware OK. Run 'source ~/.bashrc' (or open a new shell) before launching the sim."
else
  echo "ERROR: cffirmware not importable. Check that ${BUILD_DIR} holds _cffirmware*.so." >&2
  exit 1
fi
