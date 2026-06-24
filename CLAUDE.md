# CLAUDE.md — CrazySwarm2 (orchestration repo)

Guidance for Claude Code working in this repository. Read before editing or debugging.

## What this repo is

A thin **orchestration / meta-repo** for running an indoor **Crazyflie 2.1 swarm**
with **OptiTrack** mocap, built on [crazyswarm2](https://github.com/IMRCLab/crazyswarm2).
It does **not** vendor upstream source. It holds setup scripts, a tuned config
overlay, the mocap bridge, and docs; the upstream packages are cloned at install
time via a `vcs` manifest.

`src/`, `build/`, `install/`, `log/` are **generated and git-ignored** — never edit
or commit them. The source of truth is this repo's `scripts/`, `config/`, `docs/`,
`crazyswarm2.repos`, and `pose_bridge.py`.

## Architecture (data flow)

```
Motive/OptiTrack ──NatNet Multicast@50Hz──► motion_capture_tracking ──/poses──►
  crazyflie_server ──Crazyradio USB──► Crazyflie 2.1 (cf1, cf2, …)
                    ▲ user scripts via crazyflie_py
Alt mocap path: Motive → natnet_ros2 → /<body>/pose → pose_bridge.py → /poses
```

`motion_capture_tracking` (started by `launch.py`) connects directly to Motive —
the natnet_ros2 + `pose_bridge.py` path is an alternative, not the default.

## Repo layout

```
crazyswarm2.repos   # vcs manifest: crazyswarm2 + natnet_ros2 (pinned); mcap via rosdep
scripts/
  setup.sh            # one-shot: vcs import → submodules → install_deps → overlay → build
  install_deps.sh     # distro-aware apt + rosdep + pip
  build.sh            # colcon wrapper (LOW_MEM=1 for SBCs)
  setup_sim_firmware.sh  # build cffirmware bindings (SIM only)
config/             # tuned config YAMLs, copied over crazyflie/config by setup.sh
overlay/            # custom upstream code (launch.py w/ foxglove, scripts) -> copied over src/
pose_bridge.py      # natnet → /poses (NamedPoseArray @ 50 Hz)
docs/               # RUNNING, MOCAP, TROUBLESHOOTING
README.md           # single setup doc (no separate SETUP.md)
```

## Build & run

```bash
# ROS 2 must be installed first (manual; see README Setup Step 1).
./scripts/setup.sh                                  # full setup + build
./scripts/setup_sim_firmware.sh                     # only if using backend:=sim
source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
ros2 launch crazyflie launch.py backend:=sim rviz:=True   # rviz is OFF by default
ros2 run crazyflie_examples hello_world             # takeoff/hover/land
```
Supported: **Ubuntu 22.04 + Humble** and **24.04 + Jazzy** (auto-detected from
`/etc/os-release`). Tested by running on Jazzy; Humble is verified by inspection.

## Gotchas (hard-won — don't re-derive)

- **`set -u` vs ROS `setup.bash`.** Sourcing `/opt/ros/<distro>/setup.bash` under
  `set -u` aborts on the unbound `AMENT_TRACE_SETUP_FILES`. `build.sh` wraps the
  source in `set +u`/`set -u`. Without it the build silently never runs → empty
  `install/`.
- **cffirmware (simulator only).** `crazyflie_sim` imports `cffirmware` (Crazyflie
  firmware Python bindings) — not a pip package. `setup_sim_firmware.sh` builds it
  from `crazyflie-firmware` (tag 2025.02). NOT needed for hardware backends.
  - Its **CMSIS submodule needs `git-lfs`** or the checkout aborts (`arm_add_f32.c`
    missing). The script installs git-lfs.
  - The `setup.py` egg does **not** bundle the compiled `_cffirmware*.so`; expose
    `crazyflie-firmware/build` on `PYTHONPATH` instead (script appends to `~/.bashrc`).
- **conda shadows system Python.** ROS 2 runs nodes with `/usr/bin/python3`. A conda
  base env (different Python) causes `No module named '_cffirmware'` and rclpy errors.
  Deactivate conda for ROS work. (Intentionally NOT special-cased in the scripts.)
- **RViz is OFF by default** in `launch.py` (`rviz:=False`); pass `rviz:=True`.
  `foxglove:=True` by default but needs `ros-$ROS_DISTRO-foxglove-bridge` (installed
  by `install_deps.sh`); view via the Foxglove Studio app, not a window.
- **ROS apt 404 churn.** `ros-<distro>-{sensor-msgs,tf2-ros,ament-cmake-auto}` can
  404 when apt tries to *upgrade* to a pruned pool version. `install_deps.sh` uses
  `--no-upgrade` for these (desktop already provides them).
- **natnet NatNet SDK** is downloaded at build time via `wget` (needs internet).

## Conventions for Claude

- Edit `config/*.yaml` here, not in `src/` — `setup.sh` overlays them onto the
  imported package, so editing `src/` is lost on re-import.
- Customizations to upstream **code** (not config) go in `overlay/<pkg>/<rel-path>`;
  `setup.sh` copies `overlay/` over `src/` after import. The custom `launch.py`
  (foxglove node, `gui` default False) lives there — pristine upstream lacks it, so
  a fresh setup without the overlay would have no foxglove and `gui` on.
- `pose_bridge.py` `DRONES` and `PUBLISH_HZ` must match `crazyflies.yaml` and the
  Motive streaming rate (50 Hz).
- Keep scripts distro-parameterized (`ros-${ROS_DISTRO}-…`); never hardcode `jazzy`.
- Update upstream pins in `crazyswarm2.repos`, then `vcs import src < crazyswarm2.repos`.
- `gh` is not installed here and pushes need the user's GitHub auth — don't attempt
  to push; report and let the user push. Remotes: `origin`=jeremyCHH, `org`=AI-DA-STC.
```
