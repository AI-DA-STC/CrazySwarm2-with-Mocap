# CLAUDE.md — CrazySwarm2 (self-contained workspace)

Guidance for Claude Code working in this repository. Read before editing or debugging.

## What this repo is

A **self-contained ROS 2 workspace** for an indoor **Crazyflie 2.1 swarm** with
**OptiTrack** mocap, built on [crazyswarm2](https://github.com/IMRCLab/crazyswarm2).
The full customized source is **vendored in `src/`** (committed), so a clone is a
byte-for-byte copy of the rig. `setup.sh` only installs deps and builds — it does
**not** fetch upstream.

- **`src/` IS committed** — edit source/config here directly; a clone reproduces the rig.
- **`build/`, `install/`, `log/`, `core.*` are git-ignored** — never commit them.

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
src/                # VENDORED source (committed)
  crazyswarm2/        # customized: configs, launch.py (foxglove node), scripts, examples
  natnet_ros2/        # OptiTrack driver (+ vendored NatNetSDK)
scripts/
  setup.sh            # install_deps + build (source already present)
  install_deps.sh     # distro-aware apt + rosdep + pip
  build.sh            # colcon wrapper (LOW_MEM=1 for SBCs)
  setup_sim_firmware.sh  # build cffirmware bindings (SIM only)
pose_bridge.py      # natnet → /poses (NamedPoseArray @ 50 Hz)
docs/               # RUNNING, MOCAP, TROUBLESHOOTING
README.md           # single setup doc (no separate SETUP.md)
```

Key customized files inside `src/`:
- `src/crazyswarm2/crazyflie/config/*.yaml` — drone/mocap/server/teleop config.
- `src/crazyswarm2/crazyflie/launch/launch.py` — adds the **foxglove_bridge** node,
  `gui` default `False`, `foxglove` default `True` (upstream lacks the node).

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
- **natnet NatNet SDK** is vendored under `src/natnet_ros2/deps/NatNetSDK/`
  (x86_64). On a different arch the build re-downloads it via `wget` (needs internet).

## Conventions for Claude

- **Edit source/config directly in `src/`** and commit — there is no overlay or
  re-import that would clobber it. A clone reproduces exactly what's committed.
- After editing `src/`, rebuild with `./scripts/build.sh` (or `build.sh <pkg>`),
  then re-source `install/setup.bash`. Rebuild dependents after `.msg`/`.srv` edits.
- `pose_bridge.py` `DRONES` and `PUBLISH_HZ` must match `crazyflies.yaml` and the
  Motive streaming rate (50 Hz).
- Keep scripts distro-parameterized (`ros-${ROS_DISTRO}-…`); never hardcode `jazzy`.
- `gh` is not installed here and pushes need the user's GitHub auth — don't attempt
  to push; report and let the user push. Remotes: `origin`=jeremyCHH, `org`=AI-DA-STC.
```
