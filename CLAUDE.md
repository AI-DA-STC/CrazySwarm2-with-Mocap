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
`launch.py` also auto-starts RViz and the **preflight GUI**
(`preflight_kalman_plotter.py` — per-drone go/no-go checks; docs/RUNNING.md §C).

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
  `crazyflies.yaml` has the `kalman_preflight` custom log topic (feeds the
  preflight GUI; 22 B of the 26 B log-block budget — vars must exist in the
  firmware log TOC or the cpp server aborts at connect).
- `src/crazyswarm2/crazyflie/launch/launch.py` — adds the **foxglove_bridge** and
  **preflight GUI** nodes; defaults: `rviz` `True`, `preflight` `True`,
  `foxglove` `True`, `gui` `False` (upstream lacks the extra nodes).
- `src/crazyswarm2/crazyflie/scripts/preflight_kalman_plotter.py` — preflight GUI.
  Constants at the top: `ORIENT_WARN_DEG` 10°, `ERR_STALE_S` 3 s, takeoff/land
  setpoints (0.5 m/3 s, 0.03 m/3 s), `LOG_DIR` = `~/crazyswarm_ws/preflight_logs`
  (hardcoded, NOT under this repo). Keys: `r` reset kalman, `e` broadcast e-stop
  (deliberate); takeoff/land have no keys on purpose.

## Build & run

```bash
# ROS 2 must be installed first (manual; see README Setup Step 1).
./scripts/setup.sh                                  # full setup + build
./scripts/setup_sim_firmware.sh                     # only if using backend:=sim
source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
ros2 launch crazyflie launch.py backend:=sim        # rviz + preflight GUI on by default
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
- **RViz and the preflight GUI are ON by default** in `launch.py`
  (`rviz:=false` / `preflight:=False` to disable). `foxglove:=True` by default
  but needs `ros-$ROS_DISTRO-foxglove-bridge` (installed by `install_deps.sh`);
  view via the Foxglove Studio app, not a window.
- **Mocap "died" / `/poses` 0 publishers** — CONFIRMED cause on this rig: a
  leftover process bound to UDP 1511. Two `SO_REUSEPORT` sockets on 1511 → the
  kernel gives each NatNet datagram to only ONE socket → the mocap node starves
  and hangs silently in libmotioncapture `connect()` (no crash, not in
  `ros2 node list`). Diagnose `ss -uanp | grep :1511`; kill the orphan and
  relaunch. Ping to the Motive PC proves nothing (unicast ≠ multicast).
- **Motive transmission type is read once at connect** (apt
  `motion_capture_tracking` requires Multicast) — after changing it, fully
  restart the launch. A frozen mocap node ignores SIGINT (blocked in `recv`);
  launch escalates to SIGKILL — check `pgrep -f motion_capture_tracking` for
  leftovers (they make the next connect SIGABRT).
- **Rigid-body orientation.** Create the Motive rigid body with the drone's
  forward axis on global +X. `locSrv.extPosStdDev=1e-3` force-fuses mocap
  position, so position error is ~1 mm even with a rotated body — a yaw offset
  is invisible at rest and a fly-away in flight. The preflight GUI's banner /
  `err.yaw` catches it (±5° fly; 5–15° fix; >20° no fly).
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
