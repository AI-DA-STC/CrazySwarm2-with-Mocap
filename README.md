# CrazySwarm2

Reproducible setup for an indoor **Crazyflie 2.1 swarm** flown with **OptiTrack**
motion-capture localization, built on [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2).

This is a thin **orchestration repo**: it does not vendor the upstream source.
Instead it holds the setup scripts, a tuned config overlay, the mocap bridge, and
documentation, and pulls the upstream packages at install time via a
[`vcs`](http://wiki.ros.org/vcstool) manifest. One command sets everything up on
**Ubuntu 22.04 + ROS 2 Humble** or **Ubuntu 24.04 + ROS 2 Jazzy**.

## Architecture

```
Motive / OptiTrack server (Windows)
        │  NatNet (UDP multicast 239.255.42.99, ports 1510/1511)
        ▼
  natnet_ros2_node ──►  /<body>/pose  (PoseStamped, one per rigid body)
        │
        ▼
  pose_bridge.py    ──►  /poses  (NamedPoseArray @ 50 Hz)
        │
        ▼
  crazyflie_server  ──►  Crazyradio USB ──►  Crazyflie 2.1 (cf1, cf2, …)
        ▲
        └── user scripts via the crazyflie_py API
```

| Component | Source | Role |
|-----------|--------|------|
| `crazyswarm2` | github.com/IMRCLab/crazyswarm2 | Crazyflie swarm server, sim, examples, Python API |
| `natnet_ros2` | github.com/L2S-lab/natnet_ros2 | OptiTrack / NatNet driver |
| `motion_capture_tracking` | apt (`rosdep`) | Converts mocap data → `/poses` for the server |
| `pose_bridge.py` | this repo | Aggregates per-body poses into `NamedPoseArray` |
| `config/` | this repo | Tuned `crazyflies/motion_capture/server/teleop` YAML overlay |

## Supported platforms

| Ubuntu | ROS 2 | Status |
|--------|-------|--------|
| 22.04 (Jammy) | Humble | supported |
| 24.04 (Noble) | Jazzy | supported |

The scripts auto-detect the pair from `/etc/os-release` (or an already-sourced
`$ROS_DISTRO`). Other combinations are rejected with a clear error.

## Quick start

```bash
# 0. Install ROS 2 first if you haven't — see docs/SETUP.md.
git clone https://github.com/jeremyCHH/CrazySwarm2.git ~/CrazySwarm2
cd ~/CrazySwarm2

# 1. One-shot: import sources, install deps, apply config overlay, build.
./scripts/setup.sh

# 2. Activate the workspace (every new shell).
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/CrazySwarm2/install/setup.bash

# 3. Smoke-test in simulation (no hardware/mocap needed).
ros2 launch crazyflie launch.py backend:=sim
```

Re-run individual steps anytime:

```bash
./scripts/install_deps.sh        # deps only
./scripts/build.sh               # rebuild everything
./scripts/build.sh crazyflie     # rebuild one package
LOW_MEM=1 ./scripts/build.sh     # serial build on low-RAM machines
```

## Running the real swarm

```bash
# terminal 1 — OptiTrack driver
ros2 launch natnet_ros2 natnet_ros2.launch.py

# terminal 2 — mocap → /poses bridge
python3 ~/CrazySwarm2/pose_bridge.py

# terminal 3 — swarm server (C++ backend)
ros2 launch crazyflie launch.py backend:=cpp

# terminal 4 — a flight script
ros2 launch crazyflie_examples launch.py script:=hello_world
```

Full details and the mocap calibration / rigid-body / frequency tuning guide:

- [docs/SETUP.md](docs/SETUP.md) — per-distro install from a bare machine
- [docs/RUNNING.md](docs/RUNNING.md) — sim, hardware, and mocap launch flows
- [docs/MOCAP.md](docs/MOCAP.md) — OptiTrack calibration, rigid bodies, 240→50 Hz tuning
- [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) — common failures

## Configuration

Your tuned configs live in [`config/`](config/) and are copied over the upstream
defaults by `setup.sh`. Edit them here (not in `src/`) so a fresh clone reproduces
your rig. Key files:

- `config/crazyflies.yaml` — drone list, URIs, types, firmware logging rates.
- `config/motion_capture.yaml` — Motive hostname/IP, marker layouts, QoS.
- `config/server.yaml` — warning thresholds, sim backend/controller.
- `config/teleop.yaml` — gamepad mapping.

`pose_bridge.py` `DRONES` and `PUBLISH_HZ` must match `crazyflies.yaml` and the
Motive streaming rate — see [docs/MOCAP.md](docs/MOCAP.md).

## Repo layout

```
CrazySwarm2/
├── crazyswarm2.repos     # vcs manifest (pinned upstream refs)
├── scripts/              # setup.sh, install_deps.sh, build.sh
├── config/               # tuned YAML overlay
├── pose_bridge.py        # natnet → /poses bridge (50 Hz)
├── docs/                 # SETUP, RUNNING, MOCAP, TROUBLESHOOTING
└── src/  build/  install/  log/   # generated, git-ignored
```

## Updating upstream

Edit the pinned `version:` in `crazyswarm2.repos`, then:

```bash
vcs import src < crazyswarm2.repos   # checks out the new refs
./scripts/build.sh
```
