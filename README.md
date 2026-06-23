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
        │  NatNet — Multicast @ 50 Hz
        ▼
  motion_capture_tracking  ──►  /poses  (NamedPoseArray)   [started by launch.py]
        │
        ▼
  crazyflie_server  ──►  Crazyradio USB ──►  Crazyflie 2.1 (cf1, cf2, …)
        ▲
        └── user scripts via the crazyflie_py API

  Alternative mocap path (open driver):
    Motive → natnet_ros2 → /<body>/pose → pose_bridge.py → /poses
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

## Setup

The full install lives here — there is no separate setup doc. `./scripts/setup.sh`
automates the workspace build, but **two things are deliberately left manual**
(marked below) because they are one-time, system-wide changes: installing ROS 2,
and Crazyradio USB permissions.

### Step 1 — Install ROS 2  *(manual; `setup.sh` does NOT do this)*

`setup.sh` requires ROS 2 to already be installed and will stop with an error if
it isn't. Install it once. The block below auto-detects the right distro
(Humble on 22.04, Jazzy on 24.04) — just copy-paste it whole:

```bash
# auto-detect the ROS 2 distro from your Ubuntu version
source /etc/os-release
case "$VERSION_ID" in
  22.04) export ROS_DISTRO=humble ;;
  24.04) export ROS_DISTRO=jazzy  ;;
  *) echo "Unsupported Ubuntu $VERSION_ID — need 22.04 or 24.04"; return 2>/dev/null || exit 2 ;;
esac
echo "Installing ROS 2 $ROS_DISTRO"

# enable the ROS 2 apt repository
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install ROS 2 (desktop = includes RViz) and dev tools
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-desktop ros-dev-tools

# source it now and on every new shell
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
```

> Reference: official ROS 2 install guide —
> [Humble](https://docs.ros.org/en/humble/Installation.html) /
> [Jazzy](https://docs.ros.org/en/jazzy/Installation.html).

### Step 2 — Clone and run the one-shot setup

```bash
git clone https://github.com/jeremyCHH/CrazySwarm2.git ~/CrazySwarm2
cd ~/CrazySwarm2
./scripts/setup.sh
```

`setup.sh` runs, in order:

1. `vcs import src < crazyswarm2.repos` — clone the pinned upstream packages.
2. `git submodule update --init --recursive` — crazyswarm2 submodules (`crazyflie_tools`, …).
3. `scripts/install_deps.sh` — apt deps + `rosdep install` (pulls `motion_capture_tracking`).
4. config overlay — copy `config/*.yaml` over the upstream defaults.
5. `scripts/build.sh` — `colcon build --symlink-install`.

Re-run any step on its own later:

```bash
./scripts/install_deps.sh        # deps only
./scripts/build.sh               # rebuild everything
./scripts/build.sh crazyflie     # rebuild one package
LOW_MEM=1 ./scripts/build.sh     # serial build on low-RAM machines
```

### Step 3 — Crazyradio USB permissions  *(manual; hardware only)*

Skip this if you only run the simulator. It lets your user talk to the Crazyradio
without `sudo` — `setup.sh` does NOT do this:

```bash
sudo groupadd plugdev 2>/dev/null; sudo usermod -aG plugdev $USER
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
# Crazyradio (PA) and Crazyradio 2.0
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie 2.x over USB
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
# log out / back in for the group change to take effect
```

Reference: [Bitcraze USB permissions guide](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/).

### Step 4 — Simulator firmware bindings (`cffirmware`)  *(simulator only)*

The simulator backend (`backend:=sim`) imports the Crazyflie firmware Python
bindings (`cffirmware`) for software-in-the-loop control. These are **not** a pip
package and are **not** needed for hardware flight — only for the simulator. Build
them once (clones crazyflie-firmware outside the workspace, builds + installs the
bindings into your user site-packages):

```bash
./scripts/setup_sim_firmware.sh
source ~/.bashrc                       # the script adds the bindings to PYTHONPATH
cd ~ && python3 -c "import cffirmware; print('cffirmware OK')"   # verify (not from build/)
```

The script installs `git-lfs` (the firmware's CMSIS submodule needs it) and adds
the compiled bindings to `PYTHONPATH` via `~/.bashrc`, so open a new shell (or
`source ~/.bashrc`) before launching the simulator.

### Step 5 — Activate and smoke-test

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/CrazySwarm2/install/setup.bash
ros2 launch crazyflie launch.py backend:=sim   # needs Step 4; no hardware/mocap
```

> **Building by hand.** `setup.sh` already builds the workspace, but to (re)build
> manually — e.g. after editing source, or to see raw colcon output:
> ```bash
> cd ~/CrazySwarm2
> source /opt/ros/$ROS_DISTRO/setup.bash
> colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
> source install/setup.bash
> ```
>
> **Notes.** If `rosdep` can't find `motion-capture-tracking` on your distro,
> uncomment its entry in [`crazyswarm2.repos`](crazyswarm2.repos) and re-run
> `setup.sh`. On low-RAM machines use `LOW_MEM=1 ./scripts/build.sh`.

## Running the real drone

First, in **Motive** set Data Streaming to **Multicast** at **50 Hz** — this must
match `config/motion_capture.yaml`. Then:

```bash
# terminal 1 — Crazyflie server (also starts mocap tracking + Foxglove bridge)
# RViz is OFF by default — add rviz:=True for the window
ros2 launch crazyflie launch.py rviz:=True

# terminal 2 — takeoff, hover, land
ros2 run crazyflie_examples hello_world
```

With `rviz:=True` you should see `cf1` (onboard EKF estimate) and `cf1_mocap`
(mocap) overlapping. Foxglove is on by default but needs the bridge package
(`sudo apt install ros-$ROS_DISTRO-foxglove-bridge`) and is viewed from the
Foxglove Studio app. Hover/landing height and durations are set in
`hello_world.py` — see
[docs/RUNNING.md](docs/RUNNING.md#adjusting-the-flight-hello_worldpy).

## Documentation

- [docs/RUNNING.md](docs/RUNNING.md) — sim, hardware, mocap launch flows; custom logging
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
├── docs/                 # RUNNING, MOCAP, TROUBLESHOOTING
└── src/  build/  install/  log/   # generated, git-ignored
```

## Updating upstream

Edit the pinned `version:` in `crazyswarm2.repos`, then:

```bash
vcs import src < crazyswarm2.repos   # checks out the new refs
./scripts/build.sh
```
