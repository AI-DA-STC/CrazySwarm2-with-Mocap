# SETUP — installing from a bare machine

End-to-end install for **Ubuntu 22.04 + ROS 2 Humble** and **Ubuntu 24.04 + ROS 2
Jazzy**. If ROS 2 is already installed, skip to [step 2](#2-clone-and-run-setup).

---

## 1. Install ROS 2

This repo supports two Ubuntu/ROS 2 pairings (for reference):

| Ubuntu | ROS 2 distro |
|--------|--------------|
| 22.04 (Jammy) | Humble |
| 24.04 (Noble) | Jazzy |

You don't need to choose manually — the snippet below detects your Ubuntu version
and sets `ROS_DISTRO` for you. Just copy-paste the whole block.

```bash
# --- auto-detect the ROS 2 distro from your Ubuntu version ---
source /etc/os-release
case "$VERSION_ID" in
  22.04) export ROS_DISTRO=humble ;;
  24.04) export ROS_DISTRO=jazzy  ;;
  *) echo "Unsupported Ubuntu $VERSION_ID — need 22.04 or 24.04"; return 2>/dev/null || exit 2 ;;
esac
echo "Detected Ubuntu $VERSION_ID -> installing ROS 2 $ROS_DISTRO"

# --- enable the ROS 2 apt repository ---
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- install ROS 2 (desktop = includes RViz) ---
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-desktop ros-dev-tools

# --- source it (add to ~/.bashrc to make permanent) ---
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
```

> Reference: the official ROS 2 install guide for
> [Humble](https://docs.ros.org/en/humble/Installation.html) /
> [Jazzy](https://docs.ros.org/en/jazzy/Installation.html).

## 2. Clone and run setup

```bash
git clone https://github.com/jeremyCHH/CrazySwarm2.git ~/CrazySwarm2
cd ~/CrazySwarm2
./scripts/setup.sh
```

`setup.sh` runs, in order:

1. **`vcs import src < crazyswarm2.repos`** — clones the pinned upstream packages.
2. **`git submodule update --init --recursive`** — pulls crazyswarm2 submodules
   (`crazyflie_tools`, link/firmware bindings).
3. **`scripts/install_deps.sh`** — apt deps + `rosdep install` (this is where
   `motion_capture_tracking` is pulled from apt).
4. **config overlay** — copies `config/*.yaml` over the upstream defaults.
5. **`scripts/build.sh`** — `colcon build --symlink-install`.

When it finishes:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/CrazySwarm2/install/setup.bash
ros2 launch crazyflie launch.py backend:=sim   # smoke test, no hardware
```

## 3. Crazyradio USB permissions (hardware only)

So your user can talk to the Crazyradio without `sudo`:

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

## 4. Notes per distro

- **Both distros** are handled by the same scripts; the only difference is the
  `ros-<distro>-*` package names, which are resolved automatically.
- If `rosdep` reports it cannot find **`motion-capture-tracking`** on your distro,
  uncomment the `motion_capture_tracking` entry in
  [`crazyswarm2.repos`](../crazyswarm2.repos) and re-run `./scripts/setup.sh` to
  build it from source instead.
- **Low-RAM machines** (e.g. SBCs): build with `LOW_MEM=1 ./scripts/build.sh`.
- **Optional SITL** (software-in-the-loop firmware) is not required for sim, which
  uses the built-in `np` backend. See upstream crazyswarm2 docs if you need it.
