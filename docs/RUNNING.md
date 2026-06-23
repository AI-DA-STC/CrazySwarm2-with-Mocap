# RUNNING — launch flows

Activate the workspace in every terminal first:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/CrazySwarm2/install/setup.bash
```

---

## A. Simulation (no hardware, no mocap)

The fastest way to verify the install. Uses the built-in `np` backend.

```bash
ros2 launch crazyflie launch.py backend:=sim
# in another terminal, run an example against the sim:
ros2 launch crazyflie_examples launch.py script:=hello_world
```

RViz opens showing the simulated drone. Backend/controller/visualization options
are in `config/server.yaml` (`sim:` section).

## B. Hardware swarm with OptiTrack

Make sure mocap is calibrated and rigid bodies are defined first
([docs/MOCAP.md](MOCAP.md)). Run these in separate terminals:

```bash
# 1. OptiTrack / NatNet driver  -> /<body>/pose
ros2 launch natnet_ros2 natnet_ros2.launch.py

# 2. Bridge per-body poses -> /poses (NamedPoseArray @ 50 Hz)
python3 ~/CrazySwarm2/pose_bridge.py

# 3. Swarm server (choose a backend)
ros2 launch crazyflie launch.py backend:=cpp     # C++ (crazyflie-link-cpp), lowest latency
#   or
ros2 launch crazyflie launch.py backend:=cflib   # Python (cflib)

# 4. A flight script
ros2 launch crazyflie_examples launch.py script:=hello_world
```

### Verifying the pose pipeline

```bash
ros2 topic hz /cf1/pose      # natnet is publishing this body?
ros2 topic hz /poses         # bridge is forwarding? should be ~50 Hz
ros2 topic echo /cf1/pose --once
```

If `/cf1/pose` is silent → fix Motive/natnet (rigid body name, network, see
[TROUBLESHOOTING](TROUBLESHOOTING.md)). If `/cf1/pose` flows but `/poses` is
silent → check `DRONES` in `pose_bridge.py`.

## C. Manual / teleop flight

```bash
ros2 launch crazyflie launch.py backend:=cpp teleop:=True
```

Gamepad mapping (buttons, axes, limits) is in `config/teleop.yaml`. Defaults:
takeoff = start, land = back, emergency = red, arm = yellow.

## D. Useful checks

```bash
ros2 node list
ros2 topic list
ros2 service list | grep -E 'takeoff|land|go_to|arm'

# battery / link health (status topic must be enabled in crazyflies.yaml)
ros2 topic echo /cf1/status --once
ros2 topic echo /cf1/connection_statistics --once
```

## Launch arguments (crazyflie launch.py)

| Arg | Values | Meaning |
|-----|--------|---------|
| `backend` | `cpp` \| `cflib` \| `sim` | Hardware (C++/Python) or simulation |
| `mocap` | `True` \| `False` | Start the motion_capture_tracking node |
| `teleop` | `True` \| `False` | Start joystick teleop + joy node |
| `rviz` | `True` \| `False` | Start RViz |
| `gui` | `True` \| `False` | Start the swarm GUI |
| `debug` | `True` \| `False` | Launch the C++ server under gdb |
