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

## B. Hardware flight with OptiTrack

This is the real-world takeoff → hover → land flow for one drone.

The `motion_capture_tracking` node started by `launch.py` connects **directly** to
Motive over the network and publishes `/poses` — you do **not** start a separate
driver or bridge for this path.

### Prerequisites

1. Calibrate the cameras and define a rigid body per drone — see
   [docs/MOCAP.md](MOCAP.md). The rigid-body name must match the robot key in
   `config/crazyflies.yaml` (e.g. `cf1`).
2. In **Motive → Settings → Streaming**, set the data signal type to
   **Multicast** and the rate to **50 Hz**. This **must match**
   `config/motion_capture.yaml` in the crazyswarm2 package
   (`type: optitrack_closed_source`, `poses.qos.deadline: 50.0`). A mismatch here
   is the most common reason the drone won't localize.

### Fly

```bash
# terminal 1 — start the Crazyflie server (also starts mocap tracking, RViz, Foxglove)
ros2 launch crazyflie launch.py

# terminal 2 — takeoff, hover, land
ros2 run crazyflie_examples hello_world
```

The drone arms, takes off, hovers, then lands and disarms.

### Adjusting the flight (hello_world.py)

The behaviour is set by a few values at the top of and inside
`crazyflie_examples/hello_world.py`:

| What | Where in the script | Default |
|------|--------------------|---------|
| **Takeoff duration** (time to climb) | `TAKEOFF_DURATION` | `10.0` s |
| **Hover duration** (time held in the air) | `HOVER_DURATION` | `10.0` s |
| **Hover height** (target altitude) | `cf.takeoff(targetHeight=0.5, ...)` | `0.5` m |
| **Landing height** (touchdown target) | `cf.land(targetHeight=0.03, ...)` | `0.03` m |
| **Landing duration** (time to descend) | `cf.land(..., duration=5.0)` | `5.0` s |

Reading the script: it builds a `Crazyswarm()`, grabs the first drone, then
`arm(True)` → `takeoff(targetHeight, duration)` → `sleep(TAKEOFF + HOVER)` →
`land(targetHeight, duration)` → `arm(False)`. So raise `targetHeight` in
`takeoff()` to hover higher, increase `HOVER_DURATION` to hover longer, and tune
the `land()` arguments for a slower/softer or lower touchdown. Start low (≤0.5 m)
for first flights.

### Visualization

`launch.py` also brings up RViz and the Foxglove bridge:

- **RViz** shows two frames per drone — `cf1` (the **onboard EKF** state estimate)
  and `cf1_mocap` (the raw **mocap** pose). They should sit almost on top of each
  other; a large or growing gap means the estimator and mocap disagree (bad
  calibration, marker/rigid-body issue, or estimator not converged).
- **Foxglove**: open the Foxglove Studio app and connect to the running bridge
  (default `ws://localhost:8765`) to inspect topics, poses, and TF live.

### Verifying the pose pipeline

```bash
ros2 topic hz /poses          # mocap tracking publishing? should be ~50 Hz
ros2 topic echo /cf1/pose --once   # onboard pose estimate (firmware logging)
ros2 topic echo /cf1/status --once # battery / supervisor / link health
```

If `/poses` is silent → fix Motive streaming (Multicast + 50 Hz, Broadcast Frame
on) or the hostname/IP in `config/motion_capture.yaml`. See
[TROUBLESHOOTING](TROUBLESHOOTING.md).

> **Alternative (open NatNet driver).** If you stream via the open-source
> `natnet_ros2` driver instead of the closed-source direct client, start it and
> the bridge separately: `ros2 launch natnet_ros2 natnet_ros2.launch.py` then
> `python3 ~/CrazySwarm2/pose_bridge.py` (republishes per-body poses to `/poses`
> at 50 Hz). The flight steps above are otherwise identical.

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
