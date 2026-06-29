# RUNNING — launch flows

Activate the workspace in every terminal first:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/CrazySwarm2/install/setup.bash
```

> Config files referred to below as `config/<name>.yaml` live at
> `src/crazyswarm2/crazyflie/config/<name>.yaml`. Edit them there and rebuild.

---

## A. Simulation (no hardware, no mocap)

The fastest way to verify the install. Uses the built-in `np` backend.

> **One-time prerequisite:** the sim imports the `cffirmware` bindings. If you see
> `ModuleNotFoundError: No module named 'cffirmware'`, run
> `./scripts/setup_sim_firmware.sh` once (see README → Setup Step 4).

```bash
ros2 launch crazyflie launch.py backend:=sim rviz:=True   # rviz is OFF by default
# in another terminal, run an example against the sim:
ros2 launch crazyflie_examples launch.py script:=hello_world
```

RViz only opens if you pass `rviz:=True` (the launch default is `False`). With it,
RViz shows the simulated drone. Backend/controller/visualization options are in
`config/server.yaml` (`sim:` section).

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
# terminal 1 — start the Crazyflie server (also starts mocap tracking + Foxglove bridge)
# add rviz:=True for the RViz window (off by default)
ros2 launch crazyflie launch.py rviz:=True

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

Two viewers, both optional:

- **RViz** — off by default; add `rviz:=True` to the launch. It shows two frames
  per drone: `cf1` (the **onboard EKF** state estimate) and `cf1_mocap` (the raw
  **mocap** pose). They should sit almost on top of each other; a large or growing
  gap means the estimator and mocap disagree (bad calibration, marker/rigid-body
  issue, or estimator not converged).
- **Foxglove** — on by default (`foxglove:=True`), but needs the bridge package
  installed (`sudo apt install ros-$ROS_DISTRO-foxglove-bridge`). It's not a
  window: open the Foxglove Studio app and connect to `ws://localhost:8765` to
  inspect topics, poses, and TF live.

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

### Viewing a drone's topics (ROS_DOMAIN_ID)

In this setup **all drones share the same ROS domain** — they are **not** separated
by `ROS_DOMAIN_ID`. Each drone is separated by its **namespace** instead, e.g.
`/cf1/pose`, `/cf2/pose`. To narrow the topic list to one drone, filter by namespace:

```bash
ros2 topic list | grep cf1        # only cf1's topics
ros2 topic echo /cf1/pose         # echo a specific drone topic
```

`ROS_DOMAIN_ID` only matters in two cases:

1. **Your terminal sees no topics at all.** The shell's `ROS_DOMAIN_ID` must
   **match** the one the crazyflie stack was launched with (default `0`). In every
   terminal that needs to see the topics, set the same value (and a matching
   `ROS_LOCALHOST_ONLY`), then re-list:
   ```bash
   export ROS_DOMAIN_ID=<same value as the running stack>
   ros2 topic list
   ```
2. **Shared lab network / multiple users on one machine.** Pick a unique
   `ROS_DOMAIN_ID` (0–101) and `export ROS_LOCALHOST_ONLY=1` so you don't see — or
   accidentally command — someone else's robots. (See upstream crazyswarm2
   `docs2/usage.rst`.)

> The `natnet_ros2` helper GUI (`src/natnet_ros2/scripts/helper_node_r2.py`) has a
> domain-id selector — it sets which domain the **NatNet driver node** publishes on,
> i.e. the driver's domain, not a per-drone separation.

## E. Enabling extra telemetry logging (custom topics)

The drones can stream onboard firmware variables back as ROS topics. This is
configured under `all.firmware_logging.custom_topics` in
`config/crazyflies.yaml`. Several example blocks are shipped **commented out** —
enable them by uncommenting.

```yaml
all:
  firmware_logging:
    enabled: true
    custom_topics:
      estimator:          # enabled by default -> /cf1/estimator
        frequency: 5
        vars: [stateEstimate.x, stateEstimate.y, stateEstimate.z]
      # attitude:         # uncomment to enable -> /cf1/attitude
      #   frequency: 10
      #   vars: [stabilizer.roll, stabilizer.pitch, stabilizer.yaw]
      # battery:          # uncomment to enable -> /cf1/battery
      #   frequency: 1
      #   vars: [pm.vbat, pm.state]
```

**To enable a log topic:**

1. Uncomment the block (remove the leading `# `), or add your own. Each top-level
   key (`attitude`, `battery`, …) becomes the ROS topic name `/<cf>/<name>`.
2. Set `frequency` (Hz) — **keep it low**; every block shares the Crazyradio
   bandwidth (see [MOCAP §3c](MOCAP.md#3c-trim-drone-log-topics-to-protect-radio-bandwidth)).
3. List the firmware `vars` you want. Discover valid names with:
   ```bash
   ros2 run crazyflie listLogVariables --uri radio://0/80/2M/E7E7E7E701
   ```
4. Rebuild and relaunch:
   ```bash
   ./scripts/build.sh crazyflie     # rebuild after editing the config
   ros2 launch crazyflie launch.py
   ```

**To view a topic** (type is `crazyflie_interfaces/LogDataGeneric`):

```bash
ros2 topic list | grep cf1          # see which log topics exist
ros2 topic echo /cf1/attitude       # live values
ros2 topic hz /cf1/attitude         # confirm it streams at the set rate
```

The `values` array in `LogDataGeneric` is ordered exactly as the `vars` list.
The `default_topics` (`pose`, `status`) are enabled the same way — they're just
predefined names the server understands.

> Tip: enable high-rate blocks (e.g. 50 Hz attitude/kalman) only while actively
> debugging, then comment them out again for normal flight to free up the radio.

## Launch arguments (crazyflie launch.py)

| Arg | Values | Meaning |
|-----|--------|---------|
| `backend` | `cpp` \| `cflib` \| `sim` | Hardware (C++/Python) or simulation |
| `mocap` | `True` \| `False` | Start the motion_capture_tracking node (default `True`) |
| `teleop` | `True` \| `False` | Start joystick teleop + joy node (default `True`) |
| `rviz` | `True` \| `False` | Start RViz (default **`False`**) |
| `gui` | `True` \| `False` | Start the swarm GUI (default **`False`**) |
| `foxglove` | `True` \| `False` | Start the foxglove bridge (default `True`; needs `ros-$ROS_DISTRO-foxglove-bridge`) |
| `debug` | `True` \| `False` | Launch the C++ server under gdb (default `False`) |
