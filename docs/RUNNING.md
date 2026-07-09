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
ros2 launch crazyflie launch.py backend:=sim
# in another terminal, run an example against the sim:
ros2 launch crazyflie_examples launch.py script:=hello_world
```

RViz opens by default (`rviz:=false` to disable) and shows the simulated drone.
The preflight GUI (section C) also opens by default; in sim there is no mocap so
its mocap panels stay empty — pass `preflight:=False` if you don't want it.
Backend/controller/visualization options are in `config/server.yaml`
(`sim:` section).

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
# terminal 1 — start the Crazyflie server (also starts mocap tracking, RViz,
# the preflight GUI and the Foxglove bridge — all on by default)
ros2 launch crazyflie launch.py

# run the preflight checklist in the GUI — see section C below

# terminal 2 — takeoff, hover, land
ros2 run crazyflie_examples hello_world
```

The drone arms, takes off, hovers, then lands and disarms. Don't skip the
preflight check: it is the only thing that catches a rotated Motive rigid body
before it becomes a fly-away (section C, "The orientation warning").

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

- **RViz** — on by default (`rviz:=false` to disable). It shows two frames
  per drone: `cf1` (the **onboard EKF** state estimate) and `cf1_mocap` (the raw
  **mocap** pose). They should sit almost on top of each other; a large or growing
  gap means the estimator and mocap disagree (bad calibration, marker/rigid-body
  issue, or estimator not converged). Rotated axes between the two frames mean
  the rigid body was defined with a yaw offset — the preflight GUI (section C)
  quantifies this as `err.yaw`.
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
on) or the hostname/IP in `config/motion_capture.yaml`. If `/poses` **was**
flowing and stopped, check `ros2 topic info /poses` — 0 publishers means the
mocap node is dead or starved (a leftover process on UDP 1511 is the confirmed
cause on this rig). See [TROUBLESHOOTING](TROUBLESHOOTING.md#mocap-pipeline).

> **Alternative (open NatNet driver).** If you stream via the open-source
> `natnet_ros2` driver instead of the closed-source direct client, start it and
> the bridge separately: `ros2 launch natnet_ros2 natnet_ros2.launch.py` then
> `python3 ~/CrazySwarm2/pose_bridge.py` (republishes per-body poses to `/poses`
> at 50 Hz). The flight steps above are otherwise identical.

## C. Preflight GUI (preflight_kalman_plotter.py)

`launch.py` auto-starts `crazyflie/scripts/preflight_kalman_plotter.py`
(disable with `preflight:=False`). One window, one drone at a time — this is
the go/no-go check before every flight.

### What you see

- **Header** — big drone name + battery voltage, color-coded to the
  `crazyflies.yaml` thresholds (green > 3.8 V, orange 3.7–3.8 V, red < 3.7 V),
  plus `(charging)` / `ARMED` flags. Cycle drones with the ◀ Prev / Next ▶
  buttons or the left/right arrow keys (wrap-around; a `drone 2/3` indicator
  shows where you are). Only drones with `enabled: true` in `crazyflies.yaml`
  appear (`--config <path>` points the script at a different yaml; if the yaml
  can't be read it accepts everything and prints a warning).
- **2×2 plots**:
  1. **kalman telemetry** — 8 firmware vars from the `kalman_preflight` custom
     log topic (`kalman.stateX/Y/Z`, `motion.deltaX/Y`, `range.zrange`,
     `stateEstimateZ.vx/vy`), in raw firmware units.
  2. **pose** — stateEstimate x/y/z [m] and roll/pitch/yaw [deg] from
     `/<cf>/pose`.
  3. **connectivity** — mocap arrival rate [Hz] of this drone's entry in
     `/poses` (decays to 0 within ~1 s when mocap dies — that is the "mocap
     died" tell), plus radio RSSI and latency from `/<cf>/status`.
  4. **mocap − stateEstimate error** — position err.x/y/z/norm [m] as solid
     lines (left axis), orientation err.roll/pitch/yaw [deg] as dashed lines
     (right axis, wrapped to ±180°).

### The orientation warning (read this)

A red banner like `⚠ MOCAP ORIENTATION MISALIGNED: cf2 (yaw 150°)` appears when
**any** enabled drone — not just the one displayed — has a fresh orientation
error over 10° (`ORIENT_WARN_DEG` in the script; samples older than
`ERR_STALE_S` = 3 s are ignored). The error-panel title also turns red when the
displayed drone is the offender.

Why this matters: `locSrv.extPosStdDev = 1e-3` force-fuses the mocap
**position** into the onboard Kalman, so position error is always ~1 mm even
when the Motive rigid body was defined rotated. A yaw offset is invisible in
position at rest — and causes a fly-away in flight. `err.yaw ≈ 0` means the
rigid body is aligned with the drone's forward axis (see
[MOCAP §2](MOCAP.md#2-defining-rigid-bodies) for the rule when creating it).

Go/no-go: within **±5°** fly; **5–15°** fix the rigid body first; **> 20°** do
not fly.

### Buttons and keys

Two rows of buttons. The **upper row** acts on the selected drone only, via its
per-drone services:

> Record CSV | Takeoff (cfX) | Land (cfX) | Arm (cfX) | Disarm (cfX) | E-STOP (cfX)

The **lower row** broadcasts to all drones:

> Reset Kalman (all) — sets `all.params.kalman.resetEstimation` 1→0 |
> Takeoff (all) | Land (all) | Arm (all) | Disarm (all) | E-STOP (all)

Keys: `r` = reset kalman, `e` = **broadcast** e-stop (deliberately broadcast —
the panic key stops everything). Takeoff/land have **no** key bindings on
purpose. The takeoff/land setpoints are constants at the top of the script
(0.5 m over 3 s up, 0.03 m over 3 s down).

### Record CSV

**Record CSV** snapshots **every** enabled drone at 10 Hz to
`~/crazyswarm_ws/preflight_logs/preflight_<YYYYmmdd-HHMMSS>.csv` — 33 columns:
t, drone, mocap x/y/z, estimate x/y/z/rpy, err x/y/z/norm/roll/pitch/yaw, the 8
`kal_*` vars, rssi, latency_ms, mocap_hz, battery_v, pm_state,
supervisor_info. Missing values stay blank. The button shows a live row count
while recording, and the file is flushed every tick.

### Preflight checklist

1. `ros2 launch crazyflie launch.py` — wait for the drones to appear in the GUI.
2. Per drone (cycle with the arrow keys):
   - no red misalignment banner;
   - mocap rate steady (~50 Hz) in the connectivity panel;
   - RSSI / latency stable;
   - error dashed lines near 0°, err.norm at mm level;
   - battery green.
3. **Reset Kalman (all)**.
4. Arm, then a short test hop with the per-drone Takeoff / Land.
5. Fly your mission script.

## D. Manual / teleop flight

```bash
ros2 launch crazyflie launch.py backend:=cpp teleop:=True
```

Gamepad mapping (buttons, axes, limits) is in `config/teleop.yaml`. Defaults:
takeoff = start, land = back, emergency = red, arm = yellow.

## E. Useful checks

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

## F. Enabling extra telemetry logging (custom topics)

The drones can stream onboard firmware variables back as ROS topics. This is
configured under `all.firmware_logging.custom_topics` in
`config/crazyflies.yaml`. Several example blocks are shipped **commented out** —
enable them by uncommenting.

```yaml
all:
  firmware_logging:
    enabled: true
    custom_topics:
      kalman_preflight:   # enabled by default -> /cf1/kalman_preflight
        frequency: 5      # feeds the preflight GUI (section C) — keep it
        vars: [kalman.stateX, kalman.stateY, kalman.stateZ, motion.deltaX,
               motion.deltaY, range.zrange, stateEstimateZ.vx, stateEstimateZ.vy]
      # attitude:         # uncomment to enable -> /cf1/attitude
      #   frequency: 10
      #   vars: [stabilizer.roll, stabilizer.pitch, stabilizer.yaw]
      # battery:          # uncomment to enable -> /cf1/battery
      #   frequency: 1
      #   vars: [pm.vbat, pm.state]
```

The `kalman_preflight` block is what the preflight GUI plots — leave it enabled
(it uses 22 B of the 26 B log-block budget; every var must exist in the
firmware's log TOC or the cpp server aborts at connect).

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
| `rviz` | `True` \| `False` | Start RViz (default **`True`**; `rviz:=false` to disable) |
| `preflight` | `True` \| `False` | Start the preflight GUI (default **`True`**; see section C) |
| `gui` | `True` \| `False` | Start the swarm GUI (default **`False`**) |
| `foxglove` | `True` \| `False` | Start the foxglove bridge (default `True`; needs `ros-$ROS_DISTRO-foxglove-bridge`) |
| `debug` | `True` \| `False` | Launch the C++ server under gdb (default `False`) |
