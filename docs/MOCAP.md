# MOCAP — OptiTrack setup, rigid bodies, and frequency tuning

This stack uses an OptiTrack camera system running **Motive** (on the Windows
host) to localize the drones. Motive streams pose data over NatNet to
`natnet_ros2`, which `pose_bridge.py` forwards to the swarm server as `/poses`.

Three things must be right before flight: a **good calibration**, a **clean rigid
body per drone**, and a **streaming rate matched to your radio bandwidth**.

---

## 1. Camera calibration (brief)

Full procedure: <https://docs.optitrack.com/motive/calibration>.

1. **Mask extraneous reflections.** In Motive's *Calibration* pane, with no
   markers in the volume, use **Mask Visible** so the cameras ignore fixed
   reflective sources (shiny floor, window frames, LED fixtures).
2. **Wand the volume.** Select **Start Wanding** and wave the calibration wand
   through the whole flight volume, covering all heights and corners until each
   camera collects enough samples (the per-camera bars fill up).
3. **Calculate** and apply the result; aim for an "Exceptional" / "Excellent"
   mean error.
4. **Set the ground plane** with the calibration square, defining the origin and
   axes. Use **Z-up** so the orientation matches what crazyswarm2 expects.

> **Reading the camera LED — is a camera seeing noise?**
> Each OptiTrack camera has a status LED. When a camera is **flashing white**, it
> is detecting reflections / marker-like blobs in its view. Before flight you want
> the cameras *quiet* (no flashing) when the volume is empty — a camera that keeps
> flashing white with nothing in the volume is seeing stray reflections or noise
> and should be re-masked (step 1) or the offending reflector removed. During
> flight, flashing white simply means it sees the drone markers, which is expected.

## 2. Defining rigid bodies

Full procedure: <https://docs.optitrack.com/motive/rigid-body-tracking>.

Each drone needs its own rigid body so Motive streams one pose per drone:

1. Place the drone in the volume so its markers (or the single active marker /
   mocap-deck pattern) are visible. The markers should appear as a small cluster
   in the *Perspective* view.
2. Select those markers, right-click → **Rigid Body → Create From Selected
   Markers** (or press **Ctrl+T**).
3. In the rigid body's properties, give it a **name that matches everything
   downstream** — the same string used in:
   - `config/crazyflies.yaml` (the robot key, e.g. `cf1`),
   - `pose_bridge.py` `DRONES = ['cf1', 'cf2']`,
   - the `/<name>/pose` topic published by `natnet_ros2`.
4. Enable **streaming IDs / names** and make sure orientation looks stable (the
   rigid body's axes shouldn't jitter or flip). Keep marker patterns
   **asymmetric and distinct between drones** so Motive doesn't swap their IDs.

> The marker geometry in `config/motion_capture.yaml`
> (`marker_configurations`) must match the physical marker layout on the drone
> (`default_single_marker`, `mocap_deck`, etc.).

## 3. Frequency / bandwidth tuning (240 → 50 Hz)

By default an OptiTrack system may stream at **240 Hz**. That is far more than the
Crazyflie needs and it loads the Crazyradio link, because **every mocap pose is
forwarded to the drone over the radio** as an external position update. Drop the
rate to **50 Hz**, which is plenty for stable position hold, and trim the drone
log topics so the 2 Mbit/s radio is not maxed out.

### 3a. Lower the streaming rate in Motive

1. **View → Settings → Streaming** (or the *Data Streaming* pane).
2. Make sure **Broadcast Frame Data** is enabled and the NatNet streaming engine
   is on (turn off VRPN/Trackd if unused).
3. Set the **point cloud / camera rate** to **50 Hz** (Motive *Settings →
   Cameras → Rate*, or the system rate). The streaming rate follows the camera
   frame rate.

### 3b. Match the ROS side to 50 Hz

These are already set in this repo — keep them consistent if you change the rate:

- **`pose_bridge.py`** → `PUBLISH_HZ = 50.0` (was 240). The publish timer and QoS
  deadline both derive from this. Publishing faster than Motive streams only
  resends stale poses.
- **`config/motion_capture.yaml`** → `topics.poses.qos.deadline: 50.0` so the QoS
  deadline corresponds to the 50 Hz period.

### 3c. Trim drone log topics to protect radio bandwidth

In `config/crazyflies.yaml`, the `all.firmware_logging` block controls how much
telemetry each drone streams back over the radio. High log rates compete with the
mocap pose uplink and the control setpoints for the same 2 Mbit/s. The values in
this repo are intentionally low:

```yaml
all:
  firmware_logging:
    enabled: true
    default_topics:
      pose:
        frequency: 10   # Hz   (was higher)
      status:
        frequency: 1    # Hz
    custom_topics:
      estimator:
        frequency: 5    # Hz   (firmware default is 50 — reduced to save bandwidth)
        vars: [stateEstimate.x, stateEstimate.y, stateEstimate.z]
```

Guidelines:

- **One radio dongle per 1–2 drones.** Drones on the same URI channel share the
  same radio bandwidth.
- Keep total logging modest: `pose @ 10 Hz`, `status @ 1 Hz`, one custom block
  `@ 5 Hz` is enough for monitoring. Add high-rate logging (50 Hz attitude/kalman
  blocks are commented out in the YAML) only when actively debugging, and remove
  it again for normal flight.
- **Symptoms of a saturated radio:** latency / receive-rate warnings from
  `crazyflie_server` (`min_unicast_receive_rate` in `config/server.yaml`),
  choppy position hold, dropped log packets, or the firmware supervisor
  triggering an emergency landing.

After editing `config/*.yaml`, re-apply the overlay (or just rerun setup):

```bash
cp config/*.yaml src/crazyswarm2/crazyflie/config/   # or: ./scripts/setup.sh
```
