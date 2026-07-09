#!/usr/bin/python3
"""Preflight live plot of onboard Kalman/flow/range telemetry for all drones.

Subscribes to /<drone>/kalman_preflight (crazyflie_interfaces/LogDataGeneric),
values = [kalman.stateX, kalman.stateY, kalman.stateZ, motion.deltaX,
motion.deltaY, range.zrange, stateEstimateZ.vx, stateEstimateZ.vy] as declared
in crazyflies.yaml -> all.firmware_logging.custom_topics.kalman_preflight.

Drones are discovered from the ROS graph and filtered against crazyflies.yaml:
only robots marked enabled: true are plotted (the config is resolved from the
installed crazyflie package share directory, which is symlink-installed back
to the source tree; override with --config <path>). If the yaml cannot be
found or parsed the filter is disabled with a warning and every discovered
namespace is accepted — a config hiccup must never stop the tool. Starts with
`ros2 launch crazyflie launch.py` (preflight:=False to disable).

The window shows ONE drone at a time in a 2x2 grid. Top-left: kalman
telemetry. Top-right: a mirror of cfclient's Plotter -> Pose view plotting
stateEstimate.x/y/z [m] and stateEstimate.roll/pitch/yaw [deg] from
/<drone>/pose (PoseStamped published by crazyflie_server; the quaternion is
converted inline, aerospace ZYX convention). Data for ALL discovered drones
keeps accumulating in the background; only the rendering is per-drone. The
"Prev"/"Next" buttons flanking the header (or the left/right arrow keys)
cycle through the sorted drone set with wrap-around, and a small "drone 2/3"
indicator under the header shows the position in the cycle. The first drone
discovered is selected automatically. The two data sources are discovered
independently — a drone that only publishes one of them is still selectable,
with the other plot empty until data arrives.

The header shows the selected drone's name in large bold type with its
latest battery voltage from /<drone>/status right next to it (green above
3.8 V, orange 3.7-3.8 V, red below 3.7 V — matching
voltage_warning/voltage_critical in crazyflies.yaml), plus "(charging)" when
the power manager reports charging and "ARMED" when the supervisor does.
"--.- V" in gray means no status message received yet.

Bottom-left: connectivity — the drone's arrival rate in the global /poses
mocap topic (computed over a 1 s sliding window, so it decays to ZERO when
mocap dies), plus radio rssi and unicast latency from /<drone>/status,
sampled into history by a 1 Hz timer so the plot keeps scrolling even when a
source stops. Bottom-right: mocap-vs-onboard error — err.x/y/z/norm [m] as
solid lines on the LEFT axis and err.roll/pitch/yaw [deg] as DASHED lines on
a twin RIGHT axis. Each sample is the latest /poses value minus the latest
/<drone>/pose estimate, computed on each mocap arrival (latest-vs-latest;
the two timestamps are NOT aligned, which is fine for preflight where the
drone is static or slow); angle differences are wrapped into [-180, 180).
A steady position offset means a frame/marker config problem; a diverging
error means the kalman filter is not fusing mocap. The ORIENTATION error is
the key preflight signal: locSrv.extPosStdDev force-fuses mocap position, so
position residuals stay ~mm even when the Motive rigid body was defined
rotated relative to the drone's forward axis — a yaw offset invisible in
position at rest, but a fly-away in flight. A steady large err.yaw at rest
means exactly that rigid-body misalignment. (Absolute heading vs global X is
readable from stateEstimate.yaw in the pose panel; err.yaw is the
mocap-vs-onboard alignment check.) /poses
(motion_capture_tracking_interfaces/NamedPoseArray) is subscribed with
sensor-data QoS — the publisher is best-effort.

A red warning banner between the header and the plots watches ALL enabled
drones, not just the selected one — a misaligned drone must not hide while
another is displayed. Any drone whose latest error sample is fresh (within
ERR_STALE_S seconds of the newest error sample across all drones —
dataset-relative freshness, so the check needs no extra clock) and whose
worst |err.roll/pitch/yaw| exceeds ORIENT_WARN_DEG appears in the banner,
worst-first, e.g. "MOCAP ORIENTATION MISALIGNED: cf2 (yaw 150°)". The error
panel's title also turns red while the SELECTED drone is offending. If every
error source stops at once the last verdict freezes — the mocap Hz flatline
in the connectivity panel already covers that failure.

"Record CSV" (upper-left button) logs a 10 Hz snapshot of EVERY enabled
drone (not just the one displayed) to
~/crazyswarm_ws/preflight_logs/preflight_<timestamp>.csv: mocap/estimate
positions, position AND orientation errors, the kalman telemetry vars,
rssi/latency/mocap rate and battery/supervisor state; missing values stay
empty cells. While recording
the button shows the rows written so far and the header indicator shows the
filename in red.

Two bottom button rows. The UPPER row commands only the drone currently in
view through its per-drone services (/<name>/takeoff, /<name>/land,
/<name>/arm, /<name>/emergency); its labels track the selection, e.g.
"Takeoff (cf5)". The LOWER row broadcasts to every drone: "Reset Kalman
(all)" (or pressing 'r' with the window focused) sets the broadcast firmware
param all.params.kalman.resetEstimation on /crazyflie_server to 1 then 0 —
resetting the Kalman filter of every connected drone; no cflib link is
opened, the server owns the Crazyradio. "Takeoff (all)"/"Land (all)" call
/all/takeoff and /all/land (crazyflie_interfaces/Takeoff, Land) using the
TAKEOFF_*/LAND_* constants below; takeoff/land are deliberately NOT bound to
any keyboard key — an accidental keypress must not launch the drones.
"E-STOP (all)" calls /all/emergency (std_srvs/Empty); the 'e' key is bound to
the BROADCAST e-stop only (keyboard = stop everything, always the safer
default). "Arm (all)"/"Disarm (all)" call /all/arm (crazyflie_interfaces/Arm)
with arm=True/False. All service calls run on worker threads; a missing
service only logs a warning.

Raw firmware units are plotted: state* in m, delta* in flow px, zrange in mm,
vx/vy in mm/s.
"""
import argparse
import csv
import math
import os
import tempfile
import threading
import time
from collections import deque

import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import LogDataGeneric, Status
from crazyflie_interfaces.srv import Arm, Land, Takeoff
from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

TOPIC = 'kalman_preflight'
VARS = ('kalman.stateX', 'kalman.stateY', 'kalman.stateZ',
        'motion.deltaX', 'motion.deltaY', 'range.zrange',
        'stateEstimateZ.vx', 'stateEstimateZ.vy')
RESET_PARAM = 'all.params.kalman.resetEstimation'

POSE_TOPIC = 'pose'
STATUS_TOPIC = 'status'
POSE_VARS = ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
             'stateEstimate.roll', 'stateEstimate.pitch', 'stateEstimate.yaw')

MOCAP_TOPIC = '/poses'
CONN_VARS = ('mocap [Hz]', 'radio rssi [-dBm]', 'radio latency [ms]')
ERR_VARS = ('err.x', 'err.y', 'err.z', 'err.norm')          # meters
ERR_DEG_VARS = ('err.roll', 'err.pitch', 'err.yaw')         # degrees

# mocap misalignment banner (tweak to taste)
ORIENT_WARN_DEG = 10.0  # warn when any |err.roll/pitch/yaw| exceeds this [deg]
ERR_STALE_S = 3.0       # ignore error samples older than this vs the newest one

# broadcast takeoff/land setpoints (tweak to taste)
TAKEOFF_HEIGHT = 0.5     # m
TAKEOFF_DURATION = 3.0   # s
LAND_HEIGHT = 0.03       # m
LAND_DURATION = 3.0      # s

# csv recording (10 Hz snapshot of every enabled drone)
LOG_DIR = os.path.expanduser('~/crazyswarm_ws/preflight_logs')
CSV_FIELDS = (['t', 'drone',
               'mocap_x', 'mocap_y', 'mocap_z',
               'est_x', 'est_y', 'est_z', 'est_roll', 'est_pitch', 'est_yaw',
               'err_x', 'err_y', 'err_z', 'err_norm',
               'err_roll', 'err_pitch', 'err_yaw']
              + [f'kal_{v}' for v in VARS]
              + ['rssi', 'latency_ms', 'mocap_hz', 'battery_v', 'pm_state',
                 'supervisor_info'])


def duration_from_sec(seconds):
    """float seconds -> builtin_interfaces/Duration."""
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def enabled_drones(yaml_path):
    """Parse crazyflies.yaml and return the set of robot names under robots:
    that have enabled: true. Raises on a missing/unparsable file; the caller
    falls back to accepting every namespace."""
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    return {name for name, robot in (cfg.get('robots') or {}).items()
            if robot and robot.get('enabled')}


def quat_to_rpy_deg(qx, qy, qz, qw):
    """Quaternion -> roll/pitch/yaw in degrees, aerospace ZYX convention
    (inline math; no scipy/tf_transformations dependency)."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz),
                      1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def wrap_deg(a):
    """Wrap an angle difference in degrees into [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def battery_label(status):
    """(text, color) for a drone's battery readout from its latest Status."""
    if status is None:
        return '--.- V', 'gray'
    v = status.battery_voltage
    if v > 3.8:
        color = 'green'
    elif v >= 3.7:
        color = 'orange'          # voltage_warning threshold
    else:
        color = 'red'             # voltage_critical threshold
    text = f'{v:.2f} V'
    if status.pm_state == Status.PM_STATE_CHARGING:
        text += ' (charging)'
    if status.supervisor_info & Status.SUPERVISOR_INFO_IS_ARMED:
        text += '  ARMED'
    return text, color


class PreflightListener(Node):
    def __init__(self, window, allowed=None):
        super().__init__('preflight_kalman_plotter')
        self.window = window
        self.allowed = allowed  # names enabled in crazyflies.yaml (None = accept all)
        self.lock = threading.Lock()
        self.t0 = None
        self.data = {}          # name -> {'t': deque, var: deque} (kalman telemetry)
        self.pose_data = {}     # name -> {'t': deque, var: deque} (pose)
        self.new_drones = []    # discovered but not yet given axes (GUI thread drains)
        self.status = {}        # name -> latest crazyflie_interfaces/Status
        self.drone_clients = {}  # name -> per-drone service clients (lazy, at discovery)
        self.mocap_pose = {}     # name -> latest (x, y, z) from /poses
        self.mocap_arrivals = {}  # name -> deque of /poses stamps (Hz estimate)
        self.est_pose = {}       # name -> latest (x, y, z, roll, pitch, yaw)
        self.err_data = {}       # name -> {'t': deque, var: deque} (mocap error)
        self.conn_data = {}      # name -> {'t': deque, var: deque} (connectivity)
        # csv recording state (guarded by self.lock, timer runs at 10 Hz)
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.csv_rows = 0
        self.csv_timer = None
        self.set_param_client = self.create_client(
            SetParameters, '/crazyflie_server/set_parameters')
        self.emergency_client = self.create_client(Empty, '/all/emergency')
        self.arm_client = self.create_client(Arm, '/all/arm')
        self.takeoff_client = self.create_client(Takeoff, '/all/takeoff')
        self.land_client = self.create_client(Land, '/all/land')
        # the /poses publisher (pose_bridge/mocap) uses sensor-data QoS
        # (best-effort) - a default reliable subscription would receive NOTHING
        self.create_subscription(NamedPoseArray, MOCAP_TOPIC, self.mocap_cb,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.discover)
        self.create_timer(1.0, self.sample_connectivity)
        self.get_logger().info(
            f'waiting for /<drone>/{TOPIC} and /<drone>/{POSE_TOPIC} topics...')

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            parts = topic.split('/')
            if (len(parts) == 3 and parts[2] == TOPIC
                    and 'crazyflie_interfaces/msg/LogDataGeneric' in types):
                name = parts[1]
                if not self.allowed_name(name):
                    continue
                with self.lock:
                    if name in self.data:
                        continue
                    self.data[name] = {k: deque() for k in ('t',) + VARS}
                    self.new_drones.append(name)
                self._ensure_drone_clients(name)
                self.create_subscription(
                    LogDataGeneric, topic,
                    lambda msg, n=name: self.cb(msg, n), 10)
                self.get_logger().info(f'plotting {name} ({topic})')
            elif (len(parts) == 3 and parts[2] == POSE_TOPIC
                    and 'geometry_msgs/msg/PoseStamped' in types):
                name = parts[1]
                if not self.allowed_name(name):
                    continue
                with self.lock:
                    if name in self.pose_data:
                        continue
                    self.pose_data[name] = {k: deque()
                                            for k in ('t',) + POSE_VARS}
                    self.new_drones.append(name)
                self._ensure_drone_clients(name)
                self.create_subscription(
                    PoseStamped, topic,
                    lambda msg, n=name: self.pose_cb(msg, n), 10)
                self.create_subscription(
                    Status, f'/{name}/{STATUS_TOPIC}',
                    lambda msg, n=name: self.status_cb(msg, n), 10)
                self.get_logger().info(f'plotting pose of {name} ({topic})')

    def allowed_name(self, name):
        """Discovery filter: accept only drones enabled in crazyflies.yaml
        (or everything when no usable yaml was found)."""
        return self.allowed is None or name in self.allowed

    def _ensure_drone_clients(self, name):
        """Lazily create this drone's per-drone service clients (once)."""
        if name in self.drone_clients:
            return
        self.drone_clients[name] = {
            'takeoff': self.create_client(Takeoff, f'/{name}/takeoff'),
            'land': self.create_client(Land, f'/{name}/land'),
            'arm': self.create_client(Arm, f'/{name}/arm'),
            'emergency': self.create_client(Empty, f'/{name}/emergency'),
        }

    def cb(self, msg, name):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            d = self.data[name]
            d['t'].append(t - self.t0)
            for i, v in enumerate(VARS):
                # pad with NaN if the yaml lists fewer vars than expected
                d[v].append(msg.values[i] if i < len(msg.values) else float('nan'))
            while d['t'] and d['t'][-1] - d['t'][0] > self.window:
                for k in ('t',) + VARS:
                    d[k].popleft()

    def pose_cb(self, msg, name):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.orientation
        roll, pitch, yaw = quat_to_rpy_deg(q.x, q.y, q.z, q.w)
        p = msg.pose.position
        vals = (p.x, p.y, p.z, roll, pitch, yaw)
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            d = self.pose_data[name]
            d['t'].append(t - self.t0)
            for v, val in zip(POSE_VARS, vals):
                d[v].append(val)
            while d['t'] and d['t'][-1] - d['t'][0] > self.window:
                for k in ('t',) + POSE_VARS:
                    d[k].popleft()
            self.est_pose[name] = vals

    def status_cb(self, msg, name):
        with self.lock:
            self.status[name] = msg

    def mocap_cb(self, msg):
        """Global /poses (mocap truth). Per enabled drone: store the latest
        position, an arrival stamp for the Hz estimate, and a position AND
        orientation error sample against the latest onboard estimate (both
        quaternions -> rpy deg, per-angle wrapped difference). Error is
        latest-vs-latest - the two timestamps are NOT aligned, which is fine
        for preflight where the drone is static or slow."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            for named in msg.poses:
                name = named.name
                if not self.allowed_name(name):
                    continue
                pos = named.pose.position
                self.mocap_pose[name] = (pos.x, pos.y, pos.z)
                arr = self.mocap_arrivals.setdefault(name, deque())
                arr.append(t)
                while arr and t - arr[0] > 2.0:
                    arr.popleft()
                est = self.est_pose.get(name)
                if est is None:
                    continue
                ex, ey, ez = pos.x - est[0], pos.y - est[1], pos.z - est[2]
                en = math.sqrt(ex * ex + ey * ey + ez * ez)
                q = named.pose.orientation
                mroll, mpitch, myaw = quat_to_rpy_deg(q.x, q.y, q.z, q.w)
                eroll = wrap_deg(mroll - est[3])
                epitch = wrap_deg(mpitch - est[4])
                eyaw = wrap_deg(myaw - est[5])
                d = self.err_data.setdefault(
                    name,
                    {k: deque() for k in ('t',) + ERR_VARS + ERR_DEG_VARS})
                d['t'].append(t - self.t0)
                for k, val in zip(ERR_VARS + ERR_DEG_VARS,
                                  (ex, ey, ez, en, eroll, epitch, eyaw)):
                    d[k].append(val)
                while d['t'] and d['t'][-1] - d['t'][0] > self.window:
                    for k in ('t',) + ERR_VARS + ERR_DEG_VARS:
                        d[k].popleft()

    def mocap_rate(self, name, now):
        """/poses arrivals for this drone during the last second (caller
        holds self.lock). Decays to zero by itself when mocap stops."""
        arr = self.mocap_arrivals.get(name)
        if not arr:
            return 0.0
        return float(sum(1 for ts in arr if now - ts <= 1.0))

    def sample_connectivity(self):
        """1 Hz history sampler so the connectivity plot keeps scrolling even
        when a source dies: mocap Hz flatlines to zero, rssi/latency hold the
        last known /status values."""
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            if self.t0 is None:
                return
            t = now - self.t0
            for name in set(self.data) | set(self.pose_data):
                st = self.status.get(name)
                vals = (self.mocap_rate(name, now),
                        float(st.rssi) if st is not None else float('nan'),
                        float(st.latency_unicast) if st is not None
                        else float('nan'))
                d = self.conn_data.setdefault(
                    name, {k: deque() for k in ('t',) + CONN_VARS})
                d['t'].append(t)
                for k, val in zip(CONN_VARS, vals):
                    d[k].append(val)
                while d['t'] and d['t'][-1] - d['t'][0] > self.window:
                    for k in ('t',) + CONN_VARS:
                        d[k].popleft()

    def reset_kalman(self):
        """Set all.params.kalman.resetEstimation to 1 then 0 (firmware convention)."""
        if not self.set_param_client.service_is_ready():
            self.get_logger().warning(
                'crazyflie_server set_parameters service not available - no reset sent')
            return

        def _set(val):
            req = SetParameters.Request()
            req.parameters = [Parameter(
                name=RESET_PARAM,
                value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                     integer_value=val))]
            return self.set_param_client.call_async(req)

        def _wait(future, deadline=1.0):
            """Poll a call_async future to completion without blocking the executor
            thread (the node is already spun by a background thread, so
            rclpy.spin_until_future_complete must not be used here)."""
            start = time.monotonic()
            while not future.done():
                if time.monotonic() - start > deadline:
                    return None
                time.sleep(0.01)
            return future.result()

        def _worker():
            try:
                result = _wait(_set(1))
                if result is None:
                    self.get_logger().warning(
                        'reset kalman: write 1 (resetEstimation=1) timed out '
                        'waiting for set_parameters response')
                    return
                if not result.results[0].successful:
                    self.get_logger().warning(
                        'reset kalman: write 1 (resetEstimation=1) rejected - '
                        f'{result.results[0].reason}')
                    return

                time.sleep(0.1)

                result = _wait(_set(0))
                if result is None:
                    self.get_logger().warning(
                        'reset kalman: write 0 (resetEstimation=0) timed out '
                        'waiting for set_parameters response')
                    return
                if not result.results[0].successful:
                    self.get_logger().warning(
                        'reset kalman: write 0 (resetEstimation=0) rejected - '
                        f'{result.results[0].reason}')
                    return

                self.get_logger().info(
                    'kalman estimator reset confirmed on ALL drones')
            except Exception as exc:
                # e.g. node/context torn down mid-worker if the window is closed
                # during the 0.1 s gap between the two writes
                self.get_logger().warning(f'reset kalman worker aborted: {exc}')

        threading.Thread(target=_worker, daemon=True).start()

    def _call_service(self, client, request, label, deadline=1.0):
        """Fire a service call from a worker thread; never block the GUI or the
        executor thread (same polling pattern as reset_kalman's _wait)."""
        if not client.service_is_ready():
            self.get_logger().warning(
                f'{label}: service {client.srv_name} not available - nothing sent')
            return

        def _worker():
            try:
                future = client.call_async(request)
                start = time.monotonic()
                while not future.done():
                    if time.monotonic() - start > deadline:
                        self.get_logger().warning(
                            f'{label}: timed out waiting for response')
                        return
                    time.sleep(0.01)
                self.get_logger().info(f'{label}: request sent')
            except Exception as exc:
                self.get_logger().warning(f'{label} worker aborted: {exc}')

        threading.Thread(target=_worker, daemon=True).start()

    def emergency(self):
        """Broadcast E-STOP: /all/emergency (std_srvs/Empty)."""
        self._call_service(self.emergency_client, Empty.Request(),
                           'E-STOP (all)')

    def arm(self, value):
        """Broadcast arm/disarm: /all/arm (crazyflie_interfaces/Arm)."""
        req = Arm.Request()
        req.arm = value
        self._call_service(self.arm_client, req,
                           'arm (all)' if value else 'disarm (all)')

    def takeoff(self):
        """Broadcast takeoff: /all/takeoff (crazyflie_interfaces/Takeoff)."""
        req = Takeoff.Request()
        req.group_mask = 0                  # 0 = all drones in the broadcast
        req.height = TAKEOFF_HEIGHT
        req.duration = duration_from_sec(TAKEOFF_DURATION)
        # server replies after forwarding to the radio; allow extra time
        self._call_service(self.takeoff_client, req, 'takeoff (all)',
                           deadline=3.0)

    def land(self):
        """Broadcast land: /all/land (crazyflie_interfaces/Land)."""
        req = Land.Request()
        req.group_mask = 0                  # 0 = all drones in the broadcast
        req.height = LAND_HEIGHT            # final height before motors stop
        req.duration = duration_from_sec(LAND_DURATION)
        # server replies after forwarding to the radio; allow extra time
        self._call_service(self.land_client, req, 'land (all)', deadline=3.0)

    def _drone_client(self, name, kind):
        """Per-drone client lookup with no-selection / unknown-name guards."""
        if name is None:
            self.get_logger().warning(
                f'{kind}: no drone selected - nothing sent')
            return None
        clients = self.drone_clients.get(name)
        if clients is None:
            self.get_logger().warning(
                f'{kind}: no service clients for {name} - nothing sent')
            return None
        return clients[kind]

    def takeoff_one(self, name):
        """Takeoff for the selected drone only: /<name>/takeoff."""
        client = self._drone_client(name, 'takeoff')
        if client is not None:
            req = Takeoff.Request()
            req.group_mask = 0          # ignored for a unicast takeoff
            req.height = TAKEOFF_HEIGHT
            req.duration = duration_from_sec(TAKEOFF_DURATION)
            self._call_service(client, req, f'takeoff ({name})', deadline=3.0)

    def land_one(self, name):
        """Land the selected drone only: /<name>/land."""
        client = self._drone_client(name, 'land')
        if client is not None:
            req = Land.Request()
            req.group_mask = 0          # ignored for a unicast land
            req.height = LAND_HEIGHT    # final height before motors stop
            req.duration = duration_from_sec(LAND_DURATION)
            self._call_service(client, req, f'land ({name})', deadline=3.0)

    def arm_one(self, name, value):
        """Arm/disarm the selected drone only: /<name>/arm."""
        kind = 'arm' if value else 'disarm'
        client = self._drone_client(name, 'arm')
        if client is not None:
            req = Arm.Request()
            req.arm = value
            self._call_service(client, req, f'{kind} ({name})')

    def emergency_one(self, name):
        """E-STOP the selected drone only: /<name>/emergency."""
        client = self._drone_client(name, 'emergency')
        if client is not None:
            self._call_service(client, Empty.Request(), f'E-STOP ({name})')

    def start_recording(self, log_dir=None):
        """Open a timestamped CSV under preflight_logs/ and start the 10 Hz
        snapshot timer. One row per enabled drone per tick, regardless of
        which drone is displayed."""
        with self.lock:
            if self.csv_file is not None:
                return
            directory = log_dir or LOG_DIR
            os.makedirs(directory, exist_ok=True)
            path = os.path.join(
                directory, time.strftime('preflight_%Y%m%d-%H%M%S.csv'))
            self.csv_file = open(path, 'w', newline='')
            self.csv_writer = csv.DictWriter(self.csv_file,
                                             fieldnames=CSV_FIELDS,
                                             restval='')
            self.csv_writer.writeheader()
            self.csv_file.flush()
            self.csv_path = path
            self.csv_rows = 0
        self.csv_timer = self.create_timer(0.1, self.snapshot)
        self.get_logger().info(f'recording CSV: {os.path.abspath(path)}')

    def stop_recording(self):
        """Stop the snapshot timer and close the CSV (idempotent; also called
        from the window-close teardown path)."""
        if self.csv_timer is not None:
            self.destroy_timer(self.csv_timer)
            self.csv_timer = None
        with self.lock:
            if self.csv_file is None:
                return
            path, rows = self.csv_path, self.csv_rows
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        self.get_logger().info(
            f'stopped recording ({rows} rows): {os.path.abspath(path)}')

    def snapshot(self):
        """10 Hz CSV tick: one row per known drone; missing data stays empty
        (DictWriter restval)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            if self.csv_writer is None:
                return
            t = (now - self.t0) if self.t0 is not None else 0.0
            for name in sorted(set(self.data) | set(self.pose_data)):
                row = {'t': f'{t:.3f}', 'drone': name,
                       'mocap_hz': self.mocap_rate(name, now)}
                mp = self.mocap_pose.get(name)
                if mp is not None:
                    row['mocap_x'], row['mocap_y'], row['mocap_z'] = mp
                est = self.est_pose.get(name)
                if est is not None:
                    (row['est_x'], row['est_y'], row['est_z'],
                     row['est_roll'], row['est_pitch'], row['est_yaw']) = est
                ed = self.err_data.get(name)
                if ed and ed['t']:
                    for col, k in zip(('err_x', 'err_y', 'err_z', 'err_norm',
                                       'err_roll', 'err_pitch', 'err_yaw'),
                                      ERR_VARS + ERR_DEG_VARS):
                        row[col] = ed[k][-1]
                kd = self.data.get(name)
                if kd and kd['t']:
                    for v in VARS:
                        row[f'kal_{v}'] = kd[v][-1]
                st = self.status.get(name)
                if st is not None:
                    row['rssi'] = st.rssi
                    row['latency_ms'] = st.latency_unicast
                    row['battery_v'] = f'{st.battery_voltage:.3f}'
                    row['pm_state'] = st.pm_state
                    row['supervisor_info'] = st.supervisor_info
                self.csv_writer.writerow(row)
                self.csv_rows += 1
            self.csv_file.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=float, default=30.0,
                        help='seconds of history to show (default: 30)')
    parser.add_argument('--config', type=str, default=None,
                        help='path to crazyflies.yaml (default: the installed '
                             'crazyflie package share config)')
    parser.add_argument('--selftest', action='store_true',
                        help=argparse.SUPPRESS)
    # launch_ros appends --ros-args ...; tolerate unknown args
    args, _ = parser.parse_known_args()

    # resolve the enabled-drones filter; any failure falls back to accepting
    # every namespace (the tool must never refuse to start over a config)
    allowed = None
    cfg_path = args.config
    cfg_error = None
    try:
        if cfg_path is None:
            cfg_path = os.path.join(
                get_package_share_directory('crazyflie'),
                'config', 'crazyflies.yaml')
        allowed = enabled_drones(cfg_path)
    except Exception as exc:
        cfg_error = exc

    rclpy.init()
    node = PreflightListener(args.window, allowed)
    if allowed is None:
        node.get_logger().warning(
            f'cannot read enabled drones from {cfg_path} ({cfg_error}) - '
            'plotting every discovered namespace')
    else:
        node.get_logger().info(
            'enabled in crazyflies.yaml: '
            + (', '.join(sorted(allowed)) or '(none)'))

    def _spin():
        try:
            rclpy.spin(node)
        except Exception:
            pass  # ExternalShutdownException etc. - normal on rclpy.shutdown()

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    # matplotlib's default keymap.home binding includes 'r', which would also
    # trigger a view reset whenever the operator presses 'r' to reset kalman;
    # likewise strip 'e' (E-STOP) and the left/right arrows (drone selector,
    # bound by default in keymap.back/keymap.forward) from any default keymap
    if 'r' in plt.rcParams['keymap.home']:
        plt.rcParams['keymap.home'].remove('r')
    for key in plt.rcParams:
        if key.startswith('keymap.'):
            for k in ('e', 'left', 'right'):
                if k in plt.rcParams[key]:
                    plt.rcParams[key].remove(k)

    # bigger fonts throughout: ticks, axis labels and legends scale from here
    plt.rcParams['font.size'] = 14

    fig = plt.figure('crazyflie preflight', figsize=(14, 8))
    fig.subplots_adjust(bottom=0.25, top=0.87)

    # figure-level header artists, updated in place every frame (never
    # recreated): big drone name, big color-coded battery, cycle position
    header_name = fig.text(0.48, 0.945, 'waiting for drones...',
                           fontsize=22, fontweight='bold', color='gray',
                           ha='right', va='center')
    header_batt = fig.text(0.51, 0.945, '', fontsize=18,
                           ha='left', va='center')
    header_pos = fig.text(0.97, 0.905, '', fontsize=12, color='gray',
                          ha='right', va='center')
    # misalignment warning banner (empty = invisible), updated in place
    warn_banner = fig.text(0.5, 0.905, '', fontsize=15, fontweight='bold',
                           color='#cc3333', ha='center', va='center')

    # LOWER button row: broadcast /all/... commands; takeoff/land are
    # mouse-only by design (no keys)
    btn_reset = Button(fig.add_axes([0.03, 0.03, 0.155, 0.06]),
                       'Reset Kalman (all)')
    btn_reset.on_clicked(lambda _event: node.reset_kalman())
    btn_takeoff = Button(fig.add_axes([0.20, 0.03, 0.12, 0.06]),
                         'Takeoff (all)')
    btn_takeoff.on_clicked(lambda _event: node.takeoff())
    btn_land = Button(fig.add_axes([0.335, 0.03, 0.12, 0.06]), 'Land (all)')
    btn_land.on_clicked(lambda _event: node.land())
    btn_arm = Button(fig.add_axes([0.47, 0.03, 0.12, 0.06]), 'Arm (all)')
    btn_arm.on_clicked(lambda _event: node.arm(True))
    btn_disarm = Button(fig.add_axes([0.605, 0.03, 0.12, 0.06]),
                        'Disarm (all)')
    btn_disarm.on_clicked(lambda _event: node.arm(False))
    btn_estop = Button(fig.add_axes([0.74, 0.03, 0.23, 0.06]),
                       'E-STOP (all)', color='#cc3333', hovercolor='#ff5555')
    btn_estop.label.set_color('white')
    btn_estop.label.set_fontweight('bold')
    btn_estop.on_clicked(lambda _event: node.emergency())

    # selection state, GUI thread only: which drone is rendered. Mutated by
    # the prev/next callbacks and by discovery in update(); 'dirty' triggers
    # an axes rebuild on the next frame (same pattern as new_drones).
    sel = {'names': [], 'current': None, 'dirty': False}

    def cycle(step):
        names = sel['names']
        if sel['current'] is None or len(names) < 2:
            return  # zero or one drone: nothing to cycle
        i = names.index(sel['current'])
        sel['current'] = names[(i + step) % len(names)]  # wraps around
        sel['dirty'] = True

    # prev/next drone selectors flanking the header
    btn_prev = Button(fig.add_axes([0.03, 0.92, 0.09, 0.05]), '◀ Prev')
    btn_prev.on_clicked(lambda _event: cycle(-1))
    btn_next = Button(fig.add_axes([0.88, 0.92, 0.09, 0.05]), 'Next ▶')
    btn_next.on_clicked(lambda _event: cycle(1))

    # UPPER button row: CSV recording toggle plus commands for ONLY the drone
    # currently in view via its per-drone services; labels track the
    # selection (updated in update()), callbacks read the selection at click
    # time, and no-op with a warning while nothing is selected
    btn_record = Button(fig.add_axes([0.03, 0.115, 0.155, 0.06]),
                        'Record CSV')
    btn_record.on_clicked(
        lambda _event: (node.stop_recording() if node.csv_file is not None
                        else node.start_recording()))
    btn_takeoff_one = Button(fig.add_axes([0.20, 0.115, 0.12, 0.06]),
                             'Takeoff (-)')
    btn_takeoff_one.on_clicked(
        lambda _event: node.takeoff_one(sel['current']))
    btn_land_one = Button(fig.add_axes([0.335, 0.115, 0.12, 0.06]),
                          'Land (-)')
    btn_land_one.on_clicked(lambda _event: node.land_one(sel['current']))
    btn_arm_one = Button(fig.add_axes([0.47, 0.115, 0.12, 0.06]), 'Arm (-)')
    btn_arm_one.on_clicked(lambda _event: node.arm_one(sel['current'], True))
    btn_disarm_one = Button(fig.add_axes([0.605, 0.115, 0.12, 0.06]),
                            'Disarm (-)')
    btn_disarm_one.on_clicked(
        lambda _event: node.arm_one(sel['current'], False))
    btn_estop_one = Button(fig.add_axes([0.74, 0.115, 0.23, 0.06]),
                           'E-STOP (-)', color='#dd6666',
                           hovercolor='#ff8888')
    btn_estop_one.label.set_color('white')
    btn_estop_one.label.set_fontweight('bold')
    btn_estop_one.on_clicked(
        lambda _event: node.emergency_one(sel['current']))

    sel_buttons = {'Takeoff': btn_takeoff_one, 'Land': btn_land_one,
                   'Arm': btn_arm_one, 'Disarm': btn_disarm_one,
                   'E-STOP': btn_estop_one}

    for b in (btn_reset, btn_takeoff, btn_land, btn_arm, btn_disarm,
              btn_estop, btn_prev, btn_next, btn_record,
              *sel_buttons.values()):
        b.label.set_fontsize(13)

    def on_key(event):
        if event.key == 'r':
            node.reset_kalman()
        elif event.key == 'e':
            node.emergency()
        elif event.key == 'left':
            cycle(-1)
        elif event.key == 'right':
            cycle(1)

    fig.canvas.mpl_connect('key_press_event', on_key)

    axes = {}        # 'k'/'p'/'c'/'e' -> the four axes of the selected drone
    kal_lines = {}   # var -> Line2D (selected drone only)
    pose_lines = {}
    conn_lines = {}
    err_lines = {}
    err_title = {'artist': None}  # error-panel title Text (warn coloring)

    def rebuild_axes(name):
        """2x2 grid for the selected drone only: kalman / pose on top,
        connectivity / mocap-error below. Buttons and the header artists are
        left untouched."""
        for ax in axes.values():
            ax.remove()
        axes.clear()
        for lines in (kal_lines, pose_lines, conn_lines, err_lines):
            lines.clear()
        err_title['artist'] = None
        if name is None:
            return

        def _subplot(pos, key, variables, lines, title):
            ax = fig.add_subplot(2, 2, pos)
            ax.grid(True)
            for v in variables:
                lines[v] = ax.plot([], [], label=v, lw=0.8)[0]
            ax.legend(loc='upper left', ncol=2, fontsize='medium')
            ax.set_title(title, loc='left')
            axes[key] = ax
            return ax

        _subplot(1, 'k', VARS, kal_lines, 'kalman telemetry')
        _subplot(2, 'p', POSE_VARS, pose_lines,
                 'pose (stateEstimate) - x/y/z [m], r/p/y [deg]')
        _subplot(3, 'c', CONN_VARS, conn_lines,
                 'connectivity').set_xlabel('time [s]')
        ax_e = _subplot(4, 'e', ERR_VARS, err_lines,
                        'mocap - stateEstimate error [m | deg]')
        ax_e.set_xlabel('time [s]')
        # twin right axis: orientation error in degrees, dashed lines with
        # colors past the meter lines' C0-C3. Twin axes must be removed
        # explicitly on rebuild or they leak - hence their own axes['e2'].
        ax_e2 = ax_e.twinx()
        for i, v in enumerate(ERR_DEG_VARS):
            err_lines[v] = ax_e2.plot([], [], label=v, lw=0.8, ls='--',
                                      color=f'C{i + 4}')[0]
        # one merged legend for both unit families
        handles = [err_lines[v] for v in ERR_VARS + ERR_DEG_VARS]
        ax_e.legend(handles, [h.get_label() for h in handles],
                    loc='upper left', ncol=2, fontsize='medium')
        axes['e2'] = ax_e2
        # keep the title Text so update() can flip it to the warn color
        err_title['artist'] = ax_e.set_title(
            'mocap - stateEstimate error [m | deg]', loc='left')
        fig.subplots_adjust(bottom=0.25, top=0.87)

    def update(_frame):
        with node.lock:
            fresh = list(node.new_drones)
            node.new_drones.clear()
            if fresh:
                sel['names'] = sorted(set(node.data) | set(node.pose_data))
                if sel['current'] not in sel['names']:
                    # auto-select the first discovered drone
                    sel['current'] = (sel['names'][0] if sel['names']
                                      else None)
                sel['dirty'] = True
            if sel['dirty']:
                rebuild_axes(sel['current'])
                sel['dirty'] = False
            recording = node.csv_file is not None
            btn_record.label.set_text(
                f'Stop CSV ({node.csv_rows})' if recording else 'Record CSV')
            # misalignment banner: scan ALL drones' latest error samples (a
            # misaligned drone must not hide behind the selected one).
            # Freshness is dataset-relative: a drone counts only if its
            # newest sample is within ERR_STALE_S of the newest error sample
            # of ANY drone - no extra clock needed, works offline too.
            offenders = []
            latest_all = max((d['t'][-1] for d in node.err_data.values()
                              if d['t']), default=None)
            if latest_all is not None:
                for dn, d in node.err_data.items():
                    if not d['t'] or latest_all - d['t'][-1] > ERR_STALE_S:
                        continue  # no fresh mocap+estimate pair to judge
                    var, worst = max(
                        ((v, abs(d[v][-1])) for v in ERR_DEG_VARS),
                        key=lambda item: item[1])
                    if worst > ORIENT_WARN_DEG:
                        offenders.append((worst, dn, var))
            offenders.sort(reverse=True)  # worst offender first
            if offenders:
                warn_banner.set_text(
                    '⚠ MOCAP ORIENTATION MISALIGNED: ' + ', '.join(
                        f'{dn} ({var.split(".")[1]} {worst:.0f}°)'
                        for worst, dn, var in offenders))
            else:
                warn_banner.set_text('')
            offending = {dn for _, dn, _ in offenders}
            name = sel['current']
            if name is None:
                return []
            if err_title['artist'] is not None:
                err_title['artist'].set_color(
                    '#cc3333' if name in offending
                    else plt.rcParams['text.color'])
            header_name.set_text(name)
            header_name.set_color(plt.rcParams['text.color'])
            text, color = battery_label(node.status.get(name))
            header_batt.set_text(text)
            header_batt.set_color(color)
            pos_text = (
                f'drone {sel["names"].index(name) + 1}/{len(sel["names"])}')
            if recording and node.csv_path:
                pos_text += f'  REC -> {os.path.basename(node.csv_path)}'
            header_pos.set_text(pos_text)
            header_pos.set_color('#cc3333' if recording else 'gray')
            for prefix, b in sel_buttons.items():
                b.label.set_text(f'{prefix} ({name})')
            for key, src, variables, lines in (
                    ('k', node.data, VARS, kal_lines),
                    ('p', node.pose_data, POSE_VARS, pose_lines),
                    ('c', node.conn_data, CONN_VARS, conn_lines),
                    ('e', node.err_data, ERR_VARS, err_lines),
                    ('e2', node.err_data, ERR_DEG_VARS, err_lines)):
                d = src.get(name)
                if d is None or key not in axes:
                    continue
                t = list(d['t'])
                for v in variables:
                    lines[v].set_data(t, list(d[v]))
                if t:
                    axes[key].set_xlim(max(0.0, t[-1] - node.window),
                                       max(t[-1], node.window))
                    axes[key].relim()
                    axes[key].autoscale_view(scalex=False, scaley=True)
        return (list(kal_lines.values()) + list(pose_lines.values())
                + list(conn_lines.values()) + list(err_lines.values()))

    # keep a reference: an unassigned FuncAnimation is GC'd before plt.show()
    ani = FuncAnimation(fig, update, interval=100, blit=False,
                        cache_frame_data=False)

    def selftest():
        """Headless sanity check (--selftest): exercise the setup path and the
        quaternion conversion, then exit without plt.show()."""
        r, p, y = quat_to_rpy_deg(0.0, 0.0, 0.0, 1.0)
        assert abs(r) < 1e-9 and abs(p) < 1e-9 and abs(y) < 1e-9, (r, p, y)
        s = math.sin(math.radians(45.0))
        c = math.cos(math.radians(45.0))
        r, p, y = quat_to_rpy_deg(0.0, 0.0, s, c)
        assert abs(r) < 1e-6 and abs(p) < 1e-6 and abs(y - 90.0) < 1e-6, (r, p, y)
        r, p, y = quat_to_rpy_deg(s, 0.0, 0.0, c)
        assert abs(r - 90.0) < 1e-6, (r, p, y)
        assert wrap_deg(340.0) == -20.0
        assert wrap_deg(-190.0) == 170.0
        assert wrap_deg(90.0) == 90.0

        st = Status()
        st.battery_voltage = 3.85
        assert battery_label(st) == ('3.85 V', 'green')
        st.battery_voltage = 3.75
        assert battery_label(st)[1] == 'orange'
        st.battery_voltage = 3.60
        st.pm_state = Status.PM_STATE_CHARGING
        st.supervisor_info = Status.SUPERVISOR_INFO_IS_ARMED
        assert battery_label(st) == ('3.60 V (charging)  ARMED', 'red')
        assert battery_label(None) == ('--.- V', 'gray')

        # enabled_drones(): one enabled + one disabled robot in a temp yaml
        with tempfile.NamedTemporaryFile('w', suffix='.yaml',
                                         delete=False) as f:
            f.write('robots:\n'
                    '  cfon:\n    enabled: true\n'
                    '  cfoff:\n    enabled: false\n')
            tmp_yaml = f.name
        try:
            assert enabled_drones(tmp_yaml) == {'cfon'}
        finally:
            os.unlink(tmp_yaml)

        # discovery filter: a name not enabled in the yaml is rejected by the
        # predicate discover() uses, and never gets data entries
        node.allowed = {'cftest', 'cfzposeonly'}
        assert node.allowed_name('cftest')
        assert not node.allowed_name('cfbad')
        assert 'cfbad' not in node.data and 'cfbad' not in node.pose_data

        # mocap error sample: mocap offset by (0.1, -0.2, 0.3) and yawed +90
        # deg vs the onboard estimate -> position err/norm (~0.374) plus
        # orientation err.yaw ~ 90 with roll/pitch ~ 0
        with node.lock:
            node.est_pose['cftest'] = (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
        npa = NamedPoseArray()
        npa.header.stamp.sec = 100
        named = NamedPose()
        named.name = 'cftest'
        named.pose.position.x = 1.1
        named.pose.position.y = 1.8
        named.pose.position.z = 3.3
        named.pose.orientation.z = s   # +90 deg yaw quaternion
        named.pose.orientation.w = c
        npa.poses.append(named)
        node.mocap_cb(npa)
        with node.lock:
            ed = node.err_data['cftest']
            assert abs(ed['err.x'][-1] - 0.1) < 1e-9
            assert abs(ed['err.y'][-1] + 0.2) < 1e-9
            assert abs(ed['err.z'][-1] - 0.3) < 1e-9
            assert abs(ed['err.norm'][-1] - math.sqrt(0.14)) < 1e-9
            assert abs(ed['err.roll'][-1]) < 1e-6
            assert abs(ed['err.pitch'][-1]) < 1e-6
            assert abs(ed['err.yaw'][-1] - 90.0) < 1e-6
            assert node.mocap_pose['cftest'] == (1.1, 1.8, 3.3)

        # wrap case: mocap yaw 170 deg vs est yaw -170 deg -> err.yaw is the
        # short way around: -20 deg, NOT 340
        with node.lock:
            node.est_pose['cftest'] = (1.0, 2.0, 3.0, 0.0, 0.0, -170.0)
        npa2 = NamedPoseArray()
        npa2.header.stamp.sec = 101
        named2 = NamedPose()
        named2.name = 'cftest'
        named2.pose.position.x = 1.1
        named2.pose.position.y = 1.8
        named2.pose.position.z = 3.3
        named2.pose.orientation.z = math.sin(math.radians(85.0))  # yaw 170
        named2.pose.orientation.w = math.cos(math.radians(85.0))
        npa2.poses.append(named2)
        node.mocap_cb(npa2)
        with node.lock:
            assert abs(node.err_data['cftest']['err.yaw'][-1] + 20.0) < 1e-6

        # mocap Hz: 5 arrivals within the last second -> 5 Hz, decays to 0
        with node.lock:
            node.mocap_arrivals['cftest'] = deque(
                200.0 - 0.1 * i for i in range(5))
            assert node.mocap_rate('cftest', 200.0) == 5.0
            assert node.mocap_rate('cftest', 205.0) == 0.0
            assert node.mocap_rate('nosuchdrone', 200.0) == 0.0

        # two synthetic drones — cftest (both data sources) and cfzposeonly
        # (pose only; named to sort AFTER cftest) — exercise selection,
        # header, toggle and wrap-around
        with node.lock:
            node.data['cftest'] = {k: deque() for k in ('t',) + VARS}
            node.pose_data['cftest'] = {k: deque() for k in ('t',) + POSE_VARS}
            node.pose_data['cfzposeonly'] = {k: deque()
                                            for k in ('t',) + POSE_VARS}
            for i in range(20):
                node.data['cftest']['t'].append(i * 0.1)
                node.pose_data['cftest']['t'].append(i * 0.1)
                for v in VARS:
                    node.data['cftest'][v].append(float(i))
                for v in POSE_VARS:
                    node.pose_data['cftest'][v].append(float(i))
            node.new_drones.extend(['cftest', 'cfzposeonly'])
            node.status['cftest'] = st
        for _ in range(3):
            update(0)
        # (a) first drone auto-selected; header + position indicator render it
        assert sel['current'] == 'cftest'
        assert header_name.get_text() == 'cftest'
        assert header_pos.get_text() == 'drone 1/2'
        assert len(kal_lines[VARS[0]].get_xdata()) == 20
        assert len(pose_lines[POSE_VARS[0]].get_xdata()) == 20
        # 2x2 rebuild: all four plots (plus the deg twin axis) exist with
        # their line sets, and the two error samples injected above are
        # rendered on both unit axes
        assert set(axes) == {'k', 'p', 'c', 'e', 'e2'}
        assert len(conn_lines) == len(CONN_VARS)
        assert len(err_lines) == len(ERR_VARS) + len(ERR_DEG_VARS)
        assert len(err_lines[ERR_VARS[0]].get_xdata()) == 2
        assert len(err_lines[ERR_DEG_VARS[2]].get_xdata()) == 2
        assert err_lines[ERR_DEG_VARS[0]].get_linestyle() == '--'
        # (c) battery header: color-coded with ARMED suffix
        assert header_batt.get_text().endswith('ARMED')
        assert header_batt.get_color() == 'red'
        # misalignment banner: cftest's latest error sample has |yaw| 20 deg
        # (> ORIENT_WARN_DEG) and is the freshest -> banner + red panel title
        assert 'cftest' in warn_banner.get_text()
        assert 'yaw' in warn_banner.get_text()
        assert err_title['artist'].get_color() == '#cc3333'
        # a fresh small (1 deg) sample clears the warning again
        with node.lock:
            ed2 = node.err_data['cftest']
            ed2['t'].append(2.0)
            for k, val in zip(ERR_VARS + ERR_DEG_VARS,
                              (0.0, 0.0, 0.0, 0.0, 0.5, 0.2, 1.0)):
                ed2[k].append(val)
        update(0)
        assert warn_banner.get_text() == ''
        assert err_title['artist'].get_color() == plt.rcParams['text.color']
        # staleness: an old-only large error must NOT trigger the banner
        with node.lock:
            node.err_data['cfzposeonly'] = {
                k: deque() for k in ('t',) + ERR_VARS + ERR_DEG_VARS}
            sd = node.err_data['cfzposeonly']
            sd['t'].append(-50.0)  # 52 s older than the newest sample (2.0)
            for k, val in zip(ERR_VARS + ERR_DEG_VARS,
                              (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 150.0)):
                sd[k].append(val)
        update(0)
        assert warn_banner.get_text() == ''
        # (b) Next toggles to the pose-only drone; its kalman plot is empty
        cycle(1)
        update(0)
        assert sel['current'] == 'cfzposeonly'
        assert header_name.get_text() == 'cfzposeonly'
        assert header_pos.get_text() == 'drone 2/2'
        assert len(kal_lines[VARS[0]].get_xdata()) == 0
        assert header_batt.get_text() == '--.- V'
        assert header_batt.get_color() == 'gray'
        # selected-drone button labels follow the selection
        for prefix, b in sel_buttons.items():
            assert b.label.get_text() == f'{prefix} (cfzposeonly)'
        # (d) wrap-around: Next from the last returns to the first
        cycle(1)
        update(0)
        assert sel['current'] == 'cftest'
        assert header_name.get_text() == 'cftest'
        assert header_pos.get_text() == 'drone 1/2'
        assert btn_takeoff_one.label.get_text() == 'Takeoff (cftest)'

        # CSV recording into a temp dir: header, rows, known cell values
        tmpdir = tempfile.mkdtemp()
        node.start_recording(log_dir=tmpdir)
        assert node.csv_path is not None and node.csv_path.startswith(tmpdir)
        for _ in range(3):
            node.snapshot()
        update(0)
        assert btn_record.label.get_text().startswith('Stop CSV (')
        assert 'REC ->' in header_pos.get_text()
        path = node.csv_path
        node.stop_recording()
        assert node.csv_file is None
        with open(path) as f:
            csv_lines = f.read().splitlines()
        assert csv_lines[0] == ','.join(CSV_FIELDS)
        header_cols = csv_lines[0].split(',')
        assert all(col in header_cols
                   for col in ('err_roll', 'err_pitch', 'err_yaw'))
        rows = [line.split(',') for line in csv_lines[1:]]
        assert len(rows) >= 6  # >= 3 manual ticks x 2 drones
        cftest_rows = [r for r in rows if r[1] == 'cftest']
        assert cftest_rows
        assert cftest_rows[0][CSV_FIELDS.index(f'kal_{VARS[0]}')] == '19.0'
        assert cftest_rows[0][CSV_FIELDS.index('err_norm')] != ''
        assert cftest_rows[0][CSV_FIELDS.index('err_yaw')] != ''
        assert cftest_rows[0][CSV_FIELDS.index('mocap_x')] == '1.1'
        update(0)
        assert btn_record.label.get_text() == 'Record CSV'

        fig.canvas.draw()
        # layout sanity: with the bigger fonts and two button rows, the
        # bottom-row x-axis label must still sit above the selected-drone row
        xlabel_bb = axes['c'].xaxis.label.get_window_extent()
        row_bb = btn_takeoff_one.ax.get_window_extent()
        assert xlabel_bb.y0 >= row_bb.y1, (xlabel_bb, row_bb)
        # flush so the OK line survives even a watchdog force-exit
        print('selftest OK', flush=True)

    try:
        if args.selftest:
            selftest()
        else:
            plt.show()
    finally:
        # close a live CSV (flushes + logs the path), then tear down ROS. A
        # daemon watchdog force-exits if DDS teardown wedges (seen rarely
        # under a live 240 Hz /poses feed) - the process must never hang
        # after the window is closed.
        node.stop_recording()
        watchdog = threading.Timer(5.0, lambda: os._exit(0))
        watchdog.daemon = True
        watchdog.start()
        # join the executor thread before the interpreter exits: tearing down
        # the DDS layer while spin() is still inside it aborts the process
        # ("terminate called without an active exception")
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        watchdog.cancel()


if __name__ == '__main__':
    main()
