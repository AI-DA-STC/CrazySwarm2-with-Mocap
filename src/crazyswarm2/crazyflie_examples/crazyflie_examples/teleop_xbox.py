#!/usr/bin/env python
"""
Fly one Crazyflie with an Xbox controller inside the mocap volume.

Run this AFTER the server, and disable the built-in teleop so that only
this script owns the controller and the drone:

    ros2 launch crazyflie launch.py teleop:=False
    ros2 run crazyflie_examples teleop_xbox

(`teleop:=False` also stops joy_node - this script reads /dev/input/js0
directly, so no joy_node is needed.)

CONTROLS (Xbox layout)
    A               take off to TAKEOFF_HEIGHT
    Back / View     land
    B               EMERGENCY - cuts motors instantly, drone drops.
                    Recovering needs a physical reset of the drone.
    left stick      move horizontally (world frame: up = +x, right = +y)
    right stick Y   up / down
    right stick X   yaw
    Ctrl-C          lands, then exits

Release the sticks and the drone holds its position - the sticks steer a
position setpoint, they are not direct velocity commands.

SAFETY
    * geofence: auto-lands if the mocap position leaves FENCE_RADIUS of
      (0, 0); the commanded target is also clamped to TARGET_RADIUS, so
      normal stick input cannot drive past the fence at all
    * height clamped to [Z_MIN, Z_MAX]
    * auto-lands if the mocap pose goes stale for POSE_TIMEOUT (a dead
      mocap is the fly-away risk on this rig, so it is treated as one)
    * the target may not run more than MAX_LEAD ahead of where the drone
      actually is, so it cannot wind up if the drone falls behind
    * refuses to take off without a live mocap pose for this drone

Check the controller mapping without flying anything:

    ros2 run crazyflie_examples teleop_xbox --joytest
"""
import math
import sys

from crazyflie_py import Crazyswarm

from motion_capture_tracking_interfaces.msg import NamedPoseArray

from rclpy.qos import qos_profile_sensor_data

# ---------------- control tuning ----------------
LOOP_HZ = 50.0
MAX_XY_SPEED = 0.6                   # m/s at full stick
MAX_Z_SPEED = 0.4                    # m/s at full stick
MAX_YAW_RATE = math.radians(70.0)    # rad/s at full stick
DEADBAND = 0.12                      # sticks measured up to 0.034 at rest

TAKEOFF_HEIGHT = 0.5                 # m
TAKEOFF_DURATION = 3.0               # s
LAND_HEIGHT = 0.03                   # m
LAND_DURATION = 3.0                  # s

# ---------------- safety envelope ----------------
FENCE_RADIUS = 2.0        # m from (0,0): auto-land past this
TARGET_RADIUS = 1.8       # m: commanded target is clamped here
Z_MIN, Z_MAX = 0.15, 1.5  # m
MAX_LEAD = 0.8            # m the target may lead the measured position
POSE_TIMEOUT = 0.5        # s without a mocap pose -> auto-land

# ---------------- Xbox mapping ----------------
# Verified on this rig: "Generic X-Box pad" via xpad, 8 axes / 11 buttons.
# Sticks: pushing up/left gives NEGATIVE values, hence the negations below.
AX_LEFT_X, AX_LEFT_Y = 0, 1
AX_RIGHT_X, AX_RIGHT_Y = 3, 4
BTN_TAKEOFF = 0     # A
BTN_EMERGENCY = 1   # B
BTN_LAND = 6        # Back / View


def deadband(value, width=DEADBAND):
    """Zero out stick noise near centre, rescaling the rest to full range."""
    if abs(value) < width:
        return 0.0
    return (value - math.copysign(width, value)) / (1.0 - width)


def yaw_of(quaternion):
    """Yaw angle (rad) of a geometry_msgs Quaternion."""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def clamp(value, low, high):
    return max(low, min(high, value))


def joytest():
    """Print live axis/button values so the mapping can be verified."""
    from crazyflie_py import linuxjsdev
    js = linuxjsdev.Joystick()
    devices = js.devices()
    if not devices:
        print('No joystick found at /dev/input/js*.')
        return 1
    dev_id = devices[0]['id']
    print(f"reading '{devices[0]['name']}' (js{dev_id}) - Ctrl-C to stop")
    js.open(dev_id)
    try:
        while True:
            axes, buttons = js.read(dev_id)
            pressed = [i for i, b in enumerate(buttons) if b]
            print('axes ' + ' '.join(f'{i}:{a:+.2f}' for i, a in enumerate(axes))
                  + f'  buttons {pressed}', end='\r', flush=True)
    except KeyboardInterrupt:
        print()
    return 0


class MocapWatch:
    """Latest mocap pose for one named rigid body, straight off /poses."""

    def __init__(self, node, name):
        self.node = node
        self.name = name
        self.position = None       # (x, y, z)
        self.yaw = 0.0
        self.stamp = None          # seconds, node clock
        self.seen_names = set()
        # /poses is published best-effort (sensor QoS). Subscribing with the
        # default reliable QoS is incompatible, and ROS then silently delivers
        # NOTHING - the pose just never arrives.
        node.create_subscription(NamedPoseArray, '/poses', self._callback,
                                 qos_profile_sensor_data)

    def _callback(self, msg):
        for named in msg.poses:
            self.seen_names.add(named.name)
            if named.name == self.name:
                p = named.pose.position
                self.position = (p.x, p.y, p.z)
                self.yaw = yaw_of(named.pose.orientation)
                self.stamp = self.node.get_clock().now().nanoseconds / 1e9
                return

    def age(self, now):
        """Seconds since the last pose, or None if none ever arrived."""
        if self.stamp is None:
            return None
        return now - self.stamp

    def fresh(self, now):
        age = self.age(now)
        return age is not None and age <= POSE_TIMEOUT

    def radius(self):
        if self.position is None:
            return 0.0
        return math.hypot(self.position[0], self.position[1])


class Pilot:
    """Button-edge detection and the takeoff/fly/land state machine."""

    def __init__(self, swarm):
        self.swarm = swarm
        self.timeHelper = swarm.timeHelper
        self.node = swarm.allcfs
        self.cf = swarm.allcfs.crazyflies[0]
        self.name = self.cf.prefix.lstrip('/')
        self.mocap = MocapWatch(self.node, self.name)

        joystick = swarm.input
        if joystick.joyID is None:
            raise RuntimeError(
                'No joystick found at /dev/input/js*. Plug in the controller '
                'and check it appears (ls /dev/input/js*).')
        self.js = joystick.js
        self.joy_id = joystick.joyID
        self.buttons = []
        self.axes = []
        self._previous = []

        self.target = None   # [x, y, z] commanded position
        self.yaw = 0.0
        self.stopped = False  # set by the emergency stop; ends the script

    # ---------------- joystick ----------------

    def poll(self):
        """Refresh axis/button state; keep the previous state for edges."""
        self._previous = list(self.buttons)
        self.axes, self.buttons = self.js.read(self.joy_id)

    def pressed(self, index):
        """Return True on the frame a button goes down (not while held)."""
        if index >= len(self.buttons):
            return False
        was = self._previous[index] if index < len(self._previous) else 0
        return self.buttons[index] and not was

    def axis(self, index):
        if index >= len(self.axes):
            return 0.0
        return deadband(self.axes[index])

    # ---------------- helpers ----------------

    def wait(self, duration):
        """
        Spin for duration, aborting early if EMERGENCY is pressed.

        Returns False if the emergency stop fired.
        """
        end = self.timeHelper.time() + duration
        while self.timeHelper.time() < end:
            self.timeHelper.sleepForRate(LOOP_HZ)
            self.poll()
            if self.pressed(BTN_EMERGENCY):
                self.emergency()
                return False
        return True

    def emergency(self):
        print('\nEMERGENCY STOP - motors cut. '
              'Reset the drone physically before flying again.')
        self.stopped = True
        self.cf.emergency()

    # ---------------- phases ----------------

    def takeoff(self):
        """Arm and take off. Returns True once airborne and streaming."""
        now = self.timeHelper.time()
        if not self.mocap.fresh(now):
            if self.mocap.stamp is None:
                names = sorted(self.mocap.seen_names)
                print('\nCannot take off: no mocap pose for '
                      f"'{self.name}' on /poses.")
                print(f'  bodies currently seen: {names if names else "none"}')
                print('  Check the rigid body name in Motive matches the '
                      'drone name in crazyflies.yaml.')
            else:
                print(f'\nCannot take off: mocap pose is stale '
                      f'({self.mocap.age(now):.1f} s old).')
            return False

        print(f'\narming {self.name}')
        self.cf.arm(True)
        if not self.wait(1.0):
            return False

        print(f'taking off to {TAKEOFF_HEIGHT:.2f} m')
        self.cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
        if not self.wait(TAKEOFF_DURATION + 0.5):
            return False

        # Seed the setpoint from where the drone actually is, so handing
        # over from the high-level takeoff does not jump.
        x, y, _ = self.mocap.position
        self.target = [x, y, TAKEOFF_HEIGHT]
        self.yaw = self.mocap.yaw
        print('flying - left stick moves, right stick Y up/down, '
              'right stick X yaws, Back lands')
        return True

    def step(self, dt):
        """One control cycle. Returns a reason string to land, or None."""
        now = self.timeHelper.time()

        if not self.mocap.fresh(now):
            age = self.mocap.age(now)
            return f'mocap pose stale ({age:.2f} s)' if age else 'mocap pose lost'

        radius = self.mocap.radius()
        if radius > FENCE_RADIUS:
            return f'geofence breached ({radius:.2f} m > {FENCE_RADIUS:.2f} m)'

        if self.pressed(BTN_LAND):
            return 'land requested'

        # Sticks steer the position setpoint. Left stick up (negative axis)
        # is +x, left stick left (negative axis) is +y, matching a world
        # frame viewed from behind the origin.
        vx = -self.axis(AX_LEFT_Y) * MAX_XY_SPEED
        vy = -self.axis(AX_LEFT_X) * MAX_XY_SPEED
        vz = -self.axis(AX_RIGHT_Y) * MAX_Z_SPEED
        yaw_rate = -self.axis(AX_RIGHT_X) * MAX_YAW_RATE

        self.target[0] += vx * dt
        self.target[1] += vy * dt
        self.target[2] = clamp(self.target[2] + vz * dt, Z_MIN, Z_MAX)
        self.yaw += yaw_rate * dt

        # Keep the target inside the fence...
        target_radius = math.hypot(self.target[0], self.target[1])
        if target_radius > TARGET_RADIUS:
            scale = TARGET_RADIUS / target_radius
            self.target[0] *= scale
            self.target[1] *= scale

        # ...and never let it wind up far ahead of the real drone.
        px, py, pz = self.mocap.position
        for i, measured in enumerate((px, py, pz)):
            self.target[i] = clamp(self.target[i],
                                   measured - MAX_LEAD, measured + MAX_LEAD)
        self.target[2] = clamp(self.target[2], Z_MIN, Z_MAX)

        self.cf.cmdPosition(self.target, self.yaw)
        return None

    def land(self, reason):
        print(f'\nlanding: {reason}')
        # Hand control back to the onboard high-level planner before land().
        self.cf.notifySetpointsStop(100)
        self.timeHelper.sleep(0.1)
        self.cf.land(targetHeight=LAND_HEIGHT, duration=LAND_DURATION)
        self.timeHelper.sleep(LAND_DURATION + 1.0)
        self.cf.arm(False)
        print('landed and disarmed')

    # ---------------- main loop ----------------

    def run(self):
        dt = 1.0 / LOOP_HZ
        print(f'{self.name}: press A to take off, B is EMERGENCY '
              '(Ctrl-C also lands)')
        print(f'geofence {FENCE_RADIUS:.1f} m from (0,0), '
              f'height {Z_MIN:.2f}-{Z_MAX:.2f} m')

        flying = False
        next_report = 0.0
        try:
            while True:
                self.timeHelper.sleepForRate(LOOP_HZ)
                self.poll()

                if self.pressed(BTN_EMERGENCY):
                    self.emergency()
                    return

                if not flying:
                    now = self.timeHelper.time()
                    if now >= next_report:
                        next_report = now + 1.0
                        if self.mocap.fresh(now):
                            x, y, z = self.mocap.position
                            status = f'mocap ok  x={x:+.2f} y={y:+.2f} z={z:+.2f}'
                        else:
                            status = 'waiting for mocap pose...'
                        print(f'  on ground - {status}   ', end='\r', flush=True)

                    if self.pressed(BTN_TAKEOFF):
                        flying = self.takeoff()
                        if self.stopped:
                            return
                    continue

                reason = self.step(dt)
                if reason:
                    self.land(reason)
                    return
        except KeyboardInterrupt:
            if flying:
                self.land('Ctrl-C')
            else:
                print('\nexiting')


def main():
    if '--joytest' in sys.argv:
        return joytest()

    swarm = Crazyswarm()
    try:
        pilot = Pilot(swarm)
    except (RuntimeError, IndexError) as exc:
        print(exc if isinstance(exc, RuntimeError) else
              'No crazyflies found - is one enabled in crazyflies.yaml?')
        return 1
    pilot.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
