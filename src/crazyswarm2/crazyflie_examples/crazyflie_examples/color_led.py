#!/usr/bin/env python3
"""Manually control the bottom-mounted Color LED deck (bcColorLedBot).

Sets the firmware param ``colorLedBot.wrgb8888`` (uint32, packed 0xWWRRGGBB)
on every crazyflie by calling the crazyflie_server's standard ROS parameter
service ``/crazyflie_server/set_parameters`` directly -- exactly what
``ros2 param set /crazyflie_server <cf>.params.colorLedBot.wrgb8888 <n>`` does.

This deliberately does NOT use crazyflie_py's Crazyswarm()/CrazyflieServer(),
because that waits on the ``all/emergency`` broadcast service during init and
hangs on this rig. Talking to set_parameters is reliable and needs only a live
crazyflie_server (i.e. ``ros2 launch crazyflie launch.py`` running).

Two ways to use it:

  # one-shot: set a color (by name or number key) and exit
  ros2 run crazyflie_examples color_led Yellow
  ros2 run crazyflie_examples color_led 3
  ros2 run crazyflie_examples color_led Black

  # interactive: launch once, then press number keys (no Enter needed)
  ros2 run crazyflie_examples color_led

Number keys: 0=off(black)  1=green  2=red  3=yellow  4=blue  5=purple
9=white.  Color names (case-insensitive) also work as one-shot args:
Red, Yellow, Green, Blue, Purple, White, Black (= off).  In the
interactive mode, 'q' or Ctrl-C leaves it.

Hardware only: the sim backend declares no firmware params, so this has no
visible effect with backend:=sim.
"""

import sys

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import ListParameters, SetParameters


SERVER = '/crazyflie_server'
# ROS param names look like "<cf>.params.colorLedBot.wrgb8888" (one per drone).
LED_SUFFIX = '.params.colorLedBot.wrgb8888'


def wrgb(r, g, b, w=0):
    """Pack channel bytes (0-255) into the 0xWWRRGGBB param value."""
    return ((w & 0xFF) << 24) | ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)


# Named colors -> packed wrgb value. Edit the RGB tuples to taste.
COLORS = {
    'red':    wrgb(255, 0, 0),
    'yellow': wrgb(255, 255, 0),
    'green':  wrgb(0, 255, 0),
    'blue':   wrgb(0, 0, 255),
    'purple': wrgb(148, 0, 211),
    'white':  wrgb(0, 0, 0, w=255),  # dedicated W channel (RGB mix looks purplish)
    'black':  0,                     # LED off
}

# Number keys -> color names (0 = off). Keys 6-8 intentionally unassigned.
KEYMAP = {
    '0': 'black',
    '1': 'green',
    '2': 'red',
    '3': 'yellow',
    '4': 'blue',
    '5': 'purple',
    '9': 'white',
}

KEY_HELP = '0=off(black)  1=green  2=red  3=yellow  4=blue  5=purple  9=white'


class LedController(Node):
    """Minimal node that drives colorLedBot.wrgb8888 via set_parameters."""

    def __init__(self):
        super().__init__('color_led_ctl')
        self._set_cli = self.create_client(
            SetParameters, f'{SERVER}/set_parameters')
        self._list_cli = self.create_client(
            ListParameters, f'{SERVER}/list_parameters')

        if not self._set_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                f'{SERVER}/set_parameters not available after 10s. '
                'Is the crazyflie_server running (ros2 launch crazyflie '
                'launch.py) and on the same ROS_DOMAIN_ID?')

        self.led_params = self._discover_led_params()

    def _discover_led_params(self):
        """Return the full param name of every drone's LED color param."""
        if not self._list_cli.wait_for_service(timeout_sec=5.0):
            return []
        req = ListParameters.Request()
        req.prefixes = []
        req.depth = 0  # 0 == recurse all
        future = self._list_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        if res is None:
            return []
        return sorted(n for n in res.result.names if n.endswith(LED_SUFFIX))

    def set_color(self, value):
        """Set the packed color value on all discovered LED params."""
        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=int(value)))
            for name in self.led_params
        ]
        future = self._set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()


def apply_named(ctl, name):
    """Look up a color name or number key and set it. Returns True if applied."""
    token = name.strip().lower()
    token = KEYMAP.get(token, token)  # number key -> color name
    value = COLORS.get(token)
    if value is None:
        print(f"unknown color '{name}'. "
              f"choose from: {', '.join(sorted(COLORS))} (or keys: {KEY_HELP})")
        return False
    result = ctl.set_color(value)
    if result is None:
        print('  set_parameters call timed out (server still there?)')
        return False
    print(f'  -> {token:6s} (0x{value:08X}) '
          f'on {len(ctl.led_params)} drone(s)')
    return True


def interactive_loop(ctl):
    """One keypress = one color change (no Enter). 'q' or Ctrl-C quits."""
    print('Manual Color LED control - press a number key:')
    print(f'  {KEY_HELP}   q=quit')

    if not sys.stdin.isatty():
        # Piped stdin (no terminal): fall back to one token per line.
        for line in sys.stdin:
            token = line.strip().lower()
            if token in ('q', 'quit', 'exit'):
                break
            if token:
                apply_named(ctl, token)
        return

    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        while True:
            # cbreak only while waiting for the key, so apply_named prints
            # normally (cooked mode) and Ctrl-C still raises KeyboardInterrupt.
            tty.setcbreak(fd)
            try:
                key = sys.stdin.read(1)
            except KeyboardInterrupt:
                break
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            if key in ('', 'q', 'Q', '\x03', '\x04'):
                break
            if key in KEYMAP:
                apply_named(ctl, key)
            # anything else (incl. escape sequences) is ignored
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


def main():
    rclpy.init(args=sys.argv)
    try:
        ctl = LedController()
    except RuntimeError as exc:
        print(exc)
        rclpy.shutdown()
        return 1

    if not ctl.led_params:
        print('WARNING: no "colorLedBot.wrgb8888" params found on the server. '
              'Check the Color LED deck is attached and that server.yaml has '
              'firmware_params.query_all_values_on_connect: True.')

    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    rc = 0
    try:
        if args:
            # One-shot: a color name or number key was passed on the command line.
            rc = 0 if apply_named(ctl, args[0]) else 1
        else:
            # Interactive: number-key control until the user quits.
            try:
                interactive_loop(ctl)
            except (EOFError, KeyboardInterrupt):
                print()
    finally:
        ctl.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
