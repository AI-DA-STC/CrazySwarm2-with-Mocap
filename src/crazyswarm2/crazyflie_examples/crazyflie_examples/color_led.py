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

  # one-shot: set a color and exit
  ros2 run crazyflie_examples color_led Yellow
  ros2 run crazyflie_examples color_led Black

  # interactive: launch once, then type color names repeatedly
  ros2 run crazyflie_examples color_led
  color> Purple
  color> Black
  color> quit

Valid inputs (case-insensitive): Red, Yellow, Green, Blue, Purple, Pink,
White, Black (= off).  In the interactive prompt, 'q'/'quit'/'exit' or
Ctrl-D leaves it.

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
    'pink':   wrgb(255, 105, 180),
    'white':  wrgb(0, 0, 0, w=255),  # dedicated W channel (RGB mix looks purplish)
    'black':  0,                     # LED off
}


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
    """Look up a color name and set it. Returns True if applied."""
    value = COLORS.get(name.strip().lower())
    if value is None:
        print(f"unknown color '{name}'. "
              f"choose from: {', '.join(sorted(COLORS))}")
        return False
    result = ctl.set_color(value)
    if result is None:
        print('  set_parameters call timed out (server still there?)')
        return False
    print(f'  -> {name.strip().lower():6s} (0x{value:08X}) '
          f'on {len(ctl.led_params)} drone(s)')
    return True


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
            # One-shot: a color name was passed on the command line.
            rc = 0 if apply_named(ctl, args[0]) else 1
        else:
            # Interactive: prompt for colors until the user quits.
            print('Manual Color LED control. '
                  f'Colors: {", ".join(sorted(COLORS))}. '
                  "Type 'quit' to exit.")
            while True:
                try:
                    name = input('color> ')
                except (EOFError, KeyboardInterrupt):
                    print()
                    break
                if name.strip().lower() in ('q', 'quit', 'exit'):
                    break
                if not name.strip():
                    continue
                apply_named(ctl, name)
    finally:
        ctl.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
