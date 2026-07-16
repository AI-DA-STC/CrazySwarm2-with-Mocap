import atexit
import time

import rclpy

from . import genericJoystick
from .crazyflie import CrazyflieServer, TimeHelper

# Bottom Color LED deck status colors (colorLedBot.wrgb8888, 0xWWRRGGBB).
# The server sets green on connect; every crazyflie_py script turns the deck
# red for its lifetime and restores green on exit (normal return, exception,
# or Ctrl-C - atexit runs on interpreter shutdown). Drones without the deck
# ignore the broadcast, and on backend:=sim the param does not exist, in
# which case setParam just logs a warning.
LED_SCRIPT_RUNNING = 0x00FF0000  # red
LED_IDLE = 0x0000FF00            # green


class Crazyswarm:

    def __init__(self):
        rclpy.init()

        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)

        self.input = genericJoystick.Joystick(self.timeHelper)

        self.allcfs.setParam('colorLedBot.wrgb8888', LED_SCRIPT_RUNNING)
        atexit.register(self._restore_status_led)

    def _restore_status_led(self):
        try:
            self.allcfs.setParam('colorLedBot.wrgb8888', LED_IDLE)
            # setParam only enqueues the service request (call_async); give
            # DDS a moment to put it on the wire before the process dies.
            time.sleep(0.3)
        except Exception:
            pass  # rclpy may already be shut down; LED then keeps last color
