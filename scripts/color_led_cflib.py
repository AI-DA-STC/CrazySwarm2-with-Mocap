#!/usr/bin/env python3
"""
Standalone cflib script to light up the bottom-mounted Color LED deck on a
Crazyflie 2.1.

This bypasses crazyswarm2 / ROS 2 entirely and talks to the Crazyflie directly
over the Crazyradio USB dongle using cflib.

IMPORTANT — radio contention:
    cflib and the crazyswarm2 `crazyflie_server` BOTH use the Crazyradio dongle
    and cannot both own the link at the same time. STOP your `ros2 launch ...`
    (the crazyflie_server) BEFORE running this script, otherwise the link will
    fail to open.

Usage:
    # First stop the running crazyswarm2 launch / crazyflie_server, then:
    python3 scripts/color_led_cflib.py

Deck params used (firmware 2026.04):
    colorLedBot.wrgb8888  (uint32, packed 0xWWRRGGBB)  -> the color to display
    colorLedBot.brightCorr (uint8, 0/1, default 1)      -> brightness correction
"""

import sys
import time
import logging

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ----------------------------------------------------------------------------
# EDIT ME: URI of the Crazyflie to connect to (cf11 from crazyflies.yaml).
# ----------------------------------------------------------------------------
URI = 'radio://0/81/2M/E7E7E7E711'

# Param names on the Color LED deck (bottom).
PARAM_COLOR = 'colorLedBot.wrgb8888'
PARAM_BRIGHT = 'colorLedBot.brightCorr'

# Only surface errors from cflib's own logging; our status prints use flush.
logging.basicConfig(level=logging.ERROR)


def wrgb(r, g, b, w=0):
    """Pack white/red/green/blue (each 0-255) into a uint32 0xWWRRGGBB value."""
    return ((w & 0xFF) << 24) | ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF)


# Color sequence: (label, r, g, b, w, hold_seconds)
SEQUENCE = [
    ('green', 0, 255, 0, 0, 2.0),
    ('red',   255, 0, 0, 0, 1.5),
    ('green', 0, 255, 0, 0, 1.5),
    ('blue',  0, 0, 255, 0, 1.5),
    ('white', 0, 0, 0, 255, 1.5),
    ('off',   0, 0, 0, 0, 1.0),
]


def run():
    cflib.crtp.init_drivers()

    # rw_cache lets cflib cache the log/param TOC between runs so it does not
    # have to be re-downloaded every connect (faster reconnects). Harmless if
    # the dir does not exist yet — cflib creates it.
    cf = Crazyflie(rw_cache='./cache')

    print('Connecting to %s ...' % URI, flush=True)

    # SyncCrazyflie.open_link() (via the context manager) blocks until the
    # link is up AND the log/param TOCs have been downloaded. param.set_value()
    # additionally waits internally for the param TOC to be initialised, so it
    # is safe to set params immediately after the context manager enters.
    try:
        scf = SyncCrazyflie(URI, cf=cf)
        scf.open_link()
    except Exception as exc:  # noqa: BLE001 - any link error should print the hint
        print('Could not open radio link — is the crazyflie_server / ros2 '
              'launch still running? Stop it first (it holds the Crazyradio).',
              flush=True)
        print('  (underlying error: %s)' % exc, flush=True)
        return 1

    try:
        print('Connected. Enabling brightness correction.', flush=True)
        scf.cf.param.set_value(PARAM_BRIGHT, 1)

        for label, r, g, b, w, hold in SEQUENCE:
            value = wrgb(r, g, b, w)
            print('  -> %-6s (0x%08X)' % (label, value), flush=True)
            scf.cf.param.set_value(PARAM_COLOR, value)
            time.sleep(hold)

        # Make sure the LED ends up off before we disconnect.
        scf.cf.param.set_value(PARAM_COLOR, 0)
        print('Done. LED off.', flush=True)
    finally:
        scf.close_link()

    return 0


if __name__ == '__main__':
    sys.exit(run())
