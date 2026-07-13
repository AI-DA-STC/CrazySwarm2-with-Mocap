#!/usr/bin/env python3
"""Light up the bottom-mounted Color LED deck (bcColorLedBot).

The deck exposes two firmware params (firmware >= 2025.12):
  colorLedBot.wrgb8888   uint32 color packed as 0xWWRRGGBB (W = white channel)
  colorLedBot.brightCorr uint8  gamma/luminance correction (default 1)

Hardware only: the sim backend declares no firmware params, so this
script has no visible effect with backend:=sim.
"""

from crazyflie_py import Crazyswarm


def wrgb(r, g, b, w=0):
    """Pack channel bytes (0-255) into the 0xWWRRGGBB param value."""
    return (w << 24) | (r << 16) | (g << 8) | b


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    print('test')
    for cf in swarm.allcfs.crazyflies:
        # needs query_all_values_on_connect: True in server.yaml
        if cf.getParam('deck.bcColorLedBot') != 1:
            print(f'WARNING: {cf.prefix} has no bottom Color LED deck detected')
        cf.setParam('colorLedBot.brightCorr', 1)
        cf.setParam('colorLedBot.wrgb8888', wrgb(0, 255, 0))  # solid green
    timeHelper.sleep(2.0)

    # cycle red -> green -> blue -> white -> off
    for color in (wrgb(255, 0, 0),
                  wrgb(0, 255, 0),
                  wrgb(0, 0, 255),
                  wrgb(0, 0, 0, w=255),
                  0):
        for cf in swarm.allcfs.crazyflies:
            cf.setParam('colorLedBot.wrgb8888', color)
        timeHelper.sleep(1.0)


if __name__ == '__main__':
    print('test 1')
    main()
    print('test2')
