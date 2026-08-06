#!/usr/bin/env python
"""Arm and spin all propellers at low power for 10 s, then stop.

Ground test only - keep the drone on the floor, clear of fingers.

A brushed CF2.1 does NOT idle-spin its propellers on arming (that is
Bolt/brushless behavior); arming just sets a supervisor flag, and the
firmware auto-disarms again after a few idle seconds. So for a visible
check this drives the firmware's motor-test params (motorPowerSet.*,
the same path cfclient's propeller test uses). SPIN_PWM is far below
hover thrust, so the drone stays put.
"""
from crazyflie_py import Crazyswarm

SPIN_PWM = 10000      # of 65535 (~15%); hover needs roughly 38000+
SPIN_SECONDS = 10.0
MOTORS = ('m1', 'm2', 'm3', 'm4')


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies

    for cf in cfs:
        print(f'arming {cf.prefix}')
        cf.arm(True)
    timeHelper.sleep(1.0)

    try:
        for cf in cfs:
            print(f'spinning propellers on {cf.prefix} '
                  f'at {SPIN_PWM}/65535 for {SPIN_SECONDS:.0f} s')
            for m in MOTORS:
                cf.setParam(f'motorPowerSet.{m}', SPIN_PWM)
            cf.setParam('motorPowerSet.enable', 1)

        timeHelper.sleep(SPIN_SECONDS)
    finally:
        # Always stop the motors, even on Ctrl-C or a crash mid-spin.
        for cf in cfs:
            cf.setParam('motorPowerSet.enable', 0)
            for m in MOTORS:
                cf.setParam(f'motorPowerSet.{m}', 0)
            cf.arm(False)
        timeHelper.sleep(0.5)  # let the stop commands reach the radio
    print('done')


if __name__ == '__main__':
    main()
