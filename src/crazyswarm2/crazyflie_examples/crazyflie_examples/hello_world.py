"""Takeoff-hover-land for one CF. Useful to validate hardware config."""
from crazyflie_py import Crazyswarm

TAKEOFF_DURATION = 5.0
HOVER_DURATION = 5.0

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.arm(True)
    timeHelper.sleep(1.0)

    cf.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)

    cf.land(targetHeight=0.03, duration=3.0)
    timeHelper.sleep(TAKEOFF_DURATION)

    cf.arm(False)

if __name__ == '__main__':
    main()
