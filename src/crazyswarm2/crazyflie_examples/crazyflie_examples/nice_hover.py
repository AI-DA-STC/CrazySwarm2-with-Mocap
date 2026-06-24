#!/usr/bin/env python
from crazyflie_py import Crazyswarm
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Status, LogDataGeneric
import numpy as np
import csv
import time
import threading
import os
from datetime import datetime

# ---------------- Config ----------------
LOG_FILE = os.path.join(
    os.path.expanduser('~'),
    f'cf1_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
)
DRONE_NAME = 'cf1'

# ---------------- Logger ----------------
class DroneLogger:
    def __init__(self, node):
        self.latest_pose = None
        self.latest_status = None
        self.latest_estimator = [None, None, None]
        self.latest_attitude = [None, None, None]
        self.latest_kalman = [None, None, None]
        self.lock = threading.Lock()

        self.csv_file = open(LOG_FILE, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp',
            'pos_x', 'pos_y', 'pos_z',
            'ori_x', 'ori_y', 'ori_z', 'ori_w',
            'battery_v', 'rssi',
            'stateEstimate_x', 'stateEstimate_y', 'stateEstimate_z',
            'stabilizer_roll', 'stabilizer_pitch', 'stabilizer_yaw',
            'kalman_stateX', 'kalman_stateY', 'kalman_stateZ'
        ])

        node.create_subscription(PoseStamped, f'/{DRONE_NAME}/pose', self.pose_callback, 10)
        node.create_subscription(Status, f'/{DRONE_NAME}/status', self.status_callback, 10)
        node.create_subscription(LogDataGeneric, f'/{DRONE_NAME}/estimator', self.estimator_callback, 10)
        node.create_subscription(LogDataGeneric, f'/{DRONE_NAME}/attitude', self.attitude_callback, 10)
        node.create_subscription(LogDataGeneric, f'/{DRONE_NAME}/kalman', self.kalman_callback, 10)

        print(f'Logging to {LOG_FILE}')

    def pose_callback(self, msg):
        with self.lock:
            self.latest_pose = msg
            self._write_row()

    def status_callback(self, msg):
        with self.lock:
            self.latest_status = msg

    def estimator_callback(self, msg):
        with self.lock:
            if len(msg.values) >= 3:
                self.latest_estimator = list(msg.values[:3])

    def attitude_callback(self, msg):
        with self.lock:
            if len(msg.values) >= 3:
                self.latest_attitude = list(msg.values[:3])

    def kalman_callback(self, msg):
        with self.lock:
            if len(msg.values) >= 3:
                self.latest_kalman = list(msg.values[:3])

    def _write_row(self):
        if self.latest_pose is None:
            return
        p = self.latest_pose.pose.position
        o = self.latest_pose.pose.orientation
        batt = self.latest_status.battery_voltage if self.latest_status else 0.0
        rssi = self.latest_status.rssi if self.latest_status else 0
        self.writer.writerow([
            time.time(),
            p.x, p.y, p.z,
            o.x, o.y, o.z, o.w,
            batt, rssi,
            self.latest_estimator[0],
            self.latest_estimator[1],
            self.latest_estimator[2],
            self.latest_attitude[0],
            self.latest_attitude[1],
            self.latest_attitude[2],
            self.latest_kalman[0],
            self.latest_kalman[1],
            self.latest_kalman[2]
        ])

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        print(f'Log saved to {LOG_FILE}')


# ---------------- Main ----------------
def main():
    Z = 0.5
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf = allcfs.crazyflies[0]

    node = cf.node
    logger = DroneLogger(node)

    try:
        # Arm
        for cf in allcfs.crazyflies:
            cf.arm(True)
        timeHelper.sleep(1.0)

        # Takeoff
        allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
        timeHelper.sleep(1.0+Z)

        # Hover
        print('Hovering...')
        timeHelper.sleep(3.0)

        # Land
        allcfs.land(targetHeight=0.02, duration=1.0+Z)
        timeHelper.sleep(1.0+Z)

        # Disarm
        allcfs.arm(False)
        print('Done.')

    finally:
        logger.close()

if __name__ == '__main__':
    main()
