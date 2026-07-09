#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose

DRONES = ['cf1', 'cf2']

class PoseBridge(Node):
    def __init__(self):
        super().__init__('pose_bridge')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos.deadline = Duration(seconds=0, nanoseconds=int(1e9 / 240))
        self.poses_pub = self.create_publisher(NamedPoseArray, '/poses', qos)
        self.latest = {}
        for name in DRONES:
            self.create_subscription(
                PoseStamped, f'/{name}/pose',
                lambda msg, n=name: self.callback(msg, n), 10)
        self.create_timer(1.0/240.0, self.publish_all)
        self.get_logger().info(f'Pose bridge started for {DRONES}')

    def callback(self, msg, name):
        self.latest[name] = msg

    def publish_all(self):
        if not self.latest:
            return
        arr = NamedPoseArray()
        for name, msg in self.latest.items():
            arr.header = msg.header
            np_ = NamedPose()
            np_.name = name
            np_.pose = msg.pose
            arr.poses.append(np_)
        self.poses_pub.publish(arr)

def main():
    rclpy.init()
    rclpy.spin(PoseBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
