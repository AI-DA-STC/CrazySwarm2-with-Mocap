#!/usr/bin/env python3
"""Bridge per-drone OptiTrack poses from natnet_ros2 into crazyswarm2.

natnet_ros2 publishes one PoseStamped per rigid body on /<name>/pose.
crazyflie_server expects a single motion_capture_tracking_interfaces/NamedPoseArray
on /poses. This node aggregates the former into the latter.

PUBLISH_HZ MUST match the mocap streaming rate configured in Motive (see
docs/MOCAP.md). Each pose is forwarded to the drone over the Crazyradio link, so
this rate is also a radio-bandwidth knob: keep it at 50 Hz for a 2-drone swarm,
not 240 Hz. Raising it past the Motive frame rate only republishes stale poses.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose

# Rigid-body names. MUST match the names in Motive, crazyflies.yaml, and the
# /<name>/pose topics published by natnet_ros2.
DRONES = ['cf1', 'cf2']

# Mocap streaming rate (Hz). Keep in sync with Motive's point-cloud rate and
# motion_capture.yaml -> poses.qos.deadline.
PUBLISH_HZ = 50.0


class PoseBridge(Node):
    def __init__(self):
        super().__init__('pose_bridge')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        qos.deadline = Duration(seconds=0, nanoseconds=int(1e9 / PUBLISH_HZ))
        self.poses_pub = self.create_publisher(NamedPoseArray, '/poses', qos)
        self.latest = {}
        for name in DRONES:
            self.create_subscription(
                PoseStamped, f'/{name}/pose',
                lambda msg, n=name: self.callback(msg, n), 10)
        self.create_timer(1.0 / PUBLISH_HZ, self.publish_all)
        self.get_logger().info(f'Pose bridge started for {DRONES} @ {PUBLISH_HZ:.0f} Hz')

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
