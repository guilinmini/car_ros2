#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """Relay nav_msgs/Odometry into TF as a dynamic transform."""

    def __init__(self):
        super().__init__("odom_to_tf")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.declare_parameter("frame_id", "wheeltec/odom")
        self.declare_parameter("child_frame_id", "wheeltec/base_link")

    def odom_callback(self, msg: Odometry):
        parent = msg.header.frame_id or self.get_parameter("frame_id").get_parameter_value().string_value
        child = msg.child_frame_id or self.get_parameter("child_frame_id").get_parameter_value().string_value

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
