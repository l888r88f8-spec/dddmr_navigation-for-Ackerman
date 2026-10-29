#!/usr/bin/env python3
"""Publish the standalone map->base static transform using shared ROS parameters."""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def parse_arguments():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
    parser.add_argument('--roll', type=float, required=True)
    parser.add_argument('--pitch', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True)
    return parser.parse_known_args()


def quaternion_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    args, ros_arguments = parse_arguments()
    rclpy.init(args=ros_arguments)
    node = None
    try:
        node = Node('static_transform_from_params')
        node.declare_parameter('global_frame', 'map')
        node.declare_parameter('robot_base_frame', 'base_link')
        global_frame = node.get_parameter('global_frame').value
        robot_base_frame = node.get_parameter('robot_base_frame').value
        if not global_frame or not robot_base_frame:
            raise ValueError('global_frame and robot_base_frame must be nonempty')

        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = global_frame
        transform.child_frame_id = robot_base_frame
        transform.transform.translation.x = args.x
        transform.transform.translation.y = args.y
        transform.transform.translation.z = args.z
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = quaternion_from_rpy(args.roll, args.pitch, args.yaw)

        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(transform)
        node.get_logger().info(
            'Published static TF {} -> {} at ({:.2f}, {:.2f}, {:.2f})'.format(
                global_frame, robot_base_frame, args.x, args.y, args.z))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
