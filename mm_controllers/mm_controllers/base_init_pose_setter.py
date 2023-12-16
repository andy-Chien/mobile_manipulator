#!/usr/bin/env python3

import time
import math
import rclpy
import numpy as np

from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class BaseInitPoseSetter(Node):

    def __init__(self):
        super().__init__('base_init_pose_setter')

        # Initialize the transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    def set_init_pose(self, xyz=[0.0]*3, rpy=[0.0]*3):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp.sec = 0
        t.header.stamp.nanosec = 0
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]
        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        covariance = [0.0]*36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[-1] = 0.06853891909122467

        msg = PoseWithCovarianceStamped()
        msg.header = t.header
        
        msg.pose.pose.position.x = xyz[0]
        msg.pose.pose.position.y = xyz[1]
        msg.pose.pose.position.z = xyz[2]
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance = covariance

        i = 0
        while self.publisher.get_subscription_count() == 0 and i < 100:
            i += 1
            time.sleep(0.1)
        
        if i < 100:
            self.publisher.publish(msg)
            self.get_logger().info('Set base init pose success!')

def main():
    rclpy.init()
    node = BaseInitPoseSetter()
    try:
        node.get_logger().info('Try to set base init pose.')
        node.set_init_pose()
    except KeyboardInterrupt:
        node.get_logger().error('Set base init pose failed!')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
