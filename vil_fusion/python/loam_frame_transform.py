#!/usr/bin/env python2
"""
I am sorry this uses Python 2. It was easier to write this in Python 2 than to get ROS TF
to work in Python 3.

LOAM outputs everything in its own weird frame. This node transforms all of the relevant LOAM messages into
the ROS frame (X forward, Y left, Z up), and re-publishes them.
"""

import tf.transformations
from tf import Transformer, TransformBroadcaster, LookupException, TransformListener
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from loam_velodyne.msg import OdometryWithHessian
from geometry_msgs.msg import PoseStamped

ros_to_loam = tf.transformations.inverse_matrix(
    np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float32)
)


class LoamFrameTransform:
    def __init__(self, node_name="loam_frame_transform"):
        rospy.init_node(node_name)

        self.loam_init_frame_id = "loam_init"

        self.tf_listener = TransformListener(False)
        self.tf_broadcaster = TransformBroadcaster()

        self.pub_loam_odom = rospy.Publisher("~odometry/ros", OdometryWithHessian, queue_size=1)
        self.pub_loam_odom_only = rospy.Publisher("~odometry/ros/odomonly", Odometry, queue_size=1)
        self.pub_loam_mapping = rospy.Publisher("~mapping/ros", OdometryWithHessian, queue_size=1)
        self.pub_loam_mapping_only = rospy.Publisher("~mapping/ros/odomonly", Odometry, queue_size=1)
        self.sub_loam_odom = rospy.Subscriber(
            "~odometry/loam",
            OdometryWithHessian,
            callback=lambda msg: self.loam_odom_callback_2(msg, self.pub_loam_odom, self.pub_loam_odom_only))
        self.sub_loam_mapping = rospy.Subscriber(
            "~mapping/loam",
            OdometryWithHessian,
            callback=lambda msg: self.loam_odom_callback_2(msg, self.pub_loam_mapping, self.pub_loam_mapping_only))

    def loam_odom_callback_2(self, msg, pub, pub_odomonly):
        (
            msg.odom.pose.pose.position.x,
            msg.odom.pose.pose.position.y,
            msg.odom.pose.pose.position.z,
        ) = (
            msg.odom.pose.pose.position.z,
            msg.odom.pose.pose.position.x,
            msg.odom.pose.pose.position.y,
        )

        (
            msg.odom.pose.pose.orientation.x,
            msg.odom.pose.pose.orientation.y,
            msg.odom.pose.pose.orientation.z,
        ) = (
            msg.odom.pose.pose.orientation.z,
            msg.odom.pose.pose.orientation.x,
            msg.odom.pose.pose.orientation.y,
        )

        (
            msg.odom.twist.twist.linear.x,
            msg.odom.twist.twist.linear.y,
            msg.odom.twist.twist.linear.z,
        ) = (
            msg.odom.twist.twist.linear.z,
            msg.odom.twist.twist.linear.x,
            msg.odom.twist.twist.linear.y,
        )

        (
            msg.odom.twist.twist.angular.x,
            msg.odom.twist.twist.angular.y,
            msg.odom.twist.twist.angular.z,
        ) = (
            msg.odom.twist.twist.angular.z,
            msg.odom.twist.twist.angular.x,
            msg.odom.twist.twist.angular.y,
        )

        msg.odom.header.frame_id = "loam_init_ros_convention"
        msg.odom.child_frame_id += "_ros"

        pub.publish(msg)
        pub_odomonly.publish(msg.odom)

        new_trans = np.array([
            msg.odom.pose.pose.position.x,
            msg.odom.pose.pose.position.y,
            msg.odom.pose.pose.position.z,
        ])

        new_rot = np.array([
            msg.odom.pose.pose.orientation.x,
            msg.odom.pose.pose.orientation.y,
            msg.odom.pose.pose.orientation.z,
            msg.odom.pose.pose.orientation.w,
        ])

        self.tf_broadcaster.sendTransform(
            new_trans,
            new_rot,
            msg.odom.header.stamp,
            msg.odom.child_frame_id,
            "loam_init_ros_convention"
        )

    # def loam_odom_callback(self, msg):
    #     # Rotation from LOAM init frame in ROS convention, to LOAM init frame in LOAM convention
    #     loam_to_ros_translate = np.array([0.0, 0.0, 0.0])
    #     loam_to_ros_quat = tf.transformations.quaternion_from_matrix(
    #         tf.transformations.inverse_matrix(ros_to_loam)
    #     )
    #     self.tf_broadcaster.sendTransform(
    #         loam_to_ros_translate,
    #         loam_to_ros_quat,
    #         time=msg.header.stamp,
    #         parent=msg.child_frame_id,
    #         child=msg.child_frame_id + "_ros"
    #     )
    #     self.tf_listener.waitForTransform(
    #         self.loam_init_frame_id,
    #         "loam_init_ros_convention",
    #         msg.header.stamp,
    #         rospy.Duration.from_sec(0.1)
    #     )
    #     newpose = PoseStamped()
    #     newpose.header = msg.header
    #     newpose.header.frame_id = msg.header.frame_id
    #     newpose.pose = msg.pose.pose
    #     newpose = self.tf_listener.transformPose("loam_init_ros_convention", newpose)
    #     msg.pose.pose = newpose.pose
    #
    #
    #
    #     msg.header.frame_id = "loam_init_ros_convention"
    #     msg.child_frame_id = msg.child_frame_id + "_ros"
    #     self.pub_loam_odom.publish(msg)


if __name__ == "__main__":
    node = LoamFrameTransform()
    rospy.spin()
