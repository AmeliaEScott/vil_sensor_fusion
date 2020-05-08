#!/usr/bin/env python2

import tf.transformations
from tf import Transformer, TransformBroadcaster, LookupException, TransformListener
import numpy as np
import rospy
from nav_msgs.msg import Odometry
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

        self.pub_loam_odom = rospy.Publisher("~odometry/ros", Odometry, queue_size=1)
        self.sub_loam_odom = rospy.Subscriber("~odometry/loam", Odometry, callback=self.loam_odom_callback)

    def loam_odom_callback(self, msg):
        # Rotation from LOAM init frame in ROS convention, to LOAM init frame in LOAM convention
        loam_to_ros_translate = np.array([0.0, 0.0, 0.0])
        loam_to_ros_quat = tf.transformations.quaternion_from_matrix(
            tf.transformations.inverse_matrix(ros_to_loam)
        )
        self.tf_broadcaster.sendTransform(
            loam_to_ros_translate,
            loam_to_ros_quat,
            time=msg.header.stamp,
            parent=msg.child_frame_id,
            child=msg.child_frame_id + "_ros"
        )
        self.tf_listener.waitForTransform(
            self.loam_init_frame_id,
            "loam_init_ros_convention",
            msg.header.stamp,
            rospy.Duration.from_sec(0.1)
        )
        newpose = PoseStamped()
        newpose.header = msg.header
        newpose.header.frame_id = msg.header.frame_id
        newpose.pose = msg.pose.pose
        newpose = self.tf_listener.transformPose("loam_init_ros_convention", newpose)
        msg.header.frame_id = "loam_init_ros_convention"
        msg.child_frame_id = msg.child_frame_id + "_ros"
        msg.pose.pose = newpose.pose
        self.pub_loam_odom.publish(msg)


if __name__ == "__main__":
    node = LoamFrameTransform()
    rospy.spin()
