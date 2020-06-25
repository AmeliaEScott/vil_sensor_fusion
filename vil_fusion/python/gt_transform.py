#!/usr/bin/env python2

from __future__ import print_function, division

import rospy
import numpy as np
from tf import TransformBroadcaster, transformations
from geometry_msgs.msg import TransformStamped
from copy import deepcopy


class GTTransformer:

    def __init__(self, node_name="gt_transform"):
        rospy.init_node(node_name)

        self.vicon_extrinsics = rospy.get_param("~vicon_extrinsics")
        self.vicon_extrinsics = np.reshape(np.array(self.vicon_extrinsics, dtype=np.float64), (4, 4))

        self.vicon_extr_quat = transformations.quaternion_from_matrix(self.vicon_extrinsics)
        self.vicon_extr_trans = transformations.translation_from_matrix(self.vicon_extrinsics)

        self.broadcaster = TransformBroadcaster()
        self.parent_frame_id = rospy.get_param("~parent_frame", "/map")
        self.child_frame_id = rospy.get_param("~child_frame", "/ego_vehicle/imu/imu_vio")
        self.tf_listener = rospy.Subscriber("~transform", TransformStamped, callback=self.callback)

        self.init_transform = None

    def callback(self, msg):
        """
        :type msg: TransformStamped
        """
        if self.init_transform is None:
            self.init_transform = deepcopy(msg)
            self.init_transform.header.frame_id = self.parent_frame_id
            self.init_transform.child_frame_id = "vicon_init"
            # self.init_transform.child_frame_id = "rovio_world"
            # self.init_transform.transform.rotation.w = 1
            # self.init_transform.transform.rotation.x = 0
            # self.init_transform.transform.rotation.y = 0
            # self.init_transform.transform.rotation.z = 0
        self.init_transform.header.stamp = msg.header.stamp
        self.broadcaster.sendTransformMessage(self.init_transform)

        msg.header.frame_id = self.parent_frame_id
        msg.child_frame_id = "vicon"
        # msg.child_frame_id = self.child_frame_id
        self.broadcaster.sendTransformMessage(msg)

        # I do not know why this is needed. But without it, the ground truth and rovio estimate are
        # not aligned.
        trans = np.array([0.0, 0.0, 0.0])
        # 180 degrees about z axis
        rot = np.array([0.0, 0.0, 1.0, 0.0])
        self.broadcaster.sendTransform(
            trans,
            rot,
            msg.header.stamp,
            "rovio_world",
            "vicon_init"
        )

        self.broadcaster.sendTransform(
            self.vicon_extr_trans,
            self.vicon_extr_quat,
            msg.header.stamp,
            self.child_frame_id,
            "vicon"
        )


if __name__ == "__main__":
    node = GTTransformer()
    rospy.spin()