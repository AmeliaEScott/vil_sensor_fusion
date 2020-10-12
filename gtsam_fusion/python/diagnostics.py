#!/usr/bin/env python2

"""
Compares the transforms from ground truth data to odometry estimates, and published DiagnosticMessage topics.

See vil_fusion/cfg/*/diagnostics_params.yaml for example configurations.
"""

from __future__ import print_function, division
import rospy
from tf import TransformListener, transformations
import threading
import numpy as np
from gtsam_fusion.msg import DiagnosticMessage
import math
from geometry_msgs.msg import Vector3Stamped


class DiagnosticNode:

    def __init__(self, node_name="gtsam_fusion_diagnostics"):
        rospy.init_node(node_name)
        self.params = rospy.get_param("~diagnostics")
        self.tf_listener = TransformListener()

        self.thread_handles = []

        for param in self.params:
            th = threading.Thread(target=self.transform_loop, kwargs=param)
            th.start()
            self.thread_handles.append(th)

    def transform_loop(self, name, gt, est, ref, rate):
        """

        :param name: Name of this diagnostic. Used only for display purposes
        :type name: str
        :param gt: str Ground truth frame_id
        :type gt: str
        :param est: Frame ID of estimated transform from odometry algorithm
        :type est: str
        :param ref: Frame ID of stationary reference frame
        :type ref: str
        :param rate: Approximate rate of the estimated transform
        :type rate: float
        :return:
        """

        pub = rospy.Publisher("~{}".format(name), DiagnosticMessage, queue_size=1)

        # Need to wait for the first time the transform is available
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(
                    target_frame=est,
                    source_frame=gt,
                    time=rospy.Time(),
                    timeout=rospy.Duration.from_sec(10)
                )
                break
            except:
                rospy.logdebug("Waiting for transforms to be available...")
                rospy.sleep(rospy.Duration.from_sec(0.2))

        last_time = self.tf_listener.getLatestCommonTime(gt, est)
        start_time = last_time

        total_distance = 0

        while not rospy.is_shutdown():
            # This is where the loop sleeps.
            self.tf_listener.waitForTransform(
                target_frame=est,
                source_frame=gt,
                # The time we wait for must be slightly before the actual time we expect, to account for floating
                # point accuracy issues
                time=last_time + rospy.Duration.from_sec((1 / rate) * 0.9),
                timeout=rospy.Duration.from_sec(1)
            )

            now = self.tf_listener.getLatestCommonTime(est, gt)

            # Ground truth transform between last time-step and now
            gt_d_trans, gt_d_rot = self.tf_listener.lookupTransformFull(
                target_frame=gt,
                target_time=last_time,
                source_frame=gt,
                source_time=now,
                fixed_frame=ref
            )
            gt_d_trans = np.array(gt_d_trans)

            # Estimated transform between last time-step and now
            est_d_trans, est_d_rot = self.tf_listener.lookupTransformFull(
                target_frame=est,
                target_time=last_time,
                source_frame=est,
                source_time=now,
                fixed_frame=ref
            )
            est_d_trans = np.array(est_d_trans)

            total_distance += np.linalg.norm(gt_d_trans)

            lin_vel_diff = est_d_trans - gt_d_trans
            ang_vel_diff = transformations.quaternion_multiply(gt_d_rot, transformations.quaternion_inverse(est_d_rot))

            trans_err, rot_err = self.tf_listener.lookupTransform(
                target_frame=gt,
                source_frame=est,
                time=now
            )

            rot_ang = 2 * math.acos(abs(rot_err[3]))



            msg = DiagnosticMessage()
            msg.header.stamp = now
            msg.name = name
            msg.gt_distance = total_distance
            msg.abs_dist_err = np.linalg.norm(trans_err)
            msg.abs_rot_err = rot_ang
            msg.relative_dist_err = float('Inf') if msg.gt_distance == 0 else msg.abs_dist_err / msg.gt_distance

            msg.abs_linear_vel_err = np.linalg.norm(lin_vel_diff)
            msg.abs_rot_vel_err = 2 * math.acos(abs(ang_vel_diff[3]))
            msg.rel_linear_vel_err = float('Inf') if np.linalg.norm(gt_d_trans) == 0 else msg.abs_linear_vel_err / np.linalg.norm(gt_d_trans)
            msg.rel_rot_vel_err = float('Inf') if math.acos(abs(gt_d_rot[3])) == 0 else msg.abs_rot_vel_err / (2 * math.acos(abs(gt_d_rot[3])))

            msg.err.orientation.x = rot_err[0]
            msg.err.orientation.y = rot_err[1]
            msg.err.orientation.z = rot_err[2]
            msg.err.orientation.w = rot_err[3]
            msg.err.position.x = trans_err[0]
            msg.err.position.y = trans_err[1]
            msg.err.position.z = trans_err[2]

            pub.publish(msg)

            last_time = now


if __name__ == "__main__":
    node = DiagnosticNode()
    rospy.spin()
