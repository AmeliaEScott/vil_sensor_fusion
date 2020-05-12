#!/usr/bin/env python3

import numpy as np
import rospy
from vil_fusion import degeneracy_detection_functions
from vil_fusion.msg import DegeneracyScore
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class DegenDetectionNode:

    def __init__(self, name="degeneracy_detection_node"):
        rospy.init_node(name)

        self.rovio_pub = rospy.Publisher("~rovio_output", Odometry, queue_size=1)
        self.loam_pub = rospy.Publisher("~loam_output", Odometry, queue_size=1)

        self.rovio_sub = rospy.Subscriber("~rovio_input", Odometry, self.rovio_callback)
        self.loam_sub = rospy.Subscriber("~loam_input", Odometry, self.loam_callback)

        loam_func_names = rospy.get_param("~loam_degen_funcs", default=[])
        rovio_func_names = rospy.get_param("~rovio_degen_funcs", default=[])

        self.loam_degen_funcs = {}
        self.rovio_degen_funcs = {}

        for source, names, funcs in zip(
                ["loam", "rovio"],
                [loam_func_names, rovio_func_names],
                [self.loam_degen_funcs, self.rovio_degen_funcs],
        ):
            for name in names:
                funcs[name] = (
                                degeneracy_detection_functions.__dict__[name],
                                rospy.Publisher("~{}/{}".format(source, name), DegeneracyScore, queue_size=1)
                )

    def rovio_callback(self, msg):
        self.shared_callback(msg, self.rovio_pub, self.rovio_degen_funcs)

    def loam_callback(self, msg):
        self.shared_callback(msg, self.loam_pub, self.loam_degen_funcs)

    def shared_callback(self, msg: Odometry, odom_pub: rospy.Publisher, funcs):
        matrix = np.array(msg.pose.covariance).reshape([6, 6])
        is_degenerate = False
        for func, pub in funcs.values():
            degen_msg = func(matrix, msg)
            degen_msg.header = msg.header
            degen_msg.name = func.__name__
            is_degenerate = is_degenerate or degen_msg.degenerate
            pub.publish(degen_msg)

        if not is_degenerate:
            odom_pub.publish(msg)


if __name__ == "__main__":
    node = DegenDetectionNode()
    rospy.spin()
