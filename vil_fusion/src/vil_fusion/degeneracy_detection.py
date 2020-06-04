#!/usr/bin/env python2

from __future__ import print_function, division
import numpy as np
import rospy
from vil_fusion import degeneracy_detection_functions
from vil_fusion.msg import DegeneracyScore
from nav_msgs.msg import Odometry
from loam_velodyne.msg import OdometryWithHessian
from sensor_msgs.msg import Imu
import tf.transformations


class DegenDetectionNode:

    def __init__(self, name="degeneracy_detection_node"):
        rospy.init_node(name)

        self.rovio_pub = rospy.Publisher("~rovio_output", Odometry, queue_size=1)
        self.loam_pub = rospy.Publisher("~loam_output", Odometry, queue_size=1)

        self.rovio_sub = rospy.Subscriber("~rovio_input", Odometry, self.rovio_callback)
        self.loam_sub = rospy.Subscriber("~loam_input", OdometryWithHessian, self.loam_callback)

        loam_func_names = rospy.get_param("~loam_degen_funcs", default=[])
        rovio_func_names = rospy.get_param("~rovio_degen_funcs", default=[])

        self.loam_degen_funcs = {}
        self.rovio_degen_funcs = {}

        self.rovio_state = {
            'cov_prev': np.identity(6, dtype=np.float32),
            'pose_prev': np.zeros([6, 1], dtype=np.float32),
            'hessian_prev': None,
            'msg_prev': Odometry()
        }

        self.loam_state = {
            'cov_prev': np.identity(6, dtype=np.float32),
            'pose_prev': np.zeros([6, 1], dtype=np.float32),
            'hessian_prev': np.identity(6, dtype=np.float32),
            'msg_prev': Odometry()
        }

        for source, names, funcs in zip(
                ["loam", "rovio"],
                [loam_func_names, rovio_func_names],
                [self.loam_degen_funcs, self.rovio_degen_funcs],
        ):
            for name in names:
                funcs[name] = {
                    'func': degeneracy_detection_functions.__dict__[name],
                    'pub': rospy.Publisher("~{}/{}".format(source, name), DegeneracyScore, queue_size=1),
                    # 'd_pub': rospy.Publisher("~{}/d_{}".format(source, name), DegeneracyScore, queue_size=1),
                    'prev_score_all': 0,
                    'prev_score_trans': 0,
                    'prev_score_rot': 0,
                }

    def rovio_callback(self, msg):
        self.shared_callback(msg, None, self.rovio_pub, self.rovio_degen_funcs, self.rovio_state)

    def loam_callback(self, msg):
        self.shared_callback(msg.odom, np.array(msg.hessian), self.loam_pub, self.loam_degen_funcs, self.loam_state)

    def shared_callback(self, msg, hessian, odom_pub, funcs, state):
        is_degenerate = False
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        pose = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [msg.pose.pose.position.z],
            [roll],
            [pitch],
            [yaw],
        ])
        args_all = {
            'cov_now': np.array(msg.pose.covariance).reshape([6, 6]),
            'cov_prev': state['cov_prev'],
            'pose_now': pose,
            'pose_prev': state['pose_prev'],
            'hessian_now': hessian,
            'hessian_prev': state['hessian_prev'],
            'msg_now': msg,
            'msg_prev': state['msg_prev']
        }

        args_trans = {
            'cov_now': args_all['cov_now'][0:3, 0:3],
            'cov_prev': args_all['cov_prev'][0:3, 0:3],
            'pose_now': args_all['pose_now'][0:3, 0],
            'pose_prev': args_all['pose_prev'][0:3, 0],
            'hessian_now': args_all['hessian_now'][0:3, 0:3],
            'hessian_prev': args_all['hessian_prev'][0:3, 0:3],
            'msg_now': args_all['msg_now'],
            'msg_prev': args_all['msg_prev'],
        }

        args_rot = {
            'cov_now': args_all['cov_now'][3:6, 3:6],
            'cov_prev': args_all['cov_prev'][3:6, 3:6],
            'pose_now': args_all['pose_now'][3:6, 0],
            'pose_prev': args_all['pose_prev'][3:6, 0],
            'hessian_now': args_all['hessian_now'][3:6, 3:6],
            'hessian_prev': args_all['hessian_prev'][3:6, 3:6],
            'msg_now': args_all['msg_now'],
            'msg_prev': args_all['msg_prev'],
        }

        for data in funcs.values():
            func = data['func']
            pub = data['pub']
            degen_msg = DegeneracyScore()
            degen_msg.score_all = func(**args_all)
            degen_msg.score_trans = func(**args_trans)
            degen_msg.score_rot = func(**args_rot)
            degen_msg.header = msg.header
            degen_msg.name = func.__name__
            degen_msg.derivative_all = degen_msg.score_all - data['prev_score_all']
            degen_msg.derivative_rot = degen_msg.score_rot - data['prev_score_rot']
            degen_msg.derivative_trans = degen_msg.score_trans - data['prev_score_trans']
            pub.publish(degen_msg)
            data['prev_score_all'] = degen_msg.score_all
            data['prev_score_trans'] = degen_msg.score_trans
            data['prev_score_rot'] = degen_msg.score_rot

        # ConFusion does some weird things with coordinate transforms. It seems to assume that the pose measurements
        # it receives are not already published to TF. However, both LOAM and Rovio do publish to TF, so some transforms
        # get published multiple times.
        # So here, I just create some new coordinate frames for ConFusion to work with.
        # I do it here because all of my pose measurements will end up here anyway, and it's too simple of a task
        # to justify creating a new node.
        msg.header.frame_id += "_for_confusion"
        msg.child_frame_id += "_for_confusion"

        if not is_degenerate:
            odom_pub.publish(msg)

        state['cov_prev'] = args_all['cov_now']
        state['pose_prev'] = args_all['pose_now']
        state['hessian_prev'] = args_all['hessian_now']
        state['msg_prev'] = args_all['msg_now']


if __name__ == "__main__":
    node = DegenDetectionNode()
    rospy.spin()
