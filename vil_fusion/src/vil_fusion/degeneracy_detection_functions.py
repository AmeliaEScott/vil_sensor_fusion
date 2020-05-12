#!/usr/bin/env python3

import rospy
from vil_fusion.msg import DegeneracyScore
from nav_msgs.msg import Odometry


def demo1(matrix, msg) -> DegeneracyScore:
    """
    An example of a degeneracy detection function. All functions in this package should match this exact signature.
    These functions do not need to bother setting the header of the message, as that will be handled elsewhere.

    :param matrix: The 6x6 numpy matrix representing either the Hessian (for LOAM) or the Covariance (for Rovio)
    :param msg: The nav_msgs.Odometry message from LOAM or Rovio
    :return: A DegeneracyScore message with the results of the calculation. The header does not need to be set.
    """
    result = DegeneracyScore()
    result.degenerate = False
    result.threshold = 0
    result.score = 0
    return result


def demo2(matrix, msg) -> DegeneracyScore:
    """
    An example of a degeneracy detection function. All functions in this package should match this exact signature.
    These functions do not need to bother setting the header of the message, as that will be handled elsewhere.

    :param matrix: The 6x6 numpy matrix representing either the Hessian (for LOAM) or the Covariance (for Rovio)
    :param msg: The nav_msgs.Odometry message from LOAM or Rovio
    :return: A DegeneracyScore message with the results of the calculation. The header does not need to be set.
    """
    result = DegeneracyScore()
    result.degenerate = False
    result.threshold = 0
    result.score = 0
    return result