from __future__ import print_function, division
import rospy
from vil_fusion.msg import DegeneracyScore
from nav_msgs.msg import Odometry
import numpy as np
import math


def demo1(cov_now, cov_prev, pose_now, pose_prev, hessian_now, hessian_prev, msg_now, msg_prev):
    """
    An example of a degeneracy detection function. All functions in this package should match this exact signature.
    These functions do not need to bother setting the header of the message, as that will be handled elsewhere.
    Each function may use **kwargs to ignore the unneeded args.

    :param cov_now: The 6x6 OR 3x3 numpy matrix representing the covariance at the current time step
    :param cov_prev: The 6x6 OR 3x3 numpy matrix representing the covariance at the previous time step
    :param pose_now: The 6x1 OR 3x1 numpy matrix representing the pose (X, Y, Z, Roll, Pitch, Yaw) at the current time step
    :param pose_prev: The 6x1 OR 3x1 numpy matrix representing the pose (X, Y, Z, Roll, Pitch, Yaw) at the previous time step
    :param hessian_now: The 6x6 OR 3x3 numpy matrix representing the Hessian for the LOAM ICP optimization. None for Rovio
    :param hessian_prev: The 6x6 OR 3x3 numpy matrix representing the Hessian for the LOAM ICP optimization. None for Rovio
    :param msg_now: The current nav_msgs/Odometry
    :param msg_prev: The previous nav_msgs/Odometry
    :return: A number representing the degeneracy score
    """
    return 0


def covariance_to_correlation(cov_now):
    d = np.sqrt(np.diag(cov_now))
    d_inv = np.linalg.inv(d)
    return d_inv * cov_now * d_inv


def d_opt(cov_now, **kwargs):
    """
    D-optimality
    """

    _, logdet = np.linalg.slogdet(cov_now)
    return math.exp(logdet / cov_now.shape[0])


def a_opt(cov_now, **kwargs):
    """
    A-optimality
    """
    return np.trace(cov_now)


def e_opt(cov_now, **kwargs):
    """
    E-optimality
    """

    try:
        eigenvalues = np.linalg.eigvals(cov_now)
    except np.linalg.LinAlgError:
        print("Error with {}".format(cov_now))
        eigenvalues = np.array([0, 0, 0, 0, 0, 0])
    return np.min(eigenvalues)


def jensen_bregman(cov_now, cov_prev, **kwargs):
    """
    Jensen-Bregman LogDet Divergence
    """
    _, logdet = np.linalg.slogdet((cov_now + cov_prev) / 2)

    return logdet - 0.5 * np.linalg.det(np.matmul(cov_now, cov_prev))


def correlation_matrix_distance(cov_now, cov_prev, **kwargs):
    """
    Correlation matrix distance
    """

    trace = np.trace(np.matmul(cov_now, cov_prev))
    frob_x = np.linalg.norm(cov_now, ord='fro')
    frob_y = np.linalg.norm(cov_prev, ord='fro')

    return 1 - (trace / (frob_x * frob_y))


def kullback_leibler(cov_now, cov_prev, pose_now, pose_prev, **kwargs):
    """
    Kullback-Leibler Divergence
    """

    E1 = cov_prev
    E2 = cov_now
    E1i = np.linalg.inv(E1)
    E2i = np.linalg.inv(E2)
    u1 = pose_prev
    u2 = pose_now
    u_diff = u1 - u2
    I = np.identity(cov_now.shape[0])

    a = np.trace(
        np.matmul(E2i, E1) - I
    )
    b = np.matmul(
        np.transpose(u1 - u2),
        np.matmul(
            E2i,
            u1 - u2
        )
    )
    c = math.log(
        np.linalg.det(E2) / np.linalg.det(E1)
    )

    score = 0.5 * (a + b + c)
    return score


def norm_frobenius(cov_now, **kwargs):
    return np.linalg.norm(cov_now, ord="fro")


def norm_nuclear(cov_now, **kwargs):
    return np.linalg.norm(cov_now, ord="nuc")


def norm_1(cov_now, **kwargs):
    return np.linalg.norm(cov_now, ord=1)


def norm_2(cov_now, **kwargs):
    return np.linalg.norm(cov_now, ord=2)


def norm_frobenius_diff(cov_now, cov_prev, **kwargs):
    return np.linalg.norm(cov_now - cov_prev, ord="fro")


def norm_nuclear_diff(cov_now, cov_prev, **kwargs):
    return np.linalg.norm(cov_now - cov_prev, ord="nuc")


def norm_1_diff(cov_now, cov_prev, **kwargs):
    return np.linalg.norm(cov_now - cov_prev, ord=1)


def norm_2_diff(cov_now, cov_prev, **kwargs):
    return np.linalg.norm(cov_now - cov_prev, ord=2)