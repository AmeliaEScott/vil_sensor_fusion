"""
Just a collection of functions which were tested for this project. No node here!
"""

from __future__ import print_function, division
import rospy
from vil_fusion.msg import DegeneracyScore
from nav_msgs.msg import Odometry
import numpy as np
import math

def _demo1(mat_now, mat_prev, pose_now, pose_prev, hessian_now, hessian_prev, msg_now, msg_prev):
    """
    An example of a degeneracy detection function. All functions in this package should match this exact signature.
    These functions do not need to bother setting the header of the message, as that will be handled elsewhere.
    Each function may use **kwargs to ignore the unneeded args.

    :param mat_now: The 6x6 OR 3x3 numpy matrix representing the covariance or hessian at the current time step
    :param mat_prev: The 6x6 OR 3x3 numpy matrix representing the covariance or hessian at the previous time step
    :param pose_now: The 6x1 OR 3x1 numpy matrix representing the pose (X, Y, Z, Roll, Pitch, Yaw) at the current time step
    :param pose_prev: The 6x1 OR 3x1 numpy matrix representing the pose (X, Y, Z, Roll, Pitch, Yaw) at the previous time step
    :return: A number representing the degeneracy score
    """
    return 0


def _covariance_to_correlation(mat_now):
    try:
        diag = np.diag(np.diag(mat_now))
        d = np.sqrt(diag)
        d_inv = np.linalg.inv(d)
        return d_inv * mat_now * d_inv
    except np.linalg.LinAlgError:
        return np.nan


def d_opt(mat_now, **_):
    """
    D-optimality
    """

    _, logdet = np.linalg.slogdet(mat_now)
    return math.exp(logdet / mat_now.shape[0])


def d_opt_ratio(mat_now, mat_prev, **_):
    try:
        ratio = np.matmul(mat_now, np.linalg.inv(mat_prev))
        _, logdet = np.linalg.slogdet(ratio)
        return math.exp(logdet / ratio.shape[0])
    except np.linalg.LinAlgError:
        return np.nan


def a_opt(mat_now, **_):
    """
    A-optimality
    """
    return np.trace(mat_now)


def a_opt_ratio(mat_now, mat_prev, **_):
    """
    A-optimality
    """
    try:
        ratio = np.matmul(mat_now, np.linalg.inv(mat_prev))
        return np.trace(ratio)
    except np.linalg.LinAlgError:
        return np.nan


def e_opt(mat_now, **_):
    """
    E-optimality
    """

    try:
        eigenvalues = np.linalg.eigvals(mat_now)
        return np.min(eigenvalues)
    except np.linalg.LinAlgError:
        return np.nan


def e_opt_ratio(mat_now, mat_prev, **_):
    """
    E-optimality
    """

    try:
        ratio = np.matmul(mat_now, np.linalg.inv(mat_prev))
        eigenvalues = np.linalg.eigvals(ratio)
        return np.min(eigenvalues)
    except np.linalg.LinAlgError:
        return np.nan


def max_eigen(mat_now, **_):
    """
    E-optimality
    """

    try:
        eigenvalues = np.linalg.eigvals(mat_now)
        return np.max(eigenvalues)
    except np.linalg.LinAlgError:
        return np.nan


def max_eigen_ratio(mat_now, mat_prev, **_):
    """
    E-optimality
    """
    try:
        ratio = np.matmul(mat_now, np.linalg.inv(mat_prev))
        eigenvalues = np.linalg.eigvals(ratio)
        return np.max(eigenvalues)
    except np.linalg.LinAlgError:
        return np.nan


def jensen_bregman(mat_now, mat_prev, **_):
    """
    Jensen-Bregman LogDet Divergence
    """
    _, logdet = np.linalg.slogdet((mat_now + mat_prev) / 2)

    return logdet - 0.5 * np.linalg.det(np.matmul(mat_now, mat_prev))


def correlation_matrix_distance(mat_now, mat_prev, **_):
    """
    Correlation matrix distance
    """
    try:
        mat_now = _covariance_to_correlation(mat_now)
        mat_prev = _covariance_to_correlation(mat_prev)
        trace = np.trace(np.matmul(mat_now, mat_prev))
        frob_x = np.linalg.norm(mat_now, ord='fro')
        frob_y = np.linalg.norm(mat_prev, ord='fro')

        return 1 - (trace / (frob_x * frob_y))
    except ValueError:
        return np.nan


def kullback_leibler(mat_now, mat_prev, pose_now, pose_prev, **_):
    """
    Kullback-Leibler Divergence
    """

    try:
        E1 = mat_prev
        E2 = mat_now
        E1i = np.linalg.inv(E1)
        E2i = np.linalg.inv(E2)
        u1 = pose_prev
        u2 = pose_now
        u_diff = u1 - u2
        I = np.identity(mat_now.shape[0])

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
        #print("det(E2): {}, det(E1): {}".format( np.linalg.det(E2), np.linalg.det(E1)))
        c = math.log(
            abs(np.linalg.det(E2)) / abs(np.linalg.det(E1))
        )

        score = 0.5 * (a + b + c)
        return score
    except np.linalg.LinAlgError:
        return np.nan


def jensen_bregman_0(mat_now, mat_prev, **_):
    return jensen_bregman(mat_now, np.zeros_like(mat_prev))


def kullback_leibler_0pose(mat_now, mat_prev, pose_now, pose_prev, **_):
    return kullback_leibler(mat_now, mat_prev, np.zeros_like(pose_now), np.zeros_like(pose_prev))


def kullback_leibler_0cov(mat_now, mat_prev, pose_now, pose_prev, **_):
    return kullback_leibler(mat_now, np.zeros_like(mat_prev), np.zeros_like(pose_now), np.zeros_like(pose_prev))


def differential_entropy(mat_now, **_):
    x = (2 * math.pi * math.e) ** mat_now.shape[0]
    return 0.5 * math.log(x * np.linalg.det(mat_now))


def norm_frobenius(mat_now, **_):
    return np.linalg.norm(mat_now, ord="fro")


def norm_nuclear(mat_now, **_):
    return np.linalg.norm(mat_now, ord="nuc")


def norm_1(mat_now, **_):
    return np.linalg.norm(mat_now, ord=1)


def norm_2(mat_now, **_):
    return np.linalg.norm(mat_now, ord=2)


def norm_frobenius_ratio(mat_now, mat_prev, **_):
    ratio = np.matmul(mat_now, np.linalg.inv(mat_prev))
    return np.linalg.norm(ratio, ord="fro")


def norm_nuclear_ratio(mat_now, mat_prev, **_):
    ratio = np.matmul(mat_now, np.linalg.inv(mat_prev))
    return np.linalg.norm(ratio, ord="nuc")


def norm_1_ratio(mat_now, mat_prev, **_):
    ratio = np.matmul(mat_now, np.linalg.inv(mat_prev))
    return np.linalg.norm(ratio, ord=1)


def norm_2_ratio(mat_now, mat_prev, **_):
    ratio = np.matmul(mat_now, np.linalg.inv(mat_prev))
    return np.linalg.norm(ratio, ord=2)


def condition_number(mat_now, **_):
    try:
        return -np.linalg.cond(mat_now)
    except np.linalg.LinAlgError:
        return np.nan


def condition_cov(mat_now, **_):
    try:
        return np.linalg.cond(mat_now)
    except np.linalg.LinAlgError:
        return np.nan


degen_funcs = [
    d_opt,
    d_opt_ratio,
    a_opt,
    a_opt_ratio,
    e_opt,
    e_opt_ratio,
    max_eigen,
    max_eigen_ratio,
    jensen_bregman,
    correlation_matrix_distance,
    kullback_leibler,
    norm_frobenius,
    norm_frobenius_ratio,
    norm_nuclear,
    norm_nuclear_ratio,
    norm_1,
    norm_1_ratio,
    norm_2,
    norm_2_ratio
]
