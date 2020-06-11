#!/usr/bin/env python3
from typing import Tuple, List

import rosbag
from loam_velodyne.msg import OdometryWithHessian
from nav_msgs.msg import Odometry
from vil_fusion.msg import DiagnosticMessage
import os
import numpy as np
from scipy.spatial.transform import Rotation
try:
    from . import degeneracy_detection_functions as degen_funcs
except ImportError:
    import degeneracy_detection_functions as degen_funcs
import pickle
from matplotlib import pyplot as plt

LOAM_DIAGNOSTIC_TOPIC = "/diagnostics/loam_odom"
ROVIO_DIAGNOSTIC_TOPIC = "/diagnostics/rovio"

LOAM_ODOM_TOPIC = "/loam/frame_transform/odometry/ros"
ROVIO_ODOM_TOPIC = "/rovio/odometry"

# BAG_DIR = "/media/timothy/1ABED71A5F421E8D/TimothyScott/rawdata/experiment_results"
# BAG_NAME = "Test1_vehicle.tesla.model3_results.bag"
BAG_DIR = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/carla_tools/rosbags"

DEGEN_REGIONS = {
    "Test4_vehicle.tesla.model3_results.bag": [
        (139.0, 157.0),
    ]
}

PLOTS = [
    ('loam', "d_opt", "hessian_trans", "log(rel_linear_vel_err)"),
    ('loam', "e_opt", "hessian_trans", "log(rel_linear_vel_err)"),
    ('loam', "log(kullback_leibler)", "covariance_trans", "log(rel_linear_vel_err)"),
    ('loam', "jensen_bregman", "covariance_trans", "log(rel_linear_vel_err)"),
]


def numpify_diagnostics(data, hessian=False):
    times = np.zeros(shape=(len(data)), dtype=np.float64)
    diagnostics = {
        'relative_dist_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'abs_rot_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'rel_linear_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'rel_rot_vel_err': np.zeros(shape=(len(data)), dtype=np.float64)
    }
    odometry = {
        'covariance': np.zeros(shape=(6, 6, len(data)), dtype=np.float64),
        'pose': np.zeros(shape=(6, 1, len(data)), dtype=np.float64),
    }
    if hessian:
        odometry['hessian'] = np.zeros(shape=(6, 6, len(data)), dtype=np.float64)
    for i, (odom, diag) in enumerate(data):
        for field in diagnostics.keys():
            diagnostics[field][i] = getattr(diag, field)
        times[i] = diag.header.stamp.to_sec()
        if hessian:
            odometry['hessian'][:, :, i] = np.reshape(np.array(odom.hessian, dtype=np.float64), (6, 6))
            covariance = np.array(odom.odom.pose.covariance, dtype=np.float64)
            position = odom.odom.pose.pose.position
            orientation = odom.odom.pose.pose.orientation
        else:
            covariance = np.array(odom.pose.covariance, dtype=np.float64)
            position = odom.pose.pose.position
            orientation = odom.pose.pose.orientation
        orientation = np.array([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ], dtype=np.float64)
        orientation = Rotation.from_quat(orientation)
        position = np.array([
            position.x,
            position.y,
            position.z,
        ], dtype=np.float64)
        pose = np.concatenate([position, orientation.as_euler("XYZ")])
        odometry['covariance'][:, :, i] = np.reshape(covariance, (6, 6))
        odometry['pose'][:, :, i] = np.reshape(pose, (6, 1))
    return times, diagnostics, odometry


def load_ros_bag(bag_abs_path):
    bag_name = os.path.basename(bag_abs_path)
    loam_odoms = {}
    rovio_odoms = {}

    loam_data = []
    rovio_data = []

    with rosbag.Bag(bag_abs_path) as bag:
        print("Reading odometry data for {}...".format(bag_name))
        for topic, msg, _ in bag.read_messages(topics=[LOAM_ODOM_TOPIC, ROVIO_ODOM_TOPIC]):
            if topic == LOAM_ODOM_TOPIC:
                t = msg.odom.header.stamp
                loam_odoms[t] = msg
            else:
                t = msg.header.stamp
                rovio_odoms[t] = msg

        print("Read {} loam, {} rovio".format(len(loam_odoms), len(rovio_odoms)))
        print("Reading diagnostic data...")
        for topic, msg, _ in bag.read_messages(topics=[LOAM_DIAGNOSTIC_TOPIC, ROVIO_DIAGNOSTIC_TOPIC]):
            t = msg.header.stamp
            if topic == LOAM_DIAGNOSTIC_TOPIC and t in loam_odoms:
                loam_data.append((loam_odoms[t], msg))
            elif topic == ROVIO_DIAGNOSTIC_TOPIC and t in rovio_odoms:
                rovio_data.append((rovio_odoms[t], msg))

        print("Total count so far: {} loam, {} rovio".format(len(loam_data), len(rovio_data)))

    return loam_data, rovio_data


def shade_degen_regions(axes: plt.Axes, yrange: Tuple, regions: List[Tuple]):

    for region in regions:
        x = np.array(region, dtype=np.float64)
        y1 = np.array([yrange[0], yrange[0]], dtype=np.float64)
        y2 = np.array([yrange[1], yrange[1]], dtype=np.float64)
        axes.fill_between(x, y1, y2, color=(0.2, 0.2, 0.2, 0.2))


for bag_name, degen_regions in DEGEN_REGIONS.items():
    # bag_abs_path = os.path.join(BAG_DIR, bag_name)
    # loam_data, rovio_data = load_ros_bag(bag_abs_path)
    # loam_times, loam_diagnostics, loam_odometry = numpify_diagnostics(loam_data, hessian=True)
    # rovio_times, rovio_diagnostics, rovio_odometry = numpify_diagnostics(rovio_data, hessian=False)

    # Checkpoint save
    # tmp_data = {
    #     'loam_times': loam_times,
    #     'loam_diagnostics': loam_diagnostics,
    #     'loam_odometry': loam_odometry,
    #     'rovio_times': rovio_times,
    #     'rovio_diagnostics': rovio_diagnostics,
    #     'rovio_odometry': rovio_odometry
    # }
    #
    # print("Dumping data...")
    # with open(os.path.join(BAG_DIR, bag_name + ".pkl"), "wb") as fp:
    #     pickle.dump(tmp_data, fp)

    with open(os.path.join(BAG_DIR, bag_name + ".pkl"), "rb") as fp:
        tmp_data = pickle.load(fp)

    loam_times = tmp_data['loam_times']
    loam_diagnostics = tmp_data['loam_diagnostics']
    loam_odometry = tmp_data['loam_odometry']
    rovio_times = tmp_data['rovio_times']
    rovio_diagnostics = tmp_data['rovio_diagnostics']
    rovio_odometry = tmp_data['rovio_odometry']

    for source, degen_func_name, matrix_name, diag_name in PLOTS:
        if degen_func_name.startswith("log("):
            log_metric = True
            degen_func_name = degen_func_name[4:-1]
        else:
            log_metric = False
        degen_func = getattr(degen_funcs, degen_func_name)
        if source == 'loam':
            times = loam_times
            odometry = loam_odometry
            diagnostics = loam_diagnostics
        else:
            times = rovio_times
            odometry = rovio_odometry
            diagnostics = rovio_diagnostics
        if diag_name.startswith("log("):
            diag_name = diag_name[4:-1]
            log = True
            diag_plot = diagnostics[diag_name]
            diag_regress = np.log(diagnostics[diag_name])
        else:
            log = False
            diag_plot = diagnostics[diag_name]
            diag_regress = diagnostics[diag_name]

        if matrix_name.startswith("hessian"):
            matrix = odometry['hessian']
        else:
            matrix = odometry['covariance']

        if matrix_name == "hessian_rot" or matrix_name == "covariance_trans":
            matrix = matrix[0:3, 0:3, :]
        elif matrix_name == "hessian_trans" or matrix_name == "covariance_rot":
            matrix = matrix[3:6, 3:6, :]

        pose = odometry['pose']
        if matrix_name.endswith("_rot"):
            pose = pose[3:6, :, :]
        elif matrix_name.endswith("_trans"):
            pose = pose[0:3, :, :]

        metric = np.zeros_like(diag_plot)
        for i in range(1, matrix.shape[2]):

            args = {
                'mat_now': matrix[:, :, i],
                'mat_prev': matrix[:, :, i-1],
                'pose_now': pose[:, :, i],
                'pose_prev': pose[:, :, i-1]
            }

            metric[i] = degen_func(**args)

        ax1: plt.Axes
        ax2: plt.Axes
        fig: plt.Figure
        fig, (ax1, ax2) = plt.subplots(nrows=2, sharex="all")

        times = times[1:]
        diag_plot = diag_plot[1:]
        metric = metric[1:]

        ax1.plot(times, diag_plot)
        shade_degen_regions(ax1, (np.min(diag_plot), np.max(diag_plot)), degen_regions)
        if log:
            ax1.set_yscale("log")
        ax1.set_ylabel(diag_name)

        ax2.plot(times, metric)
        if log_metric:
            ax2.set_yscale("log")
        shade_degen_regions(ax2, (np.min(metric), np.max(metric)), degen_regions)
        ax2.set_ylabel("{}_{}".format(degen_func.__name__, matrix_name))
        ax2.set_xlabel("Time (s)")

        fig.show()

    # y = loam_diagnostics['rel_linear_vel_err']
    #
    # for region in degen_regions:
    #     x = np.array(region, dtype=np.float64)
    #     y1 = np.array([np.max(y), np.max(y)], dtype=np.float64)
    #     y2 = np.array([np.min(y), np.min(y)], dtype=np.float64)
    #     plt.fill_between(x, y1, y2, color=(0.2, 0.2, 0.2, 0.2))
    #
    # plt.plot(loam_times, y, "-")
    # plt.show()
