#!/usr/bin/env python3

"""
Gathers data from ROS bags containing odometry output, and plots degeneracy metrics vs. ground truth
error metrics over time. Also includes the ROC curve for degeneracy detection.
"""

from typing import Tuple, List, SupportsFloat, AnyStr, Dict
from warnings import warn

import rosbag
import os
import numpy as np
from scipy.spatial.transform import Rotation
try:
    from . import degeneracy_detection_functions as degen_funcs
except ImportError:
    import degeneracy_detection_functions as degen_funcs
import pickle
from matplotlib import pyplot as plt
from vil_fusion.msg import DiagnosticMessage
import rospy
import copy
import itertools


class OdomWithHessian:
    def __init__(self, odom):
        self.odom = odom
        self.hessian = np.zeros((6, 6), dtype=np.float64)


LOAM_DIAGNOSTIC_TOPIC = "/diagnostics/loam_odom"
ROVIO_DIAGNOSTIC_TOPIC = "/diagnostics/rovio"
FUSION_DIAGNOSTIC_TOPIC = "/diagnostics/fusion"

LOAM_ODOM_TOPIC = "/laser_odom_to_init_CORRECTED"
# LOAM_ODOM_TOPIC = "/laser_odom_to_init"
LOAM_STATUS_TOPIC = "/laser_odom_optimization_status"
ROVIO_ODOM_TOPIC = "/rovio/odometry"
FUSION_ODOM_TOPIC = "/gtsam_fusion_node/odometry"

BAG_DIR = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/sample_bags"


DEGEN_ROT = {
    # "Results_Tunnel.bag": [
    #     (25.0, 35.0)
    # ],
    # "Results_Plane2.bag": [
    #     (35.0, 78.0)
    # ],
    "Results_SanRafael.bag": [
        (35.0, 85.0)
    ],
    # "Results_Town.bag": [
    #     (39.0, 59.0)
    # ],
    # "Test1_vehicle.tesla.model3_results.bag": [
    #     (53.0, 75.0),
    #     # (115.0, 162.0),
    # ],
    # "Test2_Denser_vehicle.tesla.model3_results.bag": [
    #     (45.0, 77.0),
    #     (165.0, 180.0)
    # ],
    # "Test3_vehicle.tesla.model3_results.bag": [],
    # "Test3_vehicle.tesla.model3_results2.bag": [],
    # "Test4_vehicle.tesla.model3_results.bag": [],
    # "Test4_vehicle.audi.tt_hdl64e_results.bag": [],
    # "Town03_vehicle.tesla.model3_results.bag": [],
    # "san_04_handheld_results.bag": [],
    # "anymal_arche_results.bag": [],
    # "forest_shepherds_crook_results.bag": [],
    # "2011_09_26_drive_0022_sync_results.bag": [],
    # "2011_10_03_drive_0042_sync_results.bag": [],
    # "02_vlp16_results.bag": [],
    # "06_os64_results.bag": [],
}

DEGEN_TRANS = {
    "Results_Tunnel.bag": [
        (25.0, 35.0)
    ],
    "Results_Plane2.bag": [
        (35.0, 78.0)
    ],
    "Results_SanRafael.bag": [
        (35.0, 85.0)
    ],
    "Results_Town.bag": [
        (13.0, 64.0)
    ],
    # "Test1_vehicle.tesla.model3_results.bag": [
    #     (53.0, 75.0),
    #     # (115.0, 162.0),
    # ],
    # "Test2_Denser_vehicle.tesla.model3_results.bag": [
    #     # (45.0, 77.0),
    #     # (165.0, 180.0)
    # ],
    # "Test3_vehicle.tesla.model3_results.bag": [],
    # "Test3_vehicle.tesla.model3_results2.bag": [],
    # "Test4_vehicle.tesla.model3_results.bag": [
    #     (106.0, 125.0),
    # ],
    # "Test4_vehicle.audi.tt_hdl64e_results.bag": [
    #     (106.0, 125.0),
    # ],
    # "Town03_vehicle.tesla.model3_results.bag": [],
    # "san_04_handheld_results.bag": [
    #     (29.0, 83.0)
    # ],
    # "anymal_arche_results.bag": [(110.0, 400.0)],
    # "forest_shepherds_crook_results.bag": [],
    # "2011_09_26_drive_0022_sync_results.bag": [],
    # "2011_10_03_drive_0042_sync_results.bag": [],
    # "02_vlp16_results.bag": [],
    # "06_os64_results.bag": [],
}

PLOTS = [
    {
        'source': 'loam',
        'title': 'Translation',
        'include_bagname': False,
        'plots': [
            # {
            #     'diagnostic': True,
            #     'roc': False,
            #     'metric': 'abs_linear_vel_err',
            #     'log': False,
            #     'label': 'GT Vel Err',
            #     'matrix_subset': 'trans',
            #     # 'ylim': 0.5
            # },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'd_opt',
                'log': False,
                'label': 'D Opt',
                'matrix': 'hessian',
                'matrix_subset': 'trans'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'e_opt',
                'log': False,
                'label': 'E Opt',
                'matrix': 'hessian',
                'matrix_subset': 'trans'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'a_opt',
                'log': False,
                'label': 'A Opt',
                'matrix': 'hessian',
                'matrix_subset': 'trans'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'norm_frobenius',
                'log': False,
                'label': 'F Norm',
                'matrix': 'hessian',
                'matrix_subset': 'trans'
            },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'd_opt',
            #     'log': False,
            #     'label': 'D Opt',
            #     'matrix': 'hessian',
            #     'matrix_subset': 'trans'
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'e_opt',
            #     'log': False,
            #     'label': 'E Opt',
            #     'matrix': 'hessian',
            #     'matrix_subset': 'trans'
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'a_opt',
            #     'log': False,
            #     'label': 'A Opt',
            #     'matrix': 'hessian',
            #     'matrix_subset': 'trans'
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'norm_frobenius',
            #     'log': False,
            #     'label': 'F Norm',
            #     'matrix': 'hessian',
            #     'matrix_subset': 'trans'
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'dist_slope_tx',
            #     'log': False,
            #     'label': 'X',
            #     'matrix': 'dists_corner',
            #     'matrix_subset': 'trans',
            #     # 'ylim': 0.02
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'dist_slope_tx',
            #     'log': False,
            #     'label': 'Y',
            #     'matrix': 'dists_corner',
            #     'matrix_subset': 'trans',
            #     # 'ylim': 0.02
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'dist_slope_tx',
            #     'log': False,
            #     'label': 'Z',
            #     'matrix': 'dists_corner',
            #     'matrix_subset': 'trans',
            #     # 'ylim': 0.02
            # },
        ]
    },
    {
        'source': 'loam',
        'title': 'Rotation',
        'include_bagname': False,
        'plots': [
            # {
            #     'diagnostic': True,
            #     'roc': False,
            #     'metric': 'abs_rot_vel_err',
            #     'log': False,
            #     'label': 'GT Ang Err',
            #     'matrix_subset': 'rot',
            # },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'd_opt',
                'log': False,
                'label': 'D Opt',
                'matrix': 'hessian',
                'matrix_subset': 'rot'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'e_opt',
                'log': False,
                'label': 'E Opt',
                'matrix': 'hessian',
                'matrix_subset': 'rot'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'a_opt',
                'log': False,
                'label': 'A Opt',
                'matrix': 'hessian',
                'matrix_subset': 'rot'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'norm_frobenius',
                'log': False,
                'label': 'F Norm',
                'matrix': 'hessian',
                'matrix_subset': 'rot'
            },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'd_opt',
            #     'log': False,
            #     'label': 'D Opt',
            #     'matrix': 'hessian',
            #     'matrix_subset': 'rot'
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'e_opt',
            #     'log': False,
            #     'label': 'E Opt',
            #     'matrix': 'hessian',
            #     'matrix_subset': 'rot'
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'a_opt',
            #     'log': False,
            #     'label': 'A Opt',
            #     'matrix': 'hessian',
            #     'matrix_subset': 'rot'
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'norm_frobenius',
            #     'log': False,
            #     'label': 'F Norm',
            #     'matrix': 'hessian',
            #     'matrix_subset': 'rot'
            # },

            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'dist_slope_rx',
            #     'log': False,
            #     'label': 'RX',
            #     'matrix': 'dists_corner',
            #     'matrix_subset': 'rot',
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'dist_slope_ry',
            #     'log': False,
            #     'label': 'RY',
            #     'matrix': 'dists_corner',
            #     'matrix_subset': 'rot',
            # },
            # {
            #     'diagnostic': False,
            #     'roc': True,
            #     'metric': 'dist_slope_rz',
            #     'log': False,
            #     'label': 'RZ',
            #     'matrix': 'dists_corner',
            #     'matrix_subset': 'rot',
            # },
        ]
    },
    {
        'source': 'loam',
        'title': 'Combined',
        'include_bagname': False,
        'plots': [
            # {
            #     'diagnostic': True,
            #     'roc': False,
            #     'metric': 'abs_linear_vel_err',
            #     'log': False,
            #     'label': 'GT Vel Err',
            #     'matrix_subset': 'all',
            #     # 'ylim': 0.5
            # },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'd_opt',
                'log': False,
                'label': 'D Opt',
                'matrix': 'hessian',
                'matrix_subset': 'all'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'e_opt',
                'log': False,
                'label': 'E Opt',
                'matrix': 'hessian',
                'matrix_subset': 'all'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'a_opt',
                'log': False,
                'label': 'A Opt',
                'matrix': 'hessian',
                'matrix_subset': 'all'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'norm_frobenius',
                'log': False,
                'label': 'F Norm',
                'matrix': 'hessian',
                'matrix_subset': 'all'
            },
        ]
    },
]


def numpify_diagnostics(data, hessian=False):
    times = np.zeros(shape=(len(data)), dtype=np.float64)
    diagnostics = {
        'relative_dist_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'abs_rot_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'abs_dist_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'rel_linear_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'abs_linear_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'rel_rot_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'abs_rot_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'gt_distance': np.zeros(shape=(len(data)), dtype=np.float64),
    }
    odometry = {
        'covariance': np.zeros(shape=(6, 6, len(data)), dtype=np.float64),
        'pose': np.zeros(shape=(6, 1, len(data)), dtype=np.float64),
    }
    if hessian:
        odometry['hessian'] = np.zeros(shape=(6, 6, len(data)), dtype=np.float64)
        odometry['dists'] = np.zeros(shape=(6, 15, len(data)), dtype=np.float64)
        odometry['dists_surface'] = np.zeros_like(odometry['dists'])
        odometry['dists_corner'] = np.zeros_like(odometry['dists'])
        odometry['shifts_trans'] = np.zeros(shape=(15, len(data)))
        odometry['shifts_rot'] = np.zeros_like(odometry['shifts_trans'])
    odometry['cov_twist'] = np.zeros(shape=(6, 6, len(data)), dtype=np.float64)
    for i, (odom, diag) in enumerate(data):
        for field in diagnostics.keys():
            diagnostics[field][i] = getattr(diag, field)
        times[i] = diag.header.stamp.to_sec()
        if hessian:
            odometry['hessian'][:, :, i] = np.reshape(np.array(odom.hessian, dtype=np.float64), (6, 6))
            if hasattr(odom, "dists"):
                odometry['dists'][:, :, i] = np.reshape(np.array(odom.dists, dtype=np.float64), (6, -1))
                odometry['dists_surface'][:, :, i] = np.reshape(np.array(odom.dists_surface, dtype=np.float64), (6, -1))
                odometry['dists_corner'][:, :, i] = np.reshape(np.array(odom.dists_corner, dtype=np.float64), (6, -1))
                odometry['shifts_trans'][:, i] = np.array(odom.shift_trans, dtype=np.float64)
                odometry['shifts_rot'][:, i] = np.array(odom.shift_rot, dtype=np.float64)
            else:
                print("Could not find 'dists' in OdometryWithHessian")
            covariance = np.array(odom.odom.pose.covariance, dtype=np.float64)
            covariance_twist = np.array(odom.odom.twist.covariance, dtype=np.float64)
            position = odom.odom.pose.pose.position
            orientation = odom.odom.pose.pose.orientation
        else:
            covariance = np.array(odom.pose.covariance, dtype=np.float64)
            covariance_twist = np.array(odom.twist.covariance, dtype=np.float64)
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
        odometry['cov_twist'][:, :, i] = np.reshape(np.array(covariance_twist, dtype=np.float64), (6, 6))
        odometry['pose'][:, :, i] = np.reshape(pose, (6, 1))
    return times, diagnostics, odometry


def load_ros_bag(bag_abs_path):
    bag_name = os.path.basename(bag_abs_path)
    loam_odoms = {}
    rovio_odoms = {}
    fusion_odoms = {}

    loam_data = []
    rovio_data = []
    fusion_data = []

    with rosbag.Bag(bag_abs_path) as bag:
        print("Reading odometry data for {}...".format(bag_name))
        for topic, msg, _ in bag.read_messages(topics=[LOAM_ODOM_TOPIC, ROVIO_ODOM_TOPIC, FUSION_ODOM_TOPIC]):
            if topic == LOAM_ODOM_TOPIC:
                if hasattr(msg, "odom"):
                    t = msg.odom.header.stamp
                    loam_odoms[t] = msg
                else:
                    t = msg.header.stamp
                    loam_odoms[t] = OdomWithHessian(msg)
            elif topic == FUSION_ODOM_TOPIC:
                t = msg.header.stamp
                fusion_odoms[t] = msg
            else:
                t = msg.header.stamp
                rovio_odoms[t] = msg

        for topic, msg, _ in bag.read_messages(topics=[LOAM_STATUS_TOPIC]):
            t = msg.header.stamp
            if t in loam_odoms:
                loam_odoms[t].hessian = msg.hessian

        print("Read {} loam, {} rovio, {} fusion".format(len(loam_odoms), len(rovio_odoms), len(fusion_odoms)))
        print("Reading diagnostic data...")
        for topic, msg, _ in bag.read_messages(topics=[LOAM_DIAGNOSTIC_TOPIC, ROVIO_DIAGNOSTIC_TOPIC, FUSION_DIAGNOSTIC_TOPIC]):
            t = msg.header.stamp
            if topic == LOAM_DIAGNOSTIC_TOPIC and t in loam_odoms:
                loam_data.append((loam_odoms[t], msg))
            elif topic == ROVIO_DIAGNOSTIC_TOPIC and t in rovio_odoms:
                rovio_data.append((rovio_odoms[t], msg))
            elif topic == FUSION_DIAGNOSTIC_TOPIC and t in fusion_odoms:
                fusion_data.append((fusion_odoms[t], msg))

        if len(rovio_data) == 0:
            print("No rovio diagnostics, adding fake diagnostics.")
            for time, odom in sorted(rovio_odoms.items(), key=lambda x: x[0]):
                diag = DiagnosticMessage()
                diag.header.stamp = time
                rovio_data.append((odom, diag))
        if len(loam_data) == 0:
            print("No loam diagnostics, adding fake diagnostics.")
            for time, odom in sorted(loam_odoms.items(), key=lambda x: x[0]):
                diag = DiagnosticMessage()
                diag.header.stamp = time
                loam_data.append((odom, diag))

        print("Total count so far: {} loam, {} rovio".format(len(loam_data), len(rovio_data)))

    return loam_data, rovio_data, fusion_data


def shade_degen_regions(axes: plt.Axes, yrange: Tuple, regions: List[Tuple]):

    for region in regions:
        x = np.array(region, dtype=np.float64)
        y1 = np.array([yrange[0], yrange[0]], dtype=np.float64)
        y2 = np.array([yrange[1], yrange[1]], dtype=np.float64)
        axes.fill_between(x, y1, y2, color=(0.2, 0.2, 0.2, 0.2))


def apply_degen_function(matrix, pose, matrix_subset, func, shifts_trans=None, shifts_rot=None):
    if matrix.shape[0:2] == (6, 6):
        if matrix_subset == "rot":
            matrix = matrix[3:6, 3:6, :]
            pose = pose[3:6, :, :]
        elif matrix_subset == "trans":
            matrix = matrix[0:3, 0:3, :]
            pose = pose[0:3, :, :]
        elif matrix_subset in ["x", "y", "z", "roll", "pitch", "yaw"]:
            i = ["x", "y", "z", "roll", "pitch", "yaw"].index(matrix_subset)
            matrix = matrix[i:i + 1, i:i + 1, :]
            pose = pose[i:i + 1, :, :]
        elif matrix_subset != "all":
            raise RuntimeWarning("Invalid matrix subset {}".format(matrix_subset))

    y = np.zeros(matrix.shape[2], dtype=matrix.dtype)
    for i in range(1, matrix.shape[2]):
        args = {
            'mat_now': matrix[:, :, i],
            'mat_prev': matrix[:, :, i - 1],
            'pose_now': pose[:, :, i],
            'pose_prev': pose[:, :, i - 1]
        }
        if shifts_trans is not None:
            args['shifts_trans'] = shifts_trans[:, i]
            args['shifts_rot'] = shifts_rot[:, i]

        y[i] = func(**args)

    return y


def calc_roc(is_degen: np.ndarray, score: np.ndarray):
    percentiles = np.linspace(0, 100, 100, dtype=np.float64)
    thresholds = np.percentile(score, percentiles)
    is_degen_estimate = np.expand_dims(score, 0) <= np.expand_dims(thresholds, 1)
    true_positives = np.logical_and(is_degen_estimate, is_degen)
    false_positives = np.logical_and(is_degen_estimate, np.logical_not(is_degen))
    tpr = np.sum(true_positives, axis=1) / np.sum(is_degen)
    fpr = np.sum(false_positives, axis=1) / np.sum(np.logical_not(is_degen))

    return tpr, fpr


def plot_roc(axes: plt.Axes, is_degen, times, metric, invert=False):
    # ROC
    is_degen = is_degen[1:]

    tpr, fpr = calc_roc(is_degen, metric)
    if invert:
        tpr = 1.0 - tpr
        fpr = 1.0 - fpr
    axes.plot(fpr, tpr)
    axes.set_ylabel("TPR")
    axes.set_xlabel("FPR")


def plot_single_axes(
        axes: plt.Axes,
        source: str,
        data,
        degen_regions,
        plot_metadata
):
    print(plot_metadata)
    log = plot_metadata['log']
    metric = plot_metadata['metric']
    is_diagnostic = plot_metadata['diagnostic']
    label = plot_metadata['label']
    times = data[source + '_times']
    diagnostics = data[source + '_diagnostics']
    odometry = data[source + '_odometry']

    if is_diagnostic:
        y = diagnostics[metric]
        y = y[1:]
    else:
        matrix_subset = plot_metadata['matrix_subset']
        matrix_name = plot_metadata['matrix']
        matrix = odometry[matrix_name]
        pose = odometry['pose']

        degen_func = getattr(degen_funcs, metric)

        if 'shifts_rot' in odometry:
            y = apply_degen_function(matrix, pose, matrix_subset, degen_func,
                                     shifts_rot=odometry['shifts_rot'], shifts_trans=odometry['shifts_trans'])
        else:
            y = apply_degen_function(matrix, pose, matrix_subset, degen_func)

        if "derive" in plot_metadata and plot_metadata["derive"]:
            y = y[1:] - y[:-1]
        else:
            y = y[1:]

    axes.plot(times[1:], y)
    shade_degen_regions(axes, (np.min(y), np.max(y)), degen_regions)
    if 'ymin' in plot_metadata:
        axes.set_ylim(bottom=plot_metadata['ymin'])
    else:
        axes.set_ylim(bottom=0)
    if 'ylim' in plot_metadata:
        axes.set_ylim(top=plot_metadata['ylim'])
    if log:
        axes.set_yscale("log")
    axes.set_ylabel(label)
    return y


def plot(
        data,
        plots: Dict,
        title: AnyStr
):
    """
    Plot one figure with multiple axes

    :param times: Numpy array of times
    :param diagnostics: Dictionary of diagnostics
    :param odometry: Dictionary of odometry
    :param degen_regions: List of tuples giving degenerate regions to highlight
    :param plots: dict from PLOTS
    :param title: Title of entire figure
    :return:
    """
    fig: plt.Figure
    axeses: List[plt.Axes]
    fig, axeses = plt.subplots(nrows=len(plots['plots']), ncols=2, sharex='col', gridspec_kw={
        'width_ratios': [0.8, 0.2]
    })

    roc_ax: plt.Axes
    for plot_metadata, (ax, roc_ax) in zip(plots['plots'], axeses):
        do_roc = plot_metadata['roc']

        if plot_metadata['matrix_subset'] in ['rot', 'roll', 'pitch', 'yaw']:
            is_degen = data['is_degen_rot']
            degen_regions = data['degen_regions_rot']
        else:
            is_degen = data['is_degen_trans']
            degen_regions = data['degen_regions_trans']

        y = plot_single_axes(ax, plots['source'], data, degen_regions, plot_metadata)

        if do_roc:
            matrix_name = plot_metadata['matrix']
            plot_roc(roc_ax, is_degen, data[plots['source'] + '_times'], y, invert=matrix_name.startswith("cov"))
        else:
            roc_ax.axis('off')

    ax.set_xlabel("Time (s)")
    if plots['include_bagname']:
        suptitle = "{} - {}".format(plots['title'], title)
    else:
        suptitle = plots['title']
    fig.suptitle(suptitle, y=0.997)
    fig.show()


def plot_all_over_time(data, plots):
    for bag_name in data:
        for fig_data in PLOTS:
            source = fig_data['source']
            title = bag_name.split("_")[0]

            if source == 'loam':
                plot(
                    data[bag_name],
                    fig_data,
                    title
                )
            else:
                warn("Be careful, Rovio is not fully implemented")
                plot(
                    data[bag_name],
                    fig_data,
                    title
                )


def load_all_bags(bag_dir: str, degen_regions_rot: Dict, degen_regions_trans: Dict):
    """
    Load all data from either bags or pkl cache
    :return: Dict of bag_name -> {loam_times: ndarray, loam_odometry: ndarray, loam_diagnostics: ndarray}
    """
    data = {}

    for bag_name, degen_regions in degen_regions_rot.items():
        bag_abs_path = os.path.join(bag_dir, bag_name)
        pkl_abs_path = os.path.join(bag_dir, bag_name + ".pkl")

        if not os.path.isfile(pkl_abs_path):

            loam_data, rovio_data, fusion_data = load_ros_bag(bag_abs_path)
            loam_times, loam_diagnostics, loam_odometry = numpify_diagnostics(loam_data, hessian=True)
            rovio_times, rovio_diagnostics, rovio_odometry = numpify_diagnostics(rovio_data, hessian=False)
            fusion_times, fusion_diagnostics, fusion_odometry = numpify_diagnostics(fusion_data, hessian=False)

            loam_times -= loam_times[0]

            # Checkpoint save
            tmp_data = {
                'loam_times': loam_times,
                'loam_diagnostics': loam_diagnostics,
                'loam_odometry': loam_odometry,
                'rovio_times': rovio_times,
                'rovio_diagnostics': rovio_diagnostics,
                'rovio_odometry': rovio_odometry,
                'fusion_times': fusion_times,
                'fusion_diagnostics': fusion_diagnostics,
                'fusion_odometry': fusion_odometry,
            }

            print("Dumping data...")
            with open(pkl_abs_path, "wb") as fp:
                pickle.dump(tmp_data, fp)
        else:
            with open(pkl_abs_path, "rb") as fp:
                tmp_data = pickle.load(fp)

        tmp_data['degen_regions_rot'] = degen_regions_rot[bag_name]
        tmp_data['degen_regions_trans'] = degen_regions_trans[bag_name]
        tmp_data['degen_regions_all'] = degen_regions_rot[bag_name] + degen_regions_trans[bag_name]

        for label, regions in zip(['is_degen_rot', 'is_degen_trans', 'is_degen_all'], [
            degen_regions_rot[bag_name], degen_regions_trans[bag_name], degen_regions_rot[bag_name] + degen_regions_trans[bag_name]
        ]):
            is_degen = np.zeros_like(tmp_data['loam_times'], dtype=np.bool)
            for degen_region in regions:
                is_degen = np.logical_or(is_degen, np.logical_and(
                    tmp_data['loam_times'] > degen_region[0],
                    tmp_data['loam_times'] < degen_region[1]
                ))
            tmp_data[label] = is_degen

        data[bag_name] = tmp_data

    return data


def plot_all_rocs(data):
    all_degen_rot = np.array([], dtype=np.bool)
    all_degen_trans = np.array([], dtype=np.bool)
    all_degen_all = np.array([], dtype=np.bool)

    trans_scores = [
        {
            'func': degen_funcs.d_opt,
            'matrix_name': 'hessian',
            'label': "D-Opt Hess.",
            'inv': False
        },
        {
            'func': degen_funcs.a_opt,
            'matrix_name': 'hessian',
            'label': "A-Opt Hess.",
            'inv': False
        },
        {
            'func': degen_funcs.e_opt,
            'matrix_name': 'hessian',
            'label': "E-Opt Hess.",
            'inv': False
        },
        {
            'func': degen_funcs.max_eigen,
            'matrix_name': 'hessian',
            'label': "Max Eig. Hess.",
            'inv': False
        },
        {
            'func': degen_funcs.norm_1,
            'matrix_name': 'hessian',
            'label': "1 Norm Hess.",
            'inv': False
        },
        {
            'func': degen_funcs.norm_2,
            'matrix_name': 'hessian',
            'label': "2 Norm Hess.",
            'inv': False
        },
        {
            'func': degen_funcs.norm_frobenius,
            'matrix_name': 'hessian',
            'label': "F Norm Hess.",
            'inv': False
        },
        {
            'func': degen_funcs.condition_number,
            'matrix_name': 'hessian',
            'label': 'Cond Hess.',
            'inv': False
        },

        ###################################

        {
            'func': degen_funcs.d_opt,
            'matrix_name': 'cov_twist',
            'label': "D-Opt Cov.",
            'inv': True
        },
        {
            'func': degen_funcs.a_opt,
            'matrix_name': 'cov_twist',
            'label': "A-Opt Cov.",
            'inv': True
        },
        {
            'func': degen_funcs.e_opt,
            'matrix_name': 'cov_twist',
            'label': "E-Opt Cov.",
            'inv': True
        },
        {
            'func': degen_funcs.max_eigen,
            'matrix_name': 'cov_twist',
            'label': "Max Eigen Cov.",
            'inv': True
        },
        {
            'func': degen_funcs.norm_1,
            'matrix_name': 'cov_twist',
            'label': "1 Norm Cov.",
            'inv': True
        },
        {
            'func': degen_funcs.norm_2,
            'matrix_name': 'cov_twist',
            'label': "2 Norm Cov.",
            'inv': True
        },
        {
            'func': degen_funcs.norm_frobenius,
            'matrix_name': 'cov_twist',
            'label': "F Norm Cov.",
            'inv': True
        },
        {
            'func': degen_funcs.condition_number,
            'matrix_name': 'cov_twist',
            'label': 'Cond Cov.',
            'inv': False
        },


        ############################################


        {
            'func': degen_funcs.jensen_bregman,
            'matrix_name': 'cov_twist',
            'label': "JB Divergence",
            'inv': True
        },
        {
            'func': degen_funcs.differential_entropy,
            'matrix_name': 'cov_twist',
            'label': 'Diff. Ent.',
            'inv': True
        },
        {
            'func': degen_funcs.kullback_leibler_0pose,
            'matrix_name': 'cov_twist',
            'label': 'KL Divergence',
            'inv': True
        },

    ]

    for d in trans_scores:
        if d is not None:
            d['scores'] = np.array([], dtype=np.float64)

    rot_scores = copy.deepcopy(trans_scores)
    all_scores = copy.deepcopy(trans_scores)

    # trans_scores[6]['inv'] = False

    for bagname, bagdata in data.items():
        all_degen_rot = np.concatenate([all_degen_rot, bagdata['is_degen_rot']])
        all_degen_trans = np.concatenate([all_degen_trans, bagdata['is_degen_trans']])
        all_degen_all = np.concatenate([all_degen_all, bagdata['is_degen_all']])

        for x, subset in zip([rot_scores, trans_scores, all_scores], ["rot", "trans", "all"]):
            for d in x:
                if d is not None:
                    y = apply_degen_function(
                        data[bagname]['loam_odometry'][d['matrix_name']],
                        data[bagname]['loam_odometry']['pose'],
                        subset,
                        d['func'],
                        shifts_rot=data[bagname]['loam_odometry']['shifts_rot'],
                        shifts_trans=data[bagname]['loam_odometry']['shifts_trans']
                    )
                    d['scores'] = np.concatenate([d['scores'], y])
                    # print("+ {} = {}".format(y.shape, d['scores'].shape))

    nrows = 5
    ncols = 4
    gridspec_kw = {
        'left': 0.1, 'right': 0.95, 'top': 0.9, 'bottom': 0.1,
        'hspace': 0.4, 'wspace': 0.25
    }

    fig_rot: plt.Figure
    axeses_rot: List[plt.Axes]
    fig_rot, axeses_rot = plt.subplots(nrows=nrows, ncols=ncols, gridspec_kw=gridspec_kw)

    fig_trans: plt.Figure
    axeses_trans: List[plt.Axes]
    fig_trans, axeses_trans = plt.subplots(nrows=nrows, ncols=ncols, gridspec_kw=gridspec_kw)

    fig_all: plt.Figure
    axeses_all: List[plt.Axes]
    fig_all, axeses_all = plt.subplots(nrows=nrows, ncols=ncols, gridspec_kw=gridspec_kw)

    fig: plt.Figure
    for x, subset, axeses, is_degen, label, fig in zip(
            [rot_scores, trans_scores, all_scores],
            ["rot", "trans", "all"],
            [axeses_rot, axeses_trans, axeses_all],
            [all_degen_rot, all_degen_trans, all_degen_all],
            ["Rotation", "Translation", "Combined"],
            [fig_rot, fig_trans, fig_all]
    ):
        fig.suptitle(label, y=0.996, size='large')
        axes: plt.Axes
        for d, axes in zip(itertools.chain(x, itertools.repeat(None)), axeses.flatten()):
            if d is None:
                axes.axis('off')
                continue
            tpr, fpr = calc_roc(is_degen, d['scores'])
            tpr[-1] = 1.0
            fpr[-1] = 1.0
            tpr[0] = 0.0
            fpr[0] = 0.0
            if d['inv']:
                fpr = 1.0 - fpr
                tpr = 1.0 - tpr
            axes.plot(fpr, tpr)
            auc = abs(np.trapz(tpr, fpr))
            axes.set_ylim(0, 1.05)
            axes.set_xlim(-0.05, 1)
            axes.set_xticks([])
            axes.set_yticks([])
            axes.set_title(d['label'], size="medium")
            axes.text(0.95, 0.05, s="{:.3f}".format(auc), ha='right', va='bottom')
        for axes in axeses[-1, :]:
            axes.set_xticks([0.0, 1.0])
        for axes in axeses[:, 0]:
            axes.set_yticks([0.0, 1.0])
        fig.text(x=0.5, y=0.02, s="False Positive Rate",
                 horizontalalignment='center', verticalalignment='center', size='large')
        fig.text(x=0.03, y=0.5, s="True Positive Rate",
                 horizontalalignment='center', verticalalignment='center', size='large',
                 rotation='vertical')
        fig.show()
        fig.savefig(label + ".svg", format="svg", transparent=True)
        fig.savefig(label + ".pdf", format="pdf", transparent=True)
        fig.savefig(label + ".png", format="png", transparent=True, dpi=400 )


def compare_datasets(data):

    title_map = {
        "Results_SanRafael.bag": "Real-life underpass",
        "Results_Town.bag": "Town",
        "Results_Tunnel.bag": "Tunnel",
        "Results_Plane.bag": "Plane",
        "Results_Plane2.bag": "Plane",
    }

    for plot in PLOTS:
        fig: plt.Figure
        axeses: List[List[plt.Axes]]
        fig, axeses = plt.subplots(nrows=len(plot['plots']), ncols=len(data), sharex='col', sharey='row')

        for i, axes_row, plot_metadata in zip(range(0, 1000), axeses, plot['plots']):
            if type(axes_row) != np.ndarray:
                axes_row = [axes_row]
            axes: plt.Axes
            for j, axes, (bagname, bagdata) in zip(range(0, 1000), itertools.cycle(axes_row), data.items()):
                if i == 0:
                    axes.set_title(title_map[bagname])
                if plot_metadata['matrix_subset'] == 'rot':
                    degen_regions = bagdata['degen_regions_rot']
                else:
                    degen_regions = bagdata['degen_regions_trans']
                plot_single_axes(axes, plot['source'], bagdata, degen_regions, plot_metadata)
                if j != 0:
                    axes.set_ylabel(None)
                if i == axeses.shape[0] - 1:
                    axes.set_xlabel("Time (s)")
                axes.set_ylim(auto=True)
        fig.suptitle(plot['title'], y=0.997)
        fig.show()


def compare_datasets_on_same_plot(data):

    title_map = {
        "Results_SanRafael.bag": "Real-life underpass",
        "Results_Town.bag": "Simulated Town"
    }

    for plot in PLOTS:
        fig: plt.Figure
        axeses: List[List[plt.Axes]]
        # fig, axeses = plt.subplots(nrows=len(plot['plots']), ncols=len(data), sharex='col', sharey='row')
        fig, axeses = plt.subplots(nrows=len(plot['plots']), ncols=1, sharex='col', sharey='row')

        for i, axes_row, plot_metadata in zip(range(0, 1000), axeses, plot['plots']):
            if type(axes_row) != np.ndarray:
                axes_row = [axes_row]
            axes: plt.Axes
            for j, axes, (bagname, bagdata) in zip(range(0, 1000), itertools.cycle(axes_row), data.items()):
                # if i == 0:
                #     axes.set_title(title_map[bagname])
                if plot_metadata['matrix_subset'] == 'rot':
                    degen_regions = bagdata['degen_regions_rot']
                else:
                    degen_regions = bagdata['degen_regions_trans']
                plot_single_axes(axes, plot['source'], bagdata, degen_regions, plot_metadata)
                # if j != 0:
                #     axes.set_ylabel(None)
                if i == axeses.shape[0] - 1:
                    axes.set_xlabel("Time (s)")
                axes.set_ylim(auto=True)
                axes.set_yscale("log")
                axes.legend(["Real Underpass", "Simulated Town"], loc='lower right')
        fig.suptitle(plot['title'], y=0.997)
        fig.show()


def plot_err_over_time(data):

    ylabel = 'Global Pose Error (m)'
    # ylabel = 'Linear Velocity Error (m/s)'
    metric = 'abs_dist_err'
    # metric = 'abs_linear_vel_err'
    plots = [
        {
            'source': 'loam',
            'label': 'LOAM',
            'color': '#1010FF',
            'smooth': 15,
            'start': 50,
            'end': -1,
        },
        {
            'source': 'rovio',
            'label': 'ROVIO',
            'color': '#D0B000',
            'offset': 50,
            'smooth': 15,
            'start': 0,
            'end': -1,
        },
        {
            'source': 'fusion',
            'label': 'Fusion',
            'color': '#FF0000',
            'smooth': 15,
            'start': 0,
            'end': -1,
        },
    ]

    title_map = {
        'Results_Tunnel.bag': "Tunnel",
        'Results_Town.bag': "Town",
        'Results_Plane.bag': "Plane",
        'Results_Plane2.bag': "Plane",
    }

    for bag_name in data:
        fig: plt.Figure = plt.figure()
        for plot in plots:
            print(list(data[bag_name][plot['source'] + '_diagnostics'].keys()))

            x = data[bag_name][plot['source'] + '_times']
            y = data[bag_name][plot['source'] + '_diagnostics'][metric]

            x = x[plot['start']:plot['end']]
            y = y[plot['start']:plot['end']]

            if plot['smooth'] > 1:
                filter = np.ones([plot['smooth']], dtype=np.float32) / plot['smooth']
                time_filter = np.zeros_like(filter)
                time_filter[-1] = 1.0
                x = np.convolve(x, time_filter, mode="valid")
                y = np.convolve(y, filter, mode="valid")

            plt.plot(x, y, label=plot['label'], color=plot['color'])
        plt.legend(loc='upper right')
        plt.title(title_map[bag_name])
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)
        shade_degen_regions(fig.get_axes()[0], yrange=(0, 105), regions=DEGEN_TRANS[bag_name])
        # plt.ylim(top=0.3, bottom=-0.05)
        plt.show()


def comp_log(data):
    print(list(data.keys()))
    for bagname in data:
        pass


if __name__ == "__main__":

    data = load_all_bags(BAG_DIR, DEGEN_ROT, DEGEN_TRANS)
    plot_all_over_time(data, PLOTS)
    # plot_all_rocs(data)
    # compare_datasets(data)
    # plot_err_over_time(data)
    # comp_log(data)
