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

LOAM_DIAGNOSTIC_TOPIC = "/diagnostics/loam_odommmmm"  # TODO: Remove this typo to get diagnostics back
ROVIO_DIAGNOSTIC_TOPIC = "/diagnostics/rovio"

LOAM_ODOM_TOPIC = "/loam/frame_transform/odometry/ros"
ROVIO_ODOM_TOPIC = "/rovio/odometry"

BAG_DIR = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/sample_bags"

# DEGEN_REGIONS = {
#     # "Test1_vehicle.audi.tt_results.bag": [  # LOAM failed in this run
#     #     (505.0, 535.0),
#     #     (625.0, 665.0),
#     #     (745.0, 783.0),
#     # ],
#     "Test1_vehicle.tesla.model3_results.bag": [
#         (58.0, 85.0),
#         (125.0, 175.0),
#
#     ],
#     "Test2_Denser_vehicle.tesla.model3_results.bag": [
#         (65.0, 100.0),
#         (185.0, 195.0)
#     ],
#     "Test3_vehicle.tesla.model3_results.bag": [
#     ],
#     "Test4_vehicle.audi.tt_results.bag": [
#         (607.0, 625.0),
#     ],
#     # "Town03_vehicle.audi.tt_results.bag": [
#     #     (170.0, 217.0)
#     # ],
#     # "V1_01_easy_results.bag": [
#     #
#     # ],
#     # "V1_03_difficult_results.bag": [
#     #
#     # ],
#     # "san_rafeal_tunnel_2019-06-24-14-17-04_0_results.bag": [
#     #     (1561411065.0, 1561411103.0)
#     # ]
# }

DEGEN_ROT = {
    # "Test1_vehicle.tesla.model3_results.bag": [
    #     (53.0, 70.0),
    #     (115.0, 162.0),
    # ],
    # "Test2_Denser_vehicle.tesla.model3_results.bag": [
    #     (45.0, 77.0),
    #     (165.0, 180.0)
    # ],
    # "Test3_vehicle.tesla.model3_results.bag": [
    # ],
    # "Test4_vehicle.audi.tt_results.bag": [
    # ],
    "Town03_vehicle.tesla.model3_results.bag": [],
    # "stopnstart_results.bag": [
    #
    # ],
    # "san_04_handheld_results.bag": [],
    "2011_09_26_drive_0022_sync_results.bag": [],
    "2011_10_03_drive_0042_sync_results.bag": [],
}

DEGEN_TRANS = {
    # "Test1_vehicle.tesla.model3_results.bag": [
    #     (53.0, 70.0),
    #     (115.0, 162.0),
    # ],
    # "Test2_Denser_vehicle.tesla.model3_results.bag": [
    #     # (45.0, 77.0),
    #     # (165.0, 180.0)
    # ],
    # "Test3_vehicle.tesla.model3_results.bag": [
    # ],
    # "Test4_vehicle.audi.tt_results.bag": [
    #     (106.0, 125.0),
    # ],
    "Town03_vehicle.tesla.model3_results.bag": [],
    # "stopnstart_results.bag": [
    #
    # ],
    # "san_04_handheld_results.bag": [
    #     (29.0, 83.0)
    # ],
    "2011_09_26_drive_0022_sync_results.bag": [],
    "2011_10_03_drive_0042_sync_results.bag": [],

}

PLOTS = [
    {
        'source': 'loam',
        'title': 'Loam Translation',
        'plots': [
            # {
            #     'diagnostic': True,
            #     'roc': False,
            #     'metric': 'rel_linear_vel_err',
            #     'log': False,
            #     'label': 'GT Vel. Error',
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
                'label': 'F. Norm',
                'matrix': 'hessian',
                'matrix_subset': 'trans'
            },
        ]
    },
    {
        'source': 'loam',
        'title': 'Loam Rotation',
        'plots': [
            # {
            #     'diagnostic': True,
            #     'roc': False,
            #     'metric': 'abs_rot_vel_err',
            #     'log': False,
            #     'label': 'GT Ang. Vel. Err',
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
                'label': 'F. Norm',
                'matrix': 'hessian',
                'matrix_subset': 'rot'
            },
        ]
    },
    # {
    #     'source': 'rovio',
    #     'title': 'Pose Covariance - Rotation',
    #     'plots': [
    #         {
    #             'diagnostic': True,
    #             'roc': False,
    #             'metric': 'abs_rot_vel_err',
    #             'log': False,
    #             'label': 'GT Rot. Err',
    #             'ylim': 0.05
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'norm_2_ratio',
    #             'log': False,
    #             'label': '2 Norm Ratio',
    #             'matrix': 'covariance',
    #             'matrix_subset': 'rot',
    #             'derive': False,
    #             # 'ymin': 0.000153,
    #             # 'ylim': 500,
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'norm_frobenius_ratio',
    #             'log': False,
    #             'label': 'Frobenius Ratio',
    #             'matrix': 'covariance',
    #             'matrix_subset': 'rot',
    #             'derive': False,
    #             # 'ymin': 0.000150,
    #             # 'ylim': 500,
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'max_eigen',
    #             'log': False,
    #             'label': 'Max Eigen',
    #             'matrix': 'covariance',
    #             'matrix_subset': 'rot',
    #             'derive': False,
    #         },
    #     ]
    # },
    # {
    #     'source': 'rovio',
    #     'title': 'Rovio Covariance - Translation',
    #     'plots': [
    #         {
    #             'diagnostic': True,
    #             'roc': False,
    #             'metric': 'abs_linear_vel_err',
    #             'log': False,
    #             'label': 'GT Vel. Error',
    #             'ylim': 0.03
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'd_opt',
    #             'log': False,
    #             'label': 'D Opt(twist)',
    #             'matrix': 'cov_twist',
    #             'matrix_subset': 'trans',
    #             'derive': False,
    #             # 'ylim': 0.1,
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'e_opt',
    #             'log': False,
    #             'label': 'E opt',
    #             'matrix': 'covariance',
    #             'matrix_subset': 'trans',
    #             'derive': False,
    #             # 'ylim': 0.1,
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'max_eigen',
    #             'log': False,
    #             'label': 'Max eigen',
    #             'matrix': 'covariance',
    #             'matrix_subset': 'trans',
    #             'derive': False,
    #
    #         },
    #     ]
    # },
]


def numpify_diagnostics(data, hessian=False):
    times = np.zeros(shape=(len(data)), dtype=np.float64)
    diagnostics = {
        'relative_dist_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'abs_rot_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'rel_linear_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'abs_linear_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'rel_rot_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
        'abs_rot_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
    }
    odometry = {
        'covariance': np.zeros(shape=(6, 6, len(data)), dtype=np.float64),
        'pose': np.zeros(shape=(6, 1, len(data)), dtype=np.float64),
    }
    if hessian:
        odometry['hessian'] = np.zeros(shape=(6, 6, len(data)), dtype=np.float64)
    else:
        odometry['cov_twist'] = np.zeros(shape=(6, 6, len(data)), dtype=np.float64)
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
            odometry['cov_twist'][:, :, i] = np.reshape(np.array(odom.twist.covariance, dtype=np.float64), (6, 6))
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

    return loam_data, rovio_data


def shade_degen_regions(axes: plt.Axes, yrange: Tuple, regions: List[Tuple]):

    for region in regions:
        x = np.array(region, dtype=np.float64)
        y1 = np.array([yrange[0], yrange[0]], dtype=np.float64)
        y2 = np.array([yrange[1], yrange[1]], dtype=np.float64)
        axes.fill_between(x, y1, y2, color=(0.2, 0.2, 0.2, 0.2))


def apply_degen_function(matrix, pose, matrix_subset, func):
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
    fig.suptitle("{} - {}".format(plots['title'], title), y=0.99)
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

            loam_data, rovio_data = load_ros_bag(bag_abs_path)
            loam_times, loam_diagnostics, loam_odometry = numpify_diagnostics(loam_data, hessian=True)
            # rovio_times, rovio_diagnostics, rovio_odometry = numpify_diagnostics(rovio_data, hessian=False)

            loam_times -= loam_times[0]

            # Checkpoint save
            tmp_data = {
                'loam_times': loam_times,
                'loam_diagnostics': loam_diagnostics,
                'loam_odometry': loam_odometry,
                # 'rovio_times': rovio_times,
                # 'rovio_diagnostics': rovio_diagnostics,
                # 'rovio_odometry': rovio_odometry
            }

            print("Dumping data...")
            with open(pkl_abs_path, "wb") as fp:
                pickle.dump(tmp_data, fp)
        else:
            with open(pkl_abs_path, "rb") as fp:
                tmp_data = pickle.load(fp)

        tmp_data['degen_regions_rot'] = degen_regions_rot[bag_name]
        tmp_data['degen_regions_trans'] = degen_regions_trans[bag_name]

        for label, regions in zip(['is_degen_rot', 'is_degen_trans'], [degen_regions_rot[bag_name], degen_regions_trans[bag_name]]):
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

    trans_scores = [
        {
            'func': degen_funcs.d_opt,
            'matrix_name': 'hessian',
            'label': "D-Opt"
        },
        {
            'func': degen_funcs.a_opt,
            'matrix_name': 'hessian',
            'label': "A-Opt"
        },
        {
            'func': degen_funcs.e_opt,
            'matrix_name': 'hessian',
            'label': "E-Opt"
        },
        {
            'func': degen_funcs.max_eigen,
            'matrix_name': 'covariance',
            'label': "Max Eigen"
        },
        {
            'func': degen_funcs.d_opt_ratio,
            'matrix_name': 'hessian',
            'label': "D-Opt Ratio"
        },
        {
            'func': degen_funcs.a_opt_ratio,
            'matrix_name': 'hessian',
            'label': "A-Opt Ratio"
        },
        {
            'func': degen_funcs.e_opt_ratio,
            'matrix_name': 'hessian',
            'label': "E-Opt Ratio"
        },
        {
            'func': degen_funcs.max_eigen_ratio,
            'matrix_name': 'covariance',
            'label': "Max Eigen Ratio"
        },
        {
            'func': degen_funcs.jensen_bregman,
            'matrix_name': 'covariance',
            'label': "JB LogDet Div"
        },
        {
            'func': degen_funcs.correlation_matrix_distance,
            'matrix_name': 'covariance',
            'label': "Corr. Mat. Dist"
        },
        # {
        #     'func': degen_funcs.kullback_leibler,
        #     'matrix_name': 'covariance',
        #     'label': "Kullback Leibler"
        # },
        {
            'func': degen_funcs.norm_1,
            'matrix_name': 'hessian',
            'label': "1 Norm"
        },
        {
            'func': degen_funcs.norm_2,
            'matrix_name': 'hessian',
            'label': "2 Norm"
        },
        {
            'func': degen_funcs.norm_frobenius,
            'matrix_name': 'hessian',
            'label': "F Norm"
        },
        # {
        #     'func': degen_funcs.norm_nuclear,
        #     'matrix_name': 'hessian',
        #     'label': "N Norm"
        # },
        {
            'func': degen_funcs.norm_1_ratio,
            'matrix_name': 'hessian',
            'label': "1 Norm Ratio"
        },
        {
            'func': degen_funcs.norm_2_ratio,
            'matrix_name': 'hessian',
            'label': "2 Norm Ratio"
        },
        {
            'func': degen_funcs.norm_frobenius_ratio,
            'matrix_name': 'hessian',
            'label': "F Norm Ratio"
        },
        # {
        #     'func': degen_funcs.norm_nuclear_ratio,
        #     'matrix_name': 'hessian',
        #     'label': "N Norm Ratio"
        # },
    ]

    for d in trans_scores:
        d['scores'] = np.array([], dtype=np.float64)

    rot_scores = copy.deepcopy(trans_scores)

    for bagname, bagdata in data.items():
        all_degen_rot = np.concatenate([all_degen_rot, bagdata['is_degen_rot']])
        all_degen_trans = np.concatenate([all_degen_trans, bagdata['is_degen_trans']])

        for x, subset in zip([rot_scores, trans_scores], ["rot", "trans"]):
            for d in x:
                y = apply_degen_function(data[bagname]['loam_odometry'][d['matrix_name']], data[bagname]['loam_odometry']['pose'], subset, d['func'])
                d['scores'] = np.concatenate([d['scores'], y])
                # print("+ {} = {}".format(y.shape, d['scores'].shape))

    fig_rot: plt.Figure
    axeses_rot: List[plt.Axes]
    fig_rot, axeses_rot = plt.subplots(nrows=4, ncols=4, gridspec_kw={
        'left': 0.1, 'right': 0.95, 'top': 0.9, 'bottom': 0.1,
        'hspace': 0.4, 'wspace': 0.25
    })

    fig_trans: plt.Figure
    axeses_trans: List[plt.Axes]
    fig_trans, axeses_trans = plt.subplots(nrows=4, ncols=4, gridspec_kw={
        'left': 0.1, 'right': 0.95, 'top': 0.9, 'bottom': 0.1,
        'hspace': 0.4, 'wspace': 0.25
    })

    fig: plt.Figure
    for x, subset, axeses, is_degen, label, fig in zip(
            [rot_scores, trans_scores],
            ["rot", "trans"],
            [axeses_rot, axeses_trans],
            [all_degen_rot, all_degen_trans],
            ["Rotation", "Translation"],
            [fig_rot, fig_trans]
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
            if d['matrix_name'].startswith('cov'):
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
        for axes in axeses[3, :]:
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

    for plot in PLOTS:
        fig: plt.Figure
        axeses: List[List[plt.Axes]]
        fig, axeses = plt.subplots(nrows=len(plot['plots']), ncols=len(data), sharex='col', sharey='row')

        for i, axes_row, plot_metadata in zip(range(0, 1000), axeses, plot['plots']):
            if type(axes_row) != np.ndarray:
                axes_row = [axes_row]
            for j, axes, (bagname, bagdata) in zip(range(0, 1000), axes_row, data.items()):
                if i == 0:
                    axes.set_title(bagname[:-12][:15])
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


if __name__ == "__main__":

    data = load_all_bags(BAG_DIR, DEGEN_ROT, DEGEN_TRANS)
    # plot_all_over_time(data, PLOTS)
    # plot_all_rocs(data)
    compare_datasets(data)
