#!/usr/bin/env python3
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

LOAM_DIAGNOSTIC_TOPIC = "/diagnostics/loam_odom"
ROVIO_DIAGNOSTIC_TOPIC = "/diagnostics/rovio"

LOAM_ODOM_TOPIC = "/loam/frame_transform/odometry/ros"
ROVIO_ODOM_TOPIC = "/rovio/odometry"

# BAG_DIR = "/media/timothy/1ABED71A5F421E8D/TimothyScott/rawdata/experiment_results"
# BAG_NAME = "Test1_vehicle.tesla.model3_results.bag"
BAG_DIR = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/carla_tools/rosbags"

DEGEN_REGIONS = {
    # "Test1_vehicle.tesla.model3_results.bag": [
    #     (55.0, 80.0),
    #     (170.0, 185.0),
    # ],
    # "Test2_Denser_vehicle.tesla.model3_results.bag": [
    #     (50.0, 85.0),
    #     (165.0, 185.0),
    # ],
    # "Test3_vehicle.tesla.model3_results.bag": [
    #
    # ],
    # "Test4_vehicle.tesla.model3_results.bag": [
    #     (134.0, 151.0),
    # ],
    # "Town02_vehicle.tesla.model3_results.bag": [
    #
    # ],
    # "Town03_vehicle.tesla.model3_results.bag": [
    #     (110.0, 160.0),
    # ],
    # "V1_01_easy_results.bag": [
    #
    # ],
    "V1_03_difficult_results.bag": [

    ],
    "san_rafeal_results.bag": [
        (1561411065.0, 1561411103.0)
    ]
}

PLOTS = [
    # {
    #     'source': 'loam',
    #     'title': 'Loam Translation',
    #     'plots': [
    #         {
    #             'diagnostic': True,
    #             'roc': False,
    #             'metric': 'rel_linear_vel_err',
    #             'log': False,
    #             'label': 'GT Vel. Error',
    #             'ylim': 0.5
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'd_opt',
    #             'log': False,
    #             'label': 'D Opt',
    #             'matrix': 'hessian',
    #             'matrix_subset': 'trans'
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'e_opt',
    #             'log': False,
    #             'label': 'E Opt',
    #             'matrix': 'hessian',
    #             'matrix_subset': 'trans'
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'a_opt',
    #             'log': False,
    #             'label': 'A Opt',
    #             'matrix': 'hessian',
    #             'matrix_subset': 'trans'
    #         },
    #     ]
    # },
    # {
    #     'source': 'loam',
    #     'title': 'Loam Rotation',
    #     'plots': [
    #         {
    #             'diagnostic': True,
    #             'roc': False,
    #             'metric': 'abs_rot_vel_err',
    #             'log': False,
    #             'label': 'GT Ang. Vel. Err'
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'd_opt',
    #             'log': False,
    #             'label': 'D Opt',
    #             'matrix': 'hessian',
    #             'matrix_subset': 'rot'
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'e_opt',
    #             'log': False,
    #             'label': 'E Opt',
    #             'matrix': 'hessian',
    #             'matrix_subset': 'rot'
    #         },
    #         {
    #             'diagnostic': False,
    #             'roc': True,
    #             'metric': 'a_opt',
    #             'log': False,
    #             'label': 'A Opt',
    #             'matrix': 'hessian',
    #             'matrix_subset': 'rot'
    #         },
    #     ]
    # },
    {
        'source': 'rovio',
        'title': 'Twist Covariance - Rotation',
        'plots': [
            {
                'diagnostic': True,
                'roc': False,
                'metric': 'abs_rot_vel_err',
                'log': False,
                'label': 'GT Rot. Err',
                # 'ylim': 0.05
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'd_opt',
                'log': False,
                'label': 'D Opt',
                'matrix': 'cov_twist',
                'matrix_subset': 'rot',
                # 'ymin': 0.000153,
                # 'ylim': 0.000155,
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'e_opt',
                'log': False,
                'label': 'E Opt',
                'matrix': 'cov_twist',
                'matrix_subset': 'rot',
                # 'ymin': 0.000150,
                # 'ylim': 0.000154,
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'norm_frobenius',
                'log': False,
                'label': 'F Norm',
                'matrix': 'cov_twist',
                'matrix_subset': 'rot',
            },
        ]
    },
    {
        'source': 'rovio',
        'title': 'Twist Covariance - Translation',
        'plots': [
            {
                'diagnostic': True,
                'roc': False,
                'metric': 'abs_linear_vel_err',
                'log': False,
                'label': 'GT Vel. Error',
                #'ylim': 1.0
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'd_opt',
                'log': False,
                'label': 'D Opt',
                'matrix': 'cov_twist',
                'matrix_subset': 'trans'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'e_opt',
                'log': False,
                'label': 'E Opt',
                'matrix': 'cov_twist',
                'matrix_subset': 'trans'
            },
            {
                'diagnostic': False,
                'roc': True,
                'metric': 'a_opt',
                'log': False,
                'label': 'A Opt',
                'matrix': 'cov_twist',
                'matrix_subset': 'trans'
            },
        ]
    },
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


def plot(
        times: np.ndarray,
        diagnostics,
        odometry,
        degen_regions: List[Tuple[SupportsFloat, SupportsFloat]],
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
        print(plot_metadata)
        log = plot_metadata['log']
        metric = plot_metadata['metric']
        is_diagnostic = plot_metadata['diagnostic']
        label = plot_metadata['label']
        do_roc = plot_metadata['roc']

        if is_diagnostic:
            y = diagnostics[metric]
        else:
            matrix_name = plot_metadata['matrix']
            matrix_subset = plot_metadata['matrix_subset']

            matrix = odometry[matrix_name]

            pose = odometry['pose']

            # if source == "loam":
            if matrix_subset == "rot":
                matrix = matrix[3:6, 3:6, :]
                pose = pose[3:6, :, :]
            elif matrix_subset == "trans":
                matrix = matrix[0:3, 0:3, :]
                pose = pose[0:3, :, :]
            elif matrix_subset in ["x", "y", "z", "roll", "pitch", "yaw"]:
                i = ["x", "y", "z", "roll", "pitch", "yaw"].index(matrix_subset)
                matrix = matrix[i:i+1, i:i+1, :]
                pose = pose[i:i+1, :, :]
            # else:
            #     raise RuntimeWarning("I did not implement this part for Rovio")

            degen_func = getattr(degen_funcs, metric)
            y = np.zeros_like(times)
            for i in range(1, matrix.shape[2]):
                args = {
                    'mat_now': matrix[:, :, i],
                    'mat_prev': matrix[:, :, i - 1],
                    'pose_now': pose[:, :, i],
                    'pose_prev': pose[:, :, i - 1]
                }

                y[i] = degen_func(**args)

        y = y[1:]

        ax.plot(times[10:], y[9:])
        shade_degen_regions(ax, (np.min(y), np.max(y)), degen_regions)
        if 'ymin' in plot_metadata:
            ax.set_ylim(bottom=plot_metadata['ymin'])
        else:
            ax.set_ylim(bottom=0)
        if 'ylim' in plot_metadata:
            ax.set_ylim(top=plot_metadata['ylim'])
        if log:
            ax.set_yscale("log")
        ax.set_ylabel(label)

        if do_roc:
            # ROC
            is_degen = np.zeros_like(times, dtype=np.bool)
            for degen_region in degen_regions:
                is_degen = np.logical_or(is_degen, np.logical_and(
                    times > degen_region[0],
                    times < degen_region[1]
                ))
            is_degen = is_degen[1:]

            percentiles = np.linspace(0, 100, 100, dtype=np.float64)
            thresholds = np.percentile(y, percentiles)
            if matrix_name.startswith("cov"):
                is_degen_estimate = np.expand_dims(y, 0) >= np.expand_dims(thresholds, 1)
            else:
                is_degen_estimate = np.expand_dims(y, 0) <= np.expand_dims(thresholds, 1)
            true_positives = np.logical_and(is_degen_estimate, is_degen)
            false_positives = np.logical_and(is_degen_estimate, np.logical_not(is_degen))
            tpr = np.sum(true_positives, axis=1) / np.sum(is_degen)
            fpr = np.sum(false_positives, axis=1) / np.sum(np.logical_not(is_degen))
            roc_ax.plot(fpr, tpr)
            roc_ax.set_ylabel("TPR")
            roc_ax.set_xlabel("FPR")
        else:
            roc_ax.axis('off')

    ax.set_xlabel("Time (s)")
    fig.suptitle("{} - {}".format(plots['title'], title), y=0.99)
    fig.show()


for bag_name, degen_regions in DEGEN_REGIONS.items():
    bag_abs_path = os.path.join(BAG_DIR, bag_name)
    pkl_abs_path = os.path.join(BAG_DIR, bag_name + ".pkl")

    if not os.path.isfile(pkl_abs_path):

        loam_data, rovio_data = load_ros_bag(bag_abs_path)
        loam_times, loam_diagnostics, loam_odometry = numpify_diagnostics(loam_data, hessian=True)
        rovio_times, rovio_diagnostics, rovio_odometry = numpify_diagnostics(rovio_data, hessian=False)

        # Checkpoint save
        tmp_data = {
            'loam_times': loam_times,
            'loam_diagnostics': loam_diagnostics,
            'loam_odometry': loam_odometry,
            'rovio_times': rovio_times,
            'rovio_diagnostics': rovio_diagnostics,
            'rovio_odometry': rovio_odometry
        }

        print("Dumping data...")
        with open(pkl_abs_path, "wb") as fp:
            pickle.dump(tmp_data, fp)
    else:
        with open(pkl_abs_path, "rb") as fp:
            tmp_data = pickle.load(fp)

    loam_times = tmp_data['loam_times']
    loam_diagnostics = tmp_data['loam_diagnostics']
    loam_odometry = tmp_data['loam_odometry']
    rovio_times = tmp_data['rovio_times']
    rovio_diagnostics = tmp_data['rovio_diagnostics']
    rovio_odometry = tmp_data['rovio_odometry']

    for fig_data in PLOTS:
        source = fig_data['source']
        title = bag_name.split("_")[0]

        if source == 'loam':
            plot(
                loam_times,
                loam_diagnostics,
                loam_odometry,
                degen_regions,
                fig_data,
                title
            )
        else:
            warn("Be careful, Rovio is not fully implemented")
            plot(
                rovio_times,
                rovio_diagnostics,
                rovio_odometry,
                degen_regions,
                fig_data,
                title
            )
