#!/usr/bin/env python3

"""
Basically an older, worse version of find_correlations.py.
"""

from typing import List, Tuple

import matplotlib
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import linregress
import yaml
import pickle
import json
from loam_velodyne.msg import OdometryWithHessian
from nav_msgs.msg import Odometry
from vil_fusion.msg import DiagnosticMessage
from degeneracy_detection_functions import degen_funcs
from matplotlib import pyplot

YAML_DATA = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/sample_bags/OdometryData.yaml"
# YAML_DATA = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/sample_bags/TEMP2.yaml"

#
# with open(YAML_DATA, "r") as fp:
#     data = yaml.load(fp)
#
# """
# data = {
#     'loam': [
#         (OdometryWithHessian, DiagnosticMessage), ...
#     ],
#     'rovio': [
#         (Odometry, DiagnosticMessage), ...
#     ]
# }
# """
#
#
# def numpify_diagnostics(data, hessian=False):
#     diagnostics = {
#         'relative_dist_err': np.zeros(shape=(len(data)), dtype=np.float64),
#         'abs_rot_err': np.zeros(shape=(len(data)), dtype=np.float64),
#         'rel_linear_vel_err': np.zeros(shape=(len(data)), dtype=np.float64),
#         'rel_rot_vel_err': np.zeros(shape=(len(data)), dtype=np.float64)
#     }
#     odometry = {
#         'covariance': np.zeros(shape=(6, 6, len(data)), dtype=np.float64),
#         'pose': np.zeros(shape=(6, 1, len(data)), dtype=np.float64),
#     }
#     if hessian:
#         odometry['hessian'] = np.zeros(shape=(6, 6, len(data)), dtype=np.float64)
#
#     for i, (odom, diag) in enumerate(data):
#         for field in diagnostics.keys():
#             diagnostics[field][i] = diag[field]
#         if hessian:
#             odometry['hessian'][:, :, i] = np.reshape(np.array(odom['hessian'], dtype=np.float64), (6, 6))
#             covariance = np.array(odom['odom']['pose']['covariance'], dtype=np.float64)
#             position = odom['odom']['pose']['pose']['position']
#             orientation = odom['odom']['pose']['pose']['orientation']
#         else:
#             covariance = np.array(odom['pose']['covariance'], dtype=np.float64)
#             position = odom['pose']['pose']['position']
#             orientation = odom['pose']['pose']['orientation']
#         orientation = np.array([
#             orientation['x'],
#             orientation['y'],
#             orientation['z'],
#             orientation['w']
#         ], dtype=np.float64)
#         orientation = Rotation.from_quat(orientation)
#         position = np.array([
#             position['x'],
#             position['y'],
#             position['z'],
#         ], dtype=np.float64)
#         pose = np.concatenate([position, orientation.as_euler("XYZ")])
#         odometry['covariance'][:, :, i] = np.reshape(covariance, (6, 6))
#         odometry['pose'][:, :, i] = np.reshape(pose, (6, 1))
#     return diagnostics, odometry
#
#
# loam_diagnostics, loam_odom = numpify_diagnostics(data['loam'], hessian=True)
# rovio_diagnostics, rovio_odom = numpify_diagnostics(data['rovio'])
#
# print("##################################################\n\n\n\n")
# print(loam_diagnostics)
# print("\n###################################################\n")
# print(loam_odom)
#
# with open(YAML_DATA + ".pkl", "wb") as fp:
#     pickle.dump({
#         'loam_diag': loam_diagnostics,
#         'loam_odom': loam_odom,
#         'rovio_diag': rovio_diagnostics,
#         'rovio_odom': rovio_odom
#     }, fp)

with open(YAML_DATA + ".pkl", "rb") as fp:
    data = pickle.load(fp)


for x in ['loam_diag', 'rovio_diag']:
    newdata = {}
    for key in data[x]:
        newdata['log({})'.format(key)] = np.log(data[x][key])
    data[x] = {**data[x], **newdata}


def run_experiment(func, odom_data):
    n = odom_data['covariance'].shape[2]
    name = func.__name__
    results = {
        name + '_covariance_all': np.zeros((n,), dtype=np.float64),
        name + '_covariance_trans': np.zeros((n,), dtype=np.float64),
        name + '_covariance_rot': np.zeros((n,), dtype=np.float64),
    }
    if 'hessian' in odom_data:
        results[name + '_hessian_all'] = np.zeros((n,), dtype=np.float64)
        results[name + '_hessian_trans'] = np.zeros((n,), dtype=np.float64)
        results[name + '_hessian_rot'] = np.zeros((n,), dtype=np.float64)
    for i in range(1, odom_data['covariance'].shape[2]):
        label = name + '_covariance_all'
        args = {
            'mat_now': odom_data['covariance'][:, :, i],
            'mat_prev': odom_data['covariance'][:, :, i - 1],
            'pose_now': odom_data['pose'][:, :, i],
            'pose_prev': odom_data['pose'][:, :, i - 1]
        }
        results[label][i] = func(**args)

        label = name + '_covariance_trans'
        args = {
            'mat_now': odom_data['covariance'][0:3, 0:3, i],
            'mat_prev': odom_data['covariance'][0:3, 0:3, i - 1],
            'pose_now': odom_data['pose'][0:3, :, i],
            'pose_prev': odom_data['pose'][0:3, :, i - 1]
        }
        results[label][i] = func(**args)

        label = name + '_covariance_rot'
        args = {
            'mat_now': odom_data['covariance'][3:6, 3:6, i],
            'mat_prev': odom_data['covariance'][3:6, 3:6, i - 1],
            'pose_now': odom_data['pose'][3:6, :, i],
            'pose_prev': odom_data['pose'][3:6, :, i - 1]
        }
        results[label][i] = func(**args)

        if 'hessian' in odom_data:
            label = name + '_hessian_all'
            args = {
                'mat_now': odom_data['hessian'][:, :, i],
                'mat_prev': odom_data['hessian'][:, :, i - 1],
                'pose_now': odom_data['pose'][:, :, i],
                'pose_prev': odom_data['pose'][:, :, i - 1]
            }
            results[label][i] = func(**args)

            label = name + '_hessian_rot'
            args = {
                'mat_now': odom_data['hessian'][0:3, 0:3, i],
                'mat_prev': odom_data['hessian'][0:3, 0:3, i - 1],
                'pose_now': odom_data['pose'][0:3, :, i],
                'pose_prev': odom_data['pose'][0:3, :, i - 1]
            }
            results[label][i] = func(**args)

            label = name + '_hessian_trans'
            args = {
                'mat_now': odom_data['hessian'][3:6, 3:6, i],
                'mat_prev': odom_data['hessian'][3:6, 3:6, i - 1],
                'pose_now': odom_data['pose'][3:6, :, i],
                'pose_prev': odom_data['pose'][3:6, :, i - 1]
            }
            results[label][i] = func(**args)
    return results

# loam_results = {}
# rovio_results = {}
#
# for func in degen_funcs:
#     print('Doing {} for loam...'.format(func.__name__))
#     loam_results = {**loam_results, **run_experiment(func, data['loam_odom'])}
#     print('Doing {} for rovio...'.format(func.__name__))
#     rovio_results = {**rovio_results, **run_experiment(func, data['rovio_odom'])}
#
# print("\n\n\n######################################################\n\n\n")
#
# print(loam_results)
# print("\n\n\n######################################################\n\n\n")
# print(rovio_results)
#
# with open('results.pkl', "wb") as fp:
#     pickle.dump({
#         'loam': loam_results,
#         'rovio': rovio_results
#     }, fp)

with open('results.pkl', 'rb') as fp:
    results = pickle.load(fp)

loam_results = results['loam']
rovio_results = results['rovio']

# for result in [loam_results, rovio_results]:
#     newdata = {}
#     for key in result:
#         newdata["log({})".format(key)] = np.log(result[key])
#     for key in newdata:
#         result[key] = newdata[key]

def p_hack(metrics, diagnostics):
    results = {}
    for metric in metrics:
        for diagnostic in diagnostics:
            label = "{} -> {}".format(metric, diagnostic)
            print("Calculating regression: {}".format(label))
            x = metrics[metric].copy()
            y = diagnostics[diagnostic].copy()
            non_nan_indexes = np.logical_and(
                np.logical_not(np.isnan(x)),
                np.logical_not(np.isnan(y)),
            )
            x = x[non_nan_indexes]
            y = y[non_nan_indexes]

            slope, intercept, rvalue, _, _ = linregress(x, y)
            results[label] = rvalue

            # label = "{}_mat_ratio -> {}".format(metric, diagnostic)
            # x = metrics[metric].copy()
            # x = np.matmul(x[1:], np.linalg.inv(x[:-1]))
            # y = diagnostics[diagnostic].copy()
            # non_nan_indexes = np.logical_and(
            #     np.logical_not(np.isnan(x)),
            #     np.logical_not(np.isnan(y)),
            # )
            # x = x[non_nan_indexes]
            # y = y[non_nan_indexes]
            #
            # slope, intercept, rvalue, _, _ = linregress(x, y)
            # results[label] = rvalue

    return results


loam_correlations = p_hack(loam_results, data['loam_diag'])
rovio_correlations = p_hack(rovio_results, data['rovio_diag'])

FINAL_RESULT = {
    'loam': loam_correlations,
    'rovio': rovio_correlations
}

print(json.dumps(FINAL_RESULT, indent=4))

print("######## LOAM: ##################")
for k, v in sorted(loam_correlations.items(), key=lambda x: 0 if np.isnan(x[1]) else -abs(x[1])):
    print("  {}: {}".format(k, v))
print("\n\n\n")

print("######## ROVIO: ##################")
for k, v in sorted(rovio_correlations.items(), key=lambda x: 0 if np.isnan(x[1]) else -abs(x[1])):
    print("  {}: {}".format(k, v))


for label, source in [
    ("d_opt_hessian_rot -> log(rel_linear_vel_err)", "loam"),
    ("jensen_bregman_covariance_trans -> log(rel_linear_vel_err)", "loam"),
    ("e_opt_hessian_trans -> log(rel_linear_vel_err)", "loam"),
    ("jensen_bregman_covariance_rot -> log(relative_dist_err)", "rovio"),
    ("norm_1_covariance_rot -> log(relative_dist_err)", "rovio")
]:
    x_label, y_label = label.split(" -> ")
    if source == "loam":
        x = loam_results[x_label]
        y = data['loam_diag'][y_label]
    else:
        x = rovio_results[x_label]
        y = data["rovio_diag"][y_label]

    non_nan_indexes = np.logical_and(
        np.logical_not(np.isnan(x)),
        np.logical_not(np.isnan(y)),
    )
    x = x[non_nan_indexes]
    y = y[non_nan_indexes]

    # keep_range = np.percentile(x, [1, 99])
    # inlier_indexes = np.logical_and(x > keep_range[0], x < keep_range[1])
    # x = x[inlier_indexes]
    # y = y[inlier_indexes]

    slope, intercept, rvalue, _, _ = linregress(x, y)
    pyplot.scatter(x, y)
    lx = np.array([np.min(x), np.max(x)])
    ly = slope * lx + intercept
    pyplot.plot(lx, ly, '--')
    pyplot.xlabel(x_label)
    pyplot.ylabel(y_label)
    pyplot.title("{}: r-value={}".format(source, rvalue))
    pyplot.show()
