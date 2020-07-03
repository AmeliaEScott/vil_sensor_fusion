#!/usr/bin/env python3

"""
Does the following:

 - Read ROS bags that contain odometry outputs from Loam and Rovio, as well as diagnostics (see diagnostics.py)
 - Apply every degeneracy detection function listed in degeneracy_detection_functions.py to the odometry data
 - Compare degeneracy scores to each diagnostic measurement
 - Output the correlation coefficient for every possible combination of degeneracy score and diagnostic measurement

This is basically a dumb brute-force search to find any and all degeneracy metrics which are at all useful.
"""

import os
from typing import Dict

import rospy
import numpy as np
from scipy.stats import linregress
import pickle
import itertools
import math

try:
    from .make_prettier_graphs import numpify_diagnostics, load_ros_bag, apply_degen_function
    from .degeneracy_detection_functions import degen_funcs
except ImportError:
    from make_prettier_graphs import numpify_diagnostics, load_ros_bag, apply_degen_function
    from degeneracy_detection_functions import degen_funcs

BAG_DIR = "/media/timothy/1ABED71A5F421E8D/TimothyScott/rawdata/experiment_results_v6"
CACHE_DIR = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/carla_tools/rosbags/cache"


def read_bags(bag_dir, cache_dir):
    data = []

    for bagname in os.listdir(bag_dir):
        print("Processing {}...".format(bagname))
        bag_abs_path = os.path.join(bag_dir, bagname)
        cache_abs_path = os.path.join(cache_dir, bagname + ".pkl")

        if os.path.isfile(cache_abs_path):
            print("Found cached {}".format(cache_abs_path))
            with open(cache_abs_path, "rb") as fp:
                data.append(pickle.load(fp))
        else:
            print("No cache, reading bag")
            loam_data, rovio_data = load_ros_bag(bag_abs_path)
            print("Processing loam odometry...")
            loam_times, loam_diagnostics, loam_odometry = numpify_diagnostics(loam_data, hessian=True)
            print("Processing rovio odometry...")
            rovio_times, rovio_diagnostics, rovio_odometry = numpify_diagnostics(rovio_data, hessian=False)
            new_data = {
                "loam_times": loam_times,
                "loam_diagnostics": loam_diagnostics,
                "loam_odometry": loam_odometry,
                "rovio_times": rovio_times,
                "rovio_diagnostics": rovio_diagnostics,
                "rovio_odometry": rovio_odometry
            }
            print("Caching {}".format(bagname))
            with open(cache_abs_path, "wb") as fp:
                pickle.dump(new_data, fp)
            data.append(new_data)
        print("Bags read so far: {}".format(len(data)))

    return merge_data(data)


def merge_data(data):
    merged_data = data[0]
    del merged_data['loam_times']
    del merged_data['rovio_times']
    for bag in data[1:]:
        for a in merged_data.keys():
            for b in merged_data[a].keys():
                orig = merged_data[a][b]
                new = bag[a][b]
                merged = np.concatenate([orig, new], axis=len(orig.shape) - 1)
                # print("Merging [{}][{}]: {} + {} = {}".format(
                #     a, b, orig.shape, new.shape, merged.shape
                # ))
                merged_data[a][b] = merged

    # print("########################################################################")
    # for a in merged_data.keys():
    #     for b in merged_data[a].keys():
    #         print("merged[{}][{}]: {}".format(a, b, merged_data[a][b].shape))
    return merged_data


def p_hack(odometry: Dict[str, np.ndarray], diagnostics: Dict[str, np.ndarray]):
    matrices = list(filter(lambda x: x[0] != "pose", odometry.items()))
    diagnostics = diagnostics.items()
    # diagnostics = [("rel_linear_vel_err", diagnostics["rel_linear_vel_err"]), ("abs_rot_vel_err", diagnostics["abs_rot_vel_err"])]
    matrix_subsets = ["all", "rot", "trans", "x", "y", "z", "roll", "pitch", "yaw"]
    # matrix_subsets = ["rot", "trans"]
    functions = degen_funcs
    logs = [True, False]
    # logs = [False]

    results = []  # Array of (function label, diagnostic label, r-value)

    all_possible_options = itertools.product(
        matrices, matrix_subsets, diagnostics, functions, logs
    )

    total_length = len(matrices) * len(matrix_subsets) * len(diagnostics) * len(functions) * len(logs)

    for i, ((matrix_name, matrix), matrix_subset, (diag_name, diag_array), function, log_degen_func) \
            in enumerate(all_possible_options):
        label = "{}({}[{}])".format(function.__name__, matrix_name, matrix_subset)
        if log_degen_func:
            label = "log({})".format(label)
        print("[{:5d}/{:5d}] {} -> {}".format(i, total_length, label, diag_name))
        y = apply_degen_function(matrix, odometry["pose"], matrix_subset, function)
        if log_degen_func:
            y = np.log(y)

        nan_filter = np.logical_not(np.logical_or(np.isnan(y), np.isnan(diag_array)))
        if np.sum(nan_filter) > 0:
            slope, intercept, rvalue, _, _ = linregress(diag_array[nan_filter], y[nan_filter])
            results.append((label, diag_name, rvalue))

        y = y[1:] - y[:-1]
        diag_array = diag_array[1:]
        nan_filter = np.logical_not(np.logical_or(np.isnan(y), np.isnan(diag_array)))
        if np.sum(nan_filter):
            slope, intercept, rvalue, _, _ = linregress(diag_array[nan_filter], y[nan_filter])
            results.append(("d/dx " + label, diag_name, rvalue))

    return results


if __name__ == "__main__":
    results_cache = os.path.join(CACHE_DIR, "RESULTS.pkl")
    #
    # data = read_bags(BAG_DIR, CACHE_DIR)
    # results = p_hack(data["loam_odometry"], data["loam_diagnostics"])
    # with open(results_cache, "wb") as fp:
    #     pickle.dump(results, fp)

    with open(results_cache, "rb") as fp:
        results = pickle.load(fp)

    print("\n\n\n\n\n############################################################################\n\n\n\n\n")

    for func_label, diag_label, rvalue in sorted(
            results, key=lambda x: -1 if math.isnan(x[-1]) else abs(x[-1])):
        if "jensen_bregman" not in func_label:
            print("{} -> {}: {}".format(func_label, diag_label, rvalue))
