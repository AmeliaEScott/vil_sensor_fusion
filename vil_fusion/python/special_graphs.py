#!/usr/bin/env python3

# Graphs of correspondence distances vs. shift
from typing import List

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from loam_velodyne.msg import OdometryWithHessian

fp = open("/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/sample_bags/TEST3.bag", "rb")
bag = rosbag.Bag(fp)

msgs: List[OdometryWithHessian] = list(map(lambda x: x.message, bag.read_messages("/loam/frame_transform/odometry/ros")))
print(len(msgs))
selected_msgs = [msgs[340], msgs[1065]]

for label, shifts, dists_offset, dists_name in zip(
    ["rot", "rot_surf", "rot_corner", "trans", "trans_surf", "trans_corner"],
    [selected_msgs[0].shift_rot, selected_msgs[0].shift_rot, selected_msgs[0].shift_rot,
     selected_msgs[0].shift_trans, selected_msgs[0].shift_trans, selected_msgs[0].shift_trans],
    [3, 3, 3, 0, 0, 0],
    ["dists", "dists_surface", "dists_corner", "dists", "dists_surface", "dists_corner"]
):
    fig: plt.Figure
    fig, axeses = plt.subplots(3, 2, sharex="all", sharey="row")

    ax: plt.Axes
    for ax_row, d in zip(axeses, range(dists_offset, 10)):
        msg: OdometryWithHessian
        for ax, msg in zip(ax_row, selected_msgs):
            dists = getattr(msg, dists_name)
            dists = np.array(dists).reshape((-1, 6))
            ax.scatter(shifts, dists[:, d])
            # ax.set_ylim(bottom=0, top=0.01)
            ax.set_xlim(left=0.0, right=0.2)
    fig.suptitle(label)
    fig.show()