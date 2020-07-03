#!/usr/bin/env python2

"""
I don't remember what this script did, but it worked in conjunction with make_graphs.py, and therefore
is now obsolete.
"""

from __future__ import print_function, division
import rosbag
from loam_velodyne.msg import OdometryWithHessian
from nav_msgs.msg import Odometry
from vil_fusion.msg import DiagnosticMessage
import json
import os
import pickle
import yaml
import dill

DATA_DIR = "/media/timothy/1ABED71A5F421E8D/TimothyScott/rawdata/experiment_results"
#DATA_DIR = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/carla_tools/rosbags"
LOAM_DIAGNOSTIC_TOPIC = "/diagnostics/loam_odom"
ROVIO_DIAGNOSTIC_TOPIC = "/diagnostics/rovio"

LOAM_ODOM_TOPIC = "/loam/frame_transform/odometry/ros"
ROVIO_ODOM_TOPIC = "/rovio/odometry"

TEMP_DATA = "/home/timothy/Code/catkin_ws/src/vil_sensor_fusion/sample_bags/OdometryData"

loam_data = []  # List of (OdometryWithHessian, DiagnosticMessage)
rovio_data = []  # List of (Odometry, DiagnosticMessage)

for i, filename in enumerate(os.listdir(DATA_DIR)):
#for i, filename in enumerate(["Test1_vehicle.tesla.model3_results.bag"]):
    print("Processing {}th bag: {}".format(i, filename))
    with rosbag.Bag(os.path.join(DATA_DIR, filename), "r") as bag:
        loam_odoms = {}
        rovio_odoms = {}

        print("Reading odometry data...")
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

data = {
    'loam': [],
    'rovio': []
}

for d in loam_data:
    data['loam'].append((
        yaml.load(str(d[0])),
        yaml.load(str(d[1])),
    ))

for d in rovio_data:
    data['rovio'].append((
        yaml.load(str(d[0])),
        yaml.load(str(d[1])),
    ))

with open(TEMP_DATA + ".yaml", "w") as fp:
    yaml.dump(data, fp)