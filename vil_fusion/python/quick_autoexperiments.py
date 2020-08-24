#!/usr/bin/env python3

"""
A poorly-named script, which does the following:
 - For each bag file in DATA_HOME containing raw data (IMU, Point clouds, images):
   - Copy the bag from the external HDD to a temp folder on the main drive
   - Use roslaunch to launch the whole odometry and diagnostic system
   - Store results in a bag
   - Copy the results bag into RESULTS_DIR

This script was useful because my laptop drive did not have enough space for all of my bags, but my
external drive was too slow to run `rosbag play` at full speed.
"""

from __future__ import print_function, division, with_statement

import roslaunch
import rospkg
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
import carla
import itertools
import os
import subprocess
import time
import tempfile
import shutil

pkg = rospkg.RosPack()

DATA_HOME = "/media/timothy/1ABED71A5F421E8D/TimothyScott/rawdata/autoexperiments_v6"
RESULTS_DIR = "/media/timothy/1ABED71A5F421E8D/TimothyScott/rawdata/experiment_results_v6.3"
LAUNCH_FILE = os.path.join(pkg.get_path("vil_fusion"), "launch", "vil_fusion_bag.launch")
# LAUNCH_FILE = os.path.join(pkg.get_path("vil_fusion"), "launch", "vil_fusion_euroc.launch")

with tempfile.TemporaryDirectory() as tmpdir:
    for filename in filter(lambda x: x.endswith(".bag"), os.listdir(DATA_HOME)):
        print("######################### Processing {} ############################".format(filename))
        label = filename[:-4]
        tmpfile = os.path.join(tmpdir, filename)
        tmp_results = os.path.join(tmpdir, "{}_results.bag".format(label))
        final_results = os.path.join(RESULTS_DIR, "{}_results.bag".format(label))
        # shutil.copy(os.path.join(DATA_HOME, filename), tmpfile)
        subprocess.run(["cp", os.path.join(DATA_HOME, filename), tmpfile])

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [LAUNCH_FILE, 'new_bag:={}'.format(tmp_results), "bagfile:={}".format(tmpfile), "do_fusion:=false",
                    "rate:=0.5"]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()

        # node = rospy.init_node("auto_experiment_runner")
        #
        # while not rospy.is_shutdown():
        #     try:
        #         rospy.wait_for_message("/imu/vio", Imu, rospy.Duration.from_sec(1))
        #     except rospy.ROSException:
        #         print("####################### IMU Message timeout #########################")
        #         break
        rospy.sleep(rospy.Duration.from_sec(60 * 3))

        launch.shutdown()

        # shutil.copy(tmp_results, final_results)
        subprocess.run(["cp", tmp_results, final_results])
        # os.remove(tmp_results)
        subprocess.run(["rm", tmp_results])
        # os.remove(tmpfile)
        subprocess.run(["rm", tmpfile])