#!/usr/bin/env python3

import roslaunch
import rospkg
import rospy
from std_msgs.msg import Bool
import carla
import itertools
import os
import subprocess
import time

"""
This script automates the process of running multiple Carla simulation runs.
To configure, adjust the constants (variables named in ALL_CAPS) at the top of this file.

This script will perform the following steps, for every given combination of vehicle and map:
 - Tell the Carla simulator to load the map
 - Launch the ROS bridge using launch/carla_ros_bridge.launch.
    - This automatically spawns a vehicle, with sensors, and records a ROS bag
 - Enable the vehicle autopilot
 - Wait SIM_TIME_SECS seconds while the simulation runs (Usually on the order of hours, because
   the simulation runs so slowly on my laptop)
 - Stop the simulation
 - Use scripts/fix_rosbag_time.py to fix the timing in the bag
 - Move the bag to BAG_DEST folder
    - BAG_DEST was always on my external hard drive, because my laptop's internal hard drive kept
      filling up with bags
This results in a series of bags of real-time simulation data, named after the vehicle and map used.
"""

pkg = rospkg.RosPack()

CARLA_PREFIX = "/home/timothy/Code/carla"
BAG_SOURCE = os.path.join(pkg.get_path("carla_tools"), "rosbags")  # Where does the carla_ros_bridge.launch file store the bags? (Don't change this unless you have changed the launch file)
BAG_DEST = "/media/timothy/1ABED71A5F421E8D/TimothyScott/rawdata/autoexperiments_v7.3"  # Where to move the bags after recording?
LAUNCH_FILE = os.path.join(pkg.get_path("carla_tools"), "launch", "carla_ros_bridge.launch")  # Launch file to launch the simulator ROS bridge and spawn a vehicle
FIX_ROSBAG_SCRIPT = os.path.join(pkg.get_path("carla_tools"), "scripts", "fix_rosbag_time.py")  # Script to fix timings in bag so that it plays in real-time
VEHICLE_FILTER = "vehicle.tesla.model3"  # Unused, I think

SIM_TIME_SECS = int(60 * 60 * 1)  # How long to run the simulation (In real-life wall time, not simulated time)

maps = [  # List of Carla maps to run
    # "TestTown",
    # "Test4",
    "Test5",
]
vehicles = ["vehicle.tesla.model3", "vehicle.audi.tt", "vehicle.harley-davidson.low_rider", "vehicle.bmw.grandtourer"]  # List of vehicles to test
# vehicles = ["vehicle.tesla.model3"]
client = carla.Client("localhost", 2000)

for map in maps:

    client.load_world(map)

    for vehicle in vehicles:
        label = "{}_{}".format(map, vehicle)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [LAUNCH_FILE, 'bagname:={}'.format(label), "vehicle_filter:={}".format(vehicle), "avoid_stoplights:=true"]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()

        node = rospy.init_node("auto_experiment_runner")

        rospy.sleep(rospy.Duration.from_sec(30))

        manual_control_override = rospy.Publisher("/carla/ego_vehicle/vehicle_control_manual_override",
                                                  Bool, queue_size=10, latch=True)
        autopilot = rospy.Publisher("/carla/ego_vehicle/enable_autopilot",
                                    Bool, queue_size=10, latch=True)
        print("############# PUBLISHING AUTOPILOT #####################")
        for _ in range(0, 50):
            # NO IDEA WHY I MUST PUBLISH SO MANY TIMES!
            manual_control_override.publish(Bool(data=False))
            autopilot.publish(Bool(data=True))
            rospy.sleep(rospy.Duration.from_sec(0.1))

        rospy.sleep(rospy.Duration.from_sec(SIM_TIME_SECS))
        print("####################### SHUTTING DOWN ROSLAUNCH #####################")
        launch.shutdown()

        manual_control_override.unregister()
        autopilot.unregister()

        newbag = os.path.join(BAG_SOURCE, "{}.bag".format(label))
        newbag_timefixed = os.path.join(BAG_SOURCE, "{}_timefixed.bag".format(label))
        moved_bag = os.path.join(BAG_DEST, "{}.bag".format(label))

        print("####################### FIXING ROSBAG TIME ########################")
        subprocess.run(["python", FIX_ROSBAG_SCRIPT, newbag])
        print("####################### MOVING TO HARD DRIVE #########################")
        subprocess.run(["mv", newbag_timefixed, moved_bag])
        print("####################### REMOVING CRAP ###############################")
        subprocess.run(["rm", newbag])

        print("################# {} -> {} -> {} ################".format(newbag, newbag_timefixed, moved_bag))
