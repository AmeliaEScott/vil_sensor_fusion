#!/usr/bin/env python2
# Taken directly from http://wiki.ros.org/rosbag/Cookbook
# This script is used to convert real time to simulated time on a ROS bag.
# When recording and replaying a ROS bag, the headers in the messages will be ignored. Instead, rosbag
# will replay the messages at the same rate they were received. This is no good if the simulation runs slower than
# real-time. So this script replaces those timestamps with the timestamps from the header of each message (which
# was populated with simulation time), so that when you run `rosbag play` on the fixed bag, the messages will
# play at a rate that looks more like reality.
#
# To use, simply run:
#  python fix_rosbag_time.py <bag file>

import rosbag
import sys

frame_remap = {
    # '/map': '/carla_map'
}

if len(sys.argv) != 2:
    print("Usage: python3 fix_rosbag_time.py <bag file>")
    sys.exit(-1)

input_bagname = sys.argv[1]
output_bagname = sys.argv[1].replace(".bag", "_timefixed.bag")

print("Processing bag...")
with rosbag.Bag(output_bagname, 'w') as outbag:
    # last_msgs = dict()
    for topic, msg, t in rosbag.Bag(input_bagname).read_messages():
        # This also replaces tf timestamps under the assumption
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            for tf in msg.transforms:
                if tf.header.frame_id in frame_remap:
                    tf.header.frame_id = frame_remap[tf.header.frame_id]
                if tf.child_frame_id in frame_remap:
                    tf.child_frame_id = frame_remap[tf.child_frame_id]
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        elif not msg._has_header:
            outbag.write(topic, msg, t)
        else:
            # if topic not in last_msgs or last_msgs[topic].header.stamp != msg.header.stamp:
            #     last_msgs[topic] = msg
            if msg.header.frame_id in frame_remap:
                msg.header.frame_id = frame_remap[msg.header.frame_id]
            outbag.write(topic, msg, msg.header.stamp)
            # else:
            #     print("Duplicate timestamp in topic {}".format(topic))
