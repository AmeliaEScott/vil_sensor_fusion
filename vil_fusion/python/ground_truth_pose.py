#!/usr/bin/env python2

"""
This was part of an experiment involving ConFusion, in which the ground truth pose from the simulator was
sent to ConFusion as an input. This node just converts the transform that Carla gives into the TransformStamped
topic that ConFusion wants.
"""

from tf import TransformBroadcaster, TransformListener, transformations
import rospy
from geometry_msgs.msg import TransformStamped

rospy.init_node("ground_truth_pose_node")

pub = rospy.Publisher("~tf", TransformStamped, queue_size=1)

listener = TransformListener()
broadcaster = TransformBroadcaster()


rovio_imu_frame = "/ego_vehicle/imu/imu_fusion"

while not rospy.is_shutdown():
    try:
        listener.waitForTransform("map", rovio_imu_frame, rospy.Time.now(),
                                   timeout=rospy.Duration.from_sec(0.1))
        break
    except:
        print("No ground truth transform found yet, trying again...")
        rospy.sleep(rospy.Duration.from_sec(0.1))
init_time = listener.getLatestCommonTime("map", rovio_imu_frame)
init_trans, init_rot = listener.lookupTransform("map", rovio_imu_frame, init_time)

broadcaster.sendTransform(init_trans, init_rot, init_time, "magic_rovio_init", "map")
rospy.sleep(0.1)

last_time = 0

while not rospy.is_shutdown():
    time = listener.getLatestCommonTime("map", rovio_imu_frame)
    if time != last_time:
        last_time = time

        broadcaster.sendTransform(init_trans, init_rot, time, "magic_rovio_init", "map")

        listener.waitForTransform("magic_rovio_init", rovio_imu_frame, time, rospy.Duration.from_sec(0.01))
        trans, rot = listener.lookupTransform("magic_rovio_init", rovio_imu_frame, time)
        # print("{} ({}, {})".format(time, trans, rot))

        msg = TransformStamped()
        msg.header.stamp = time
        msg.header.frame_id = "magic_rovio_init"
        msg.child_frame_id = "rovio_imu_gt"
        msg.transform.rotation.x = rot[0]
        msg.transform.rotation.y = rot[1]
        msg.transform.rotation.z = rot[2]
        msg.transform.rotation.w = rot[3]
        msg.transform.translation.x = trans[0]
        msg.transform.translation.y = trans[1]
        msg.transform.translation.z = trans[2]
        # print(msg)
        pub.publish(msg)

    rospy.sleep(rospy.Duration.from_sec(0.0001))
