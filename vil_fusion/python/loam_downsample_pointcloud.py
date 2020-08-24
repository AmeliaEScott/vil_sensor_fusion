#!/usr/bin/env python3

"""
Several datasets I have use 64-channel LiDARs. This node uses LOAM to downsample them.
"""

import rospy
import numpy as np
import message_filters
from sensor_msgs.msg import PointCloud2


class PointcloudDownsampler:

    def __init__(self, node_name="pointcloud_downsampler"):
        rospy.init_node(node_name)

        # Default values for downsampling Velodyne HDL-64E from KITTI to VLP-16
        self.channels = rospy.get_param("~channels", default=64)
        self.vert_downsample = rospy.get_param("~vert_downsample", default=4)
        self.horiz_downsample = rospy.get_param("~horiz_downsample", default=2)
        self.time_downsample = rospy.get_param("~time_downsample", default=1)
        self.transpose = rospy.get_param("~transpose", default=False)

        self.input = rospy.Subscriber("/velodyne_cloud_3", PointCloud2, callback=self.callback)
        self.output = rospy.Publisher("~velodyne_cloud_downsampled", PointCloud2, queue_size=1)

    def callback(self, msg: PointCloud2):
        if msg.header.seq % self.time_downsample == 0:
            point_length = msg.point_step // 4
            data = np.reshape(np.fromstring(msg.data, dtype=np.float32), [-1, point_length])


            rings = data[:, 4]
            print(rings)
            print("RINGS: ({}, {})".format(np.min(rings), np.max(rings)))
            downsampled_data = data

            msg.data = downsampled_data.tostring()
            msg.width = downsampled_data.shape[0]
            self.output.publish(msg)


if __name__ == "__main__":
    node = PointcloudDownsampler()
    rospy.spin()
