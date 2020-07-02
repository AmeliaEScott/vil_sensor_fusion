#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import PointCloud2


class PointcloudDownsampler:

    def __init__(self, node_name="pointcloud_downsampler"):
        rospy.init_node(node_name)

        self.input = rospy.Subscriber("~input", PointCloud2, callback=self.callback)
        self.output = rospy.Publisher("~output", PointCloud2, queue_size=1)

    def callback(self, msg: PointCloud2):
        # TODO: Actually do the downsampling
        self.output.publish(msg)


if __name__ == "__main__":
    node = PointcloudDownsampler()
    rospy.spin()
