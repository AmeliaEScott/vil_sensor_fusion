#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu


class ImuFilter:
    def __init__(self, name="imu_filter_node"):
        rospy.init_node(name)

        self.imus = {}
        self.filter_length = rospy.get_param("~filter_length")

        for imu_name in ["vio", "lidar", "fusion"]:
            def callback(msg, name=imu_name):
                self.imu_callback(msg, name)
            self.imus[imu_name] = {
                'data': [],
                'pub': rospy.Publisher("~{}".format(imu_name), Imu, queue_size=1),
                'sub': rospy.Subscriber("/imu/{}".format(imu_name), Imu, callback)
            }

    def imu_callback(self, msg: Imu, name):
        data = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])

        buffer = self.imus[name]['data']

        buffer.append(data)
        buffer = buffer[-self.filter_length:]
        data = sum(buffer) / len(buffer)

        msg.linear_acceleration.x = data[0]
        msg.linear_acceleration.y = data[1]
        msg.linear_acceleration.z = data[2]
        msg.angular_velocity.x = data[3]
        msg.angular_velocity.y = data[4]
        msg.angular_velocity.z = data[5]
        self.imus[name]['pub'].publish(msg)


if __name__ == "__main__":
    node = ImuFilter()
    rospy.spin()