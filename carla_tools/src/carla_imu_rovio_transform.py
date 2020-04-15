#!/usr/bin/env python3

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import Imu, PointCloud2
import numpy as np


class CarlaImuRovioTransform:
    def __init__(self, node_name="carla_imu_rovio_transform"):
        rospy.init_node(node_name)
        self.imu_sub = rospy.Subscriber("~imu_carla", Imu, callback=self.imu_listener)
        self.rovio_pub = rospy.Publisher("~imu_rovio", Imu, queue_size=1)
        self.loam_pub = rospy.Publisher("~imu_loam", Imu, queue_size=1)

        self.lidar_sub = rospy.Subscriber("~lidar_carla", PointCloud2, callback=self.lidar_callback)
        self.lidar_pub = rospy.Publisher("~lidar_loam", PointCloud2, queue_size=1)

    def imu_listener(self, msg):
        """
        The Carla convention is left-handed:
         x: Forward
         y: Right
         z: Up

        The ROS convention is right-handed:
         x: Forward
         y: Left
         z: Up

        The Rovio convention is right-handed:
         x: Right
         y: Down
         z: Forward

        LOAM convention is right-handed:
         x: Left
         y: Up
         z: Forward
        """
        accel = (msg.linear_acceleration.y, -msg.linear_acceleration.z, msg.linear_acceleration.x)
        gyro = (-msg.angular_velocity.y, msg.angular_velocity.z, -msg.angular_velocity.x)

        msg.header.frame_id = ""
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        self.rovio_pub.publish(msg)


        accel = (-msg.linear_acceleration.y, msg.linear_acceleration.z, msg.linear_acceleration.x)
        gyro = (msg.angular_velocity.y, -msg.angular_velocity.z, -msg.angular_velocity.x)

        msg.header.frame_id = ""
        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        self.loam_pub.publish(msg)

    def lidar_callback(self, msg):
        """
        Parameters
        ----------
        msg: PointCloud2
        """
        data = np.reshape(np.fromstring(msg.data, dtype=np.float32), [-1, 3])
        data = np.concatenate((-data[:, 1:2], data[:, 0:1], data[:, 2:3]), axis=1)
        msg.data = data.tostring()
        self.lidar_pub.publish(msg)


if __name__ == "__main__":
    node = CarlaImuRovioTransform()
    rospy.spin()
