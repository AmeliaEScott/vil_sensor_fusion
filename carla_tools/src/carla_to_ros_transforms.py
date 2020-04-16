#!/usr/bin/env python3

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import Imu, PointCloud2
import numpy as np


class CarlaImuRovioTransform:
    def __init__(self, node_name="carla_to_ros_transforms"):
        rospy.init_node(node_name)
        self.imu_sub = rospy.Subscriber("~imu/carla", Imu, callback=self.imu_listener)
        self.rovio_pub = rospy.Publisher("~imu/rovio", Imu, queue_size=1)
        self.loam_pub = rospy.Publisher("~imu/loam", Imu, queue_size=1)
        self.ros_pub = rospy.Publisher("~imu/ros", Imu, queue_size=1)

        self.lidar_sub = rospy.Subscriber("~lidar_carla", PointCloud2, callback=self.lidar_callback)
        self.lidar_pub = rospy.Publisher("~lidar_loam", PointCloud2, queue_size=1)

    @staticmethod
    def transform_covariance(covariance, transform):
        cov = np.reshape(np.array(covariance), [3, 3])
        return np.matmul(transform, np.matmul(cov, np.transpose(transform)))

    @staticmethod
    def imu_transform(msg, transform):
        carla_accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        # Convert to right-handed rotation
        carla_gyro = -np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        ros_accel = np.matmul(transform, carla_accel)
        ros_accel_cov = CarlaImuRovioTransform.transform_covariance(msg.linear_acceleration_covariance, transform)
        ros_gyro = np.matmul(transform, carla_gyro)
        ros_gyro_cov = CarlaImuRovioTransform.transform_covariance(msg.angular_velocity_covariance, transform)
        ros_msg = Imu()
        ros_msg.header.stamp = msg.header.stamp
        ros_msg.linear_acceleration.x = ros_accel[0]
        ros_msg.linear_acceleration.y = ros_accel[1]
        ros_msg.linear_acceleration.z = ros_accel[2]
        ros_msg.linear_acceleration_covariance = np.reshape(ros_accel_cov, [-1])
        ros_msg.angular_velocity.x = ros_gyro[0]
        ros_msg.angular_velocity.y = ros_gyro[1]
        ros_msg.angular_velocity.z = ros_gyro[2]
        ros_msg.angular_velocity_covariance = np.reshape(ros_gyro_cov, [-1])
        ros_msg.orientation_covariance[0] = -1
        return ros_msg

    def imu_listener(self, carla_msg):
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

        carla_to_ros = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        ros_msg = self.imu_transform(carla_msg, carla_to_ros)
        self.ros_pub.publish(ros_msg)

        ros_to_rovio = np.array([
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0]
        ])
        rovio_msg = self.imu_transform(ros_msg, ros_to_rovio)
        self.rovio_pub.publish(rovio_msg)

        ros_to_loam = np.array([
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0]
        ])
        loam_msg = self.imu_transform(ros_msg, ros_to_loam)
        self.loam_pub.publish(loam_msg)

    def lidar_callback(self, msg):
        """
        Parameters
        ----------
        msg: PointCloud2
        """
        data = np.reshape(np.fromstring(msg.data, dtype=np.float32), [-1, 3])
        data = np.concatenate((-data[:, 1:2], data[:, 0:1], data[:, 2:3]), axis=1)
        #data = np.concatenate((data[:, 1:2], data[:, 2:3], data[:, 0:1]), axis=1)
        msg.data = data.tostring()
        self.lidar_pub.publish(msg)


if __name__ == "__main__":
    node = CarlaImuRovioTransform()
    rospy.spin()
