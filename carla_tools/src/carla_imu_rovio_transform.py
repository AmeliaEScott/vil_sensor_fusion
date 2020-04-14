#!/usr/bin/env python3

from __future__ import print_function, division
import rospy
from sensor_msgs.msg import Imu


class CarlaImuRovioTransform:
    def __init__(self, node_name="carla_imu_rovio_transform"):
        rospy.init_node(node_name)
        self.imu_sub = rospy.Subscriber("~imu_carla", Imu, callback=self.imu_listener)
        self.imu_pub = rospy.Publisher("~imu_rovio", Imu, queue_size=1)

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
        """
        accel = (msg.linear_acceleration.y, -msg.linear_acceleration.z, msg.linear_acceleration.x)
        gyro = (-msg.angular_velocity.y, msg.angular_velocity.z, -msg.angular_velocity.x)
        # Convert IMU axes from ROS convention to Rovio convention
        # accel = (-accel[1], -accel[2], accel[0])
        # gyro = (-gyro[1], -gyro[2], gyro[0])

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        self.imu_pub.publish(msg)


if __name__ == "__main__":
    node = CarlaImuRovioTransform()
    rospy.spin()
