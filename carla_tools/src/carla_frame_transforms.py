#!/usr/bin/env python2

from __future__ import division
import rospy
import tf.transformations
from tf import Transformer, TransformBroadcaster, LookupException, TransformListener
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Header
import numpy as np
import transform_helper


class CarlaFrameTransformer:

    def __init__(self, node_name="carla_frame_transformer"):
        rospy.init_node(node_name)

        self.tf_listener = TransformListener(False)
        self.tf_broadcaster = TransformBroadcaster()

        self.init_imu_transform = None
        self.init_lidar_transform = None

        while not rospy.is_shutdown() and (self.init_imu_transform is None or self.init_lidar_transform is None):
            try:
                # Initial transform between Carla fixed /map frame and Carla imu and lidar
                self.init_imu_transform = self.tf_listener.lookupTransform(
                    "/map", "/ego_vehicle/imu", time=rospy.Time())
                self.init_lidar_transform = self.tf_listener.lookupTransform(
                    "/map", "/ego_vehicle/lidar/lidar1", time=rospy.Time())
            except LookupException:
                # Transforms from Carla are not available yet
                rospy.sleep(rospy.Duration.from_sec(0.1))

        self.send_transforms()
        #self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.send_transforms)

        # Initialize subscriber after we know that the transforms are available
        self.pub_imu_ros = rospy.Publisher("~imu/ros", Imu, queue_size=1)
        self.pub_imu_loam = rospy.Publisher("~imu/loam", Imu, queue_size=1)
        self.pub_imu_rovio = rospy.Publisher("~imu/rovio", Imu, queue_size=1)
        self.sub_imu = rospy.Subscriber("~imu/carla", Imu, callback=self.imu_callback)

        self.pub_lidar_loam = rospy.Publisher("~lidar/loam", PointCloud2, queue_size=1)
        self.pub_lidar_velodyne = rospy.Publisher("~lidar/velodyne", PointCloud2, queue_size=1)
        self.sub_lidar = rospy.Subscriber("~lidar/carla", PointCloud2, callback=self.lidar_callback)

    def send_transforms(self, *args):
        time = self.tf_listener.getLatestCommonTime("/ego_vehicle/imu", "/map")

        # Transformation between Carla's fixed map frame and LOAM's fixed camera_init frame, but rotated to be
        # in the ROS convention
        self.tf_broadcaster.sendTransform(
            self.init_lidar_transform[0],
            self.init_lidar_transform[1],
            time=time,
            parent="/map",
            child="/loam_init/ros_convention"
        )

        # Rotation from LOAM init frame in ROS convention, to LOAM init frame in LOAM convention
        # ros_to_loam_translate = tf.transformations.translation_from_matrix(transform_helper.ros_to_loam)
        ros_to_loam_translate = np.array([0.0, 0.0, 0.0])
        ros_to_loam_quat = tf.transformations.quaternion_from_matrix(transform_helper.ros_to_loam)
        self.tf_broadcaster.sendTransform(
            ros_to_loam_translate,
            ros_to_loam_quat,
            time=time,
            parent="/loam_init/ros_convention",
            child="/camera_init"
        )

        # Rotation from Carla's moving IMU frame to LOAM's moving IMU frame
        self.tf_broadcaster.sendTransform(
            ros_to_loam_translate,
            ros_to_loam_quat,
            time=time,
            parent="/ego_vehicle/imu",
            child="/imu/loam"
        )

        # Rotation from Carla's moving Lidar frame to LOAM's moving Lidar frame
        #  Note: This will probably go unused, because LOAM takes point clouds in the Velodyne frame. But I am leaving
        #  it here in case it is needed in the future.
        self.tf_broadcaster.sendTransform(
            ros_to_loam_translate,
            ros_to_loam_quat,
            time=time,
            parent="/ego_vehicle/lidar/lidar1",
            child="/lidar/loam"
        )

        # Rotation from Carla's moving Lidar frame to Velodyne's moving Lidar frame
        ros_to_velodyne_translate = np.array([0.0, 0.0, 0.0])
        ros_to_velodyne_quat = tf.transformations.quaternion_from_matrix(transform_helper.ros_to_velodyne)
        self.tf_broadcaster.sendTransform(
            ros_to_velodyne_translate,
            ros_to_velodyne_quat,
            time=time,
            parent="/ego_vehicle/lidar/lidar1",
            child="/lidar/velodyne"
        )

        # Transformation between Carla's fixed map frame and Rovio's fixed world frame, but rotated to be
        # in the ROS convention
        self.tf_broadcaster.sendTransform(
            self.init_imu_transform[0],
            self.init_imu_transform[1],
            time=time,
            parent="/map",
            child="/rovio_init/ros_convention"
        )

        ros_to_rovio_world_translate = np.array([0.0, 0.0, 0.0])
        ros_to_rovio_world_quat = tf.transformations.quaternion_from_matrix(transform_helper.ros_to_rovio_world)

        self.tf_broadcaster.sendTransform(
            ros_to_rovio_world_translate,
            ros_to_rovio_world_quat,
            time=time,
            parent="/rovio_init/ros_convention",
            child="/world"
        )

        # Rotation from Rovio init frame in ROS convention, to Rovio world frame in Rovio convention
        ros_to_rovio_translate = np.array([0.0, 0.0, 0.0])
        ros_to_rovio_quat = tf.transformations.quaternion_from_matrix(transform_helper.ros_to_rovio)

        # Rotation from Carla's moving IMU frame to Rovio's moving IMU frame
        self.tf_broadcaster.sendTransform(
            ros_to_rovio_translate,
            ros_to_rovio_quat,
            time=time,
            parent="/ego_vehicle/imu",
            child="/imu/rovio"
        )

    def transform_imu(self, msg, target):
        _, rotation = self.tf_listener.lookupTransform(target, msg.header.frame_id, rospy.Time())
        new_msg = transform_helper.transform_imu(msg, rotation)
        new_msg.header.frame_id = target
        return new_msg

    def imu_callback(self, msg):
        self.send_transforms()

        # Message comes in Carla left-handed convention. First step: Convert to right-handed
        msg.angular_velocity.x *= -1
        # msg.angular_velocity.y *= -1
        msg.angular_velocity.z *= -1
        msg.linear_acceleration.y *= -1

        msg_ros = msg
        self.pub_imu_ros.publish(msg_ros)

        self.pub_imu_loam.publish(self.transform_imu(msg_ros, "/imu/loam"))
        self.pub_imu_rovio.publish(self.transform_imu(msg_ros, "/imu/rovio"))

    def lidar_callback(self, msg):
        fake_header = Header()
        fake_header.stamp = rospy.Time()
        fake_header.frame_id = msg.header.frame_id

        transform = self.tf_listener.asMatrix("/lidar/loam", fake_header)
        msg_loam = transform_helper.transform_pointcloud2(msg, transform)
        msg_loam.header.frame_id = "/lidar/loam"
        self.pub_lidar_loam.publish(msg_loam)

        transform = self.tf_listener.asMatrix("/lidar/velodyne", fake_header)
        msg_velodyne = transform_helper.transform_pointcloud2(msg, transform)
        msg_velodyne.header.frame_id = "/lidar/velodyne"
        self.pub_lidar_velodyne.publish(msg_velodyne)


if __name__ == "__main__":
    node = CarlaFrameTransformer()
    rospy.spin()
