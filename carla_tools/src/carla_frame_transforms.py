#!/usr/bin/env python2

from __future__ import division
import rospy
import tf.transformations
from tf import Transformer, TransformBroadcaster, LookupException, TransformListener
from sensor_msgs.msg import Imu, PointCloud2
import numpy as np
import transform_helper


class CarlaFrameTransformer:

    def __init__(self, node_name="carla_frame_transformer"):
        rospy.init_node(node_name)

        self.tf_listener = TransformListener(False)
        self.tf_broadcaster = TransformBroadcaster()

        self.imu_names = rospy.get_param("~imu_names")

        self.init_imu_transform = None
        self.init_lidar_transform = None

        self.carla_map_frame_id = "/map"

        self.rovio_world_frame_id = rospy.get_param("/rovio/world_frame", "rovio_world")
        self.do_loam = rospy.get_param("~do_loam", True)

        self.loam_init_frame_id = "loam_init"

        while not rospy.is_shutdown() and (self.init_imu_transform is None or self.init_lidar_transform is None):
            try:
                # Initial transform between Carla fixed /map frame and Carla imu and lidar
                self.init_imu_transform = self.tf_listener.lookupTransform(
                    self.carla_map_frame_id, "/ego_vehicle/imu/imu_vio", time=rospy.Time())
                if self.do_loam:
                    self.init_lidar_transform = self.tf_listener.lookupTransform(
                        self.carla_map_frame_id, "/ego_vehicle/lidar/lidar1", time=rospy.Time())
                else:
                    self.init_lidar_transform = self.init_imu_transform
            except LookupException:
                # Transforms from Carla are not available yet
                rospy.sleep(rospy.Duration.from_sec(0.1))

        self.send_transforms()

        # IMU send message on every frame of Carla simulation.
        # I want to send these transforms on every frame of Carla simulation.
        # I am too lazy to figure out the "correct" way to do that.
        self.dummy_imu_sub = rospy.Subscriber("/carla/ego_vehicle/imu/{}".format(self.imu_names[0]),
                                              Imu, self.send_transforms)

    def send_transforms(self, *args):
        time = self.tf_listener.getLatestCommonTime("/ego_vehicle/imu/imu_vio", self.carla_map_frame_id)

        if self.do_loam:
            # Transformation between Carla's fixed map frame and LOAM's fixed init frame, but rotated to be
            # in the ROS convention
            self.tf_broadcaster.sendTransform(
                self.init_lidar_transform[0],
                self.init_lidar_transform[1],
                time=time,
                parent=self.carla_map_frame_id,
                child="loam_init_ros_convention"
            )

            # Rotation from LOAM init frame in ROS convention, to LOAM init frame in LOAM convention
            # ros_to_loam_translate = tf.transformations.translation_from_matrix(transform_helper.ros_to_loam)
            ros_to_loam_translate = np.array([0.0, 0.0, 0.0])
            ros_to_loam_quat = tf.transformations.quaternion_from_matrix(transform_helper.ros_to_loam)
            self.tf_broadcaster.sendTransform(
                ros_to_loam_translate,
                ros_to_loam_quat,
                time=time,
                parent="loam_init_ros_convention",
                child=self.loam_init_frame_id
            )

        self.tf_broadcaster.sendTransform(
            self.init_imu_transform[0],
            self.init_imu_transform[1],
            time=time,
            parent=self.carla_map_frame_id,
            child=self.rovio_world_frame_id
        )


if __name__ == "__main__":
    node = CarlaFrameTransformer()
    rospy.spin()
