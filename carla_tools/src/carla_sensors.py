#!/usr/bin/env python2

from __future__ import print_function, division
import rospy
import carla
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
import math
import tf
from tf.transformations import quaternion_from_euler

class CarlaSensors:
    def __init__(self, node_name="carla_sensors"):
        rospy.init_node(node_name)

        self.base_freq = rospy.get_param("/carla/fixed_delta_seconds")
        self.camera_period = rospy.get_param("/carla_sensors/camera_period")
        self.lidar_period = rospy.get_param("/carla_sensors/lidar_period")

        self.should_do_depth = rospy.get_param("/carla_sensors/should_do_depth")
        self.camera_params = rospy.get_param("/carla_sensors/camera_params")
        self.recording_length = rospy.get_param("/carla_sensors/recording_length")

        self.carla_logfile = rospy.get_param("carla_log")
        self.imu0_transform = rospy.get_param("/carla_sensors/imu0_transform")
        self.camera_transform = rospy.get_param("/carla_sensors/camera_transform")
        self.imu1_transform = rospy.get_param("/carla_sensors/imu1_transform")
        self.lidar_transform = rospy.get_param("/carla_sensors/lidar_transform")

        self.pub_imu0 = rospy.Publisher("~imu0", Imu, queue_size=1)
        self.pub_imu1 = rospy.Publisher("~imu1", Imu, queue_size=1)

        hostname = rospy.get_param("/carla/host", "localhost")
        port = rospy.get_param("/carla/port", 2000)

        self.carla_client = carla.Client(hostname, port)

        print(self.carla_client.replay_file(self.carla_logfile, 0, self.recording_length, 0))

        self.carla_world = self.carla_client.get_world()

        while True:
            try:
                actors = self.carla_world.get_actors()
                self.ego_vehicle_actor = next(
                    filter(
                        lambda a: a.attributes.get('role_name') == 'hero',
                        actors
                    ).__iter__())
                break
            except StopIteration:
                self.carla_world.wait_for_tick()
                print("No Hero vehicle found. Trying again in one tick...")

        # self.pose_offset = None
        # self.orientation_offset = None
        # self.carla_world.on_tick(self.tick_listener)

        self.rgb_camera = self.setup_camera()
        self.imu0 = self.setup_imu(self.imu0_transform, lambda imu_data: self.imu_listener(imu_data, self.pub_imu0))
        #self.imu1 = self.setup_imu(self.imu1_transform, lambda imu_data: self.imu_listener(imu_data, self.pub_imu1))

        #self.imu0_subscriber = rospy.Subscriber("")

    @staticmethod
    def convert_ros_to_carla(x, y, z, roll, pitch, yaw):
        """
        The Carla convention is left-handed:
         x: Forward
         y: Right
         z: Up

        The ROS convention is right-handed:
         x: Forward
         y: Left
         z: Up
        """
        return carla.Transform(carla.Location(x, -y, z), carla.Rotation(roll, -pitch, -yaw))

    def setup_camera(self):
        blueprint = self.carla_world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", "{}".format(self.camera_params['width']))
        blueprint.set_attribute("image_size_y", "{}".format(self.camera_params['height']))
        blueprint.set_attribute("fov", "{}".format(self.camera_params['fov']))
        blueprint.set_attribute("sensor_tick", "{:.4f}".format(self.camera_period / self.base_freq))
        blueprint.set_attribute("role_name", "myspecialboy")

        transform = self.convert_ros_to_carla(**self.camera_transform)
        rgb_camera = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        #rgb_camera.listen(self.camera_listener)
        return rgb_camera

    def setup_imu(self, transform_param, listener):
        blueprint = self.carla_world.get_blueprint_library().find("sensor.other.imu")
        blueprint.set_attribute("sensor_tick", "{:.4f}".format(1 / self.base_freq))
        # TODO: Noise

        transform = self.convert_ros_to_carla(**transform_param)
        #imu = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        imu = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.rgb_camera)
        #imu.listen(listener)
        return imu

    def imu_listener(self, imu_data, publisher):
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
        msg = Imu()
        # msg.header.stamp = imu_msg.header
        # accel = (imu_msg.linear_acceleration.y, -imu_msg.linear_acceleration.z, imu_msg.linear_acceleration.x)
        # gyro = (-imu_msg.angular_velocity.y, imu_msg.angular_velocity.z, -imu_msg.angular_velocity.x)
        msg.header.stamp = rospy.Time.from_sec(imu_data.timestamp)
        accel = (imu_data.accelerometer.y, -imu_data.accelerometer.z, imu_data.accelerometer.x)
        gyro = (-imu_data.gyroscope.y, imu_data.gyroscope.z, -imu_data.gyroscope.x)
        # Convert IMU axes from ROS convention to Rovio convention
        # accel = (-accel[1], -accel[2], accel[0])
        # gyro = (-gyro[1], -gyro[2], gyro[0])

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        msg.linear_acceleration_covariance[0] = -1

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance[0] = -1

        msg.orientation.x = -1
        msg.orientation_covariance[0] = -1

        publisher.publish(msg)

if __name__ == "__main__":
    node = CarlaSensors()
    rospy.spin()
