#!/usr/bin/env python

from __future__ import print_function, division
import rospy
import carla
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
#from tf import transformations
import math
import tf
from tf.transformations import quaternion_from_euler

class CarlaRosbridge:
    def __init__(self, node_name="carla_rosbridge"):
        rospy.init_node(node_name)

        self.base_freq = rospy.get_param("~base_freq")
        self.camera_period = rospy.get_param("~camera_period")
        self.lidar_period = rospy.get_param("~lidar_period")

        self.should_do_depth = rospy.get_param("~should_do_depth")
        self.camera_params = rospy.get_param("~camera_params")
        self.recording_length = rospy.get_param("~recording_length")

        self.carla_logfile = rospy.get_param("~carla_logfile")
        self.imu0_transform = rospy.get_param("~imu0_transform")
        self.camera_transform = rospy.get_param("~camera_transform")
        self.imu1_transform = rospy.get_param("~imu1_transform")
        self.lidar_transform = rospy.get_param("~lidar_transform")

        hostname = rospy.get_param("~carla_hostname", "localhost")
        port = rospy.get_param("~carla_port", 2000)

        self.pub_image = rospy.Publisher("~cam0/image_raw", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("~cam0/image_depth", Image, queue_size=1)
        self.pub_imu0 = rospy.Publisher("~imu0", Imu, queue_size=1)
        self.pub_imu1 = rospy.Publisher("~imu1", Imu, queue_size=1)

        self.pub_gt_pose = rospy.Publisher("~ground_truth/pose", PoseStamped, queue_size=1)
        self.pub_gt_twist = rospy.Publisher("~ground_truth/twist", TwistStamped, queue_size=1)
        #self.pub_gt_transform = rospy.Publisher("~ground_truth/transform", TransformStamped, queue_size=1)
        self.pub_gt_transform = tf.TransformBroadcaster()

        self.carla_client = carla.Client(hostname, port)
        #self.carla_client.set_timeout(15)

        print(self.carla_client.replay_file(self.carla_logfile, 0, self.recording_length, 0))

        #rospy.sleep(10)
        self.carla_world = self.carla_client.get_world()
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1 / self.base_freq
        self.carla_world.apply_settings(settings)

        # Need 1 tick to load all of the actors
        self.carla_world.tick()

        while True:
            try:
                actors = self.carla_world.get_actors()
                self.ego_vehicle_actor = next(filter(lambda a: a.attributes.get('role_name') == 'hero', actors).__iter__())
                break
            except StopIteration:
                self.carla_world.tick()
                print("No Hero vehicle found.")

        self.initial_transform = None
        self.carla_world.on_tick(self.tick_listener)

        self.rgb_camera = self.setup_camera()
        self.imu0 = self.setup_imu(self.imu0_transform, lambda imu_data: self.imu_listener(imu_data, self.pub_imu0))
        self.imu1 = self.setup_imu(self.imu1_transform, lambda imu_data: self.imu_listener(imu_data, self.pub_imu1))

        # if self.should_do_depth:
        #     blueprint: carla.ActorBlueprint = self.carla_world.get_blueprint_library().find("sensor.camera.depth")
        #     blueprint.set_attribute("image_size_x", "720")
        #     blueprint.set_attribute("image_size_y", "480")
        #     blueprint.set_attribute("fov", "110")
        #     blueprint.set_attribute("sensor_tick", "{:.4f}".format(self.camera_period / self.base_freq))
        #
        #     transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        #     self.depth_camera = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        #     self.depth_camera.listen(self.depth_camera_listener)

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

    @staticmethod
    def convert_carla_to_ros(t):
        location = (t.location.x, -t.location.y, t.location.z)
        rpy = (math.radians(t.rotation.roll), math.radians(-t.rotation.pitch), math.radians(-t.rotation.yaw))
        #print("Carla loc: {}, Ros loc: {}\nCarla rot: {}, Ros rot: {}".format((t.location.x, t.location.y, t.location.z), location, (t.rotation.roll, t.rotation.pitch, t.rotation.yaw), rpy))
        return location, tf.transformations.quaternion_from_euler(*rpy)

    # @staticmethod
    # def euler_to_quaternion(roll, pitch, yaw):
    #     """
    #     https://computergraphics.stackexchange.com/questions/8195/
    #
    #     All arguments in radians.
    #     """
    #     qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * \
    #          math.sin(pitch / 2) * math.sin(yaw / 2)
    #     qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * \
    #          math.cos(pitch / 2) * math.sin(yaw / 2)
    #     qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * \
    #          math.sin(pitch / 2) * math.cos(yaw / 2)
    #     qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * \
    #          math.sin(pitch / 2) * math.sin(yaw / 2)
    #     return [qx, qy, qz, qw]

    # @staticmethod
    # def convert_transform_to_carla(position: np.ndarray, orientation: np.ndarray):
    #     (roll, pitch, yaw) = transformations.euler_from_quaternion(orientation)
    #     location = carla.Location(x=position[0], y=position[1], z=position[2])
    #     rotation = carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
    #     return carla.Transform(location, rotation)
    #
    # @staticmethod
    # def convert_transform_to_ros(transform: carla.Transform):
    #     quaternion = transformations.quaternion_from_euler(
    #         transform.rotation.roll,
    #         transform.rotation.pitch,
    #         transform.rotation.yaw
    #     )
    #     position = np.ndarray([transform.location.x, transform.location.y, transform.location.z])
    #     return position, quaternion

    def setup_camera(self):
        blueprint = self.carla_world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", "{}".format(self.camera_params['width']))
        blueprint.set_attribute("image_size_y", "{}".format(self.camera_params['height']))
        blueprint.set_attribute("fov", "{}".format(self.camera_params['fov']))
        blueprint.set_attribute("sensor_tick", "{:.4f}".format(self.camera_period / self.base_freq))

        transform = self.convert_ros_to_carla(**self.camera_transform)
        rgb_camera = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        rgb_camera.listen(self.camera_listener)
        return rgb_camera

    def setup_imu(self, transform_param, listener):
        blueprint = self.carla_world.get_blueprint_library().find("sensor.other.imu")
        blueprint.set_attribute("sensor_tick", "{:.4f}".format(1 / self.base_freq))
        # TODO: Noise

        transform = self.convert_ros_to_carla(**transform_param)
        imu = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        imu.listen(listener)
        return imu

    def mainloop(self):
        while not rospy.is_shutdown():
            self.carla_world.tick()
            # TODO: Potentially wait for all sensors to be published?

    def tick_listener(self, event):
        print("TICK!")
        carla_pose = self.ego_vehicle_actor.get_transform()
        ros_location, ros_quaternion = self.convert_carla_to_ros(carla_pose)

        if self.initial_transform is None:
            self.initial_transform = (
                (-ros_location[0], -ros_location[1], -ros_location[2]),
                tf.transformations.quaternion_inverse(ros_quaternion)
            )

        self.pub_gt_transform.sendTransform(
            self.initial_transform[0],
            self.initial_transform[1],
            rospy.Time.from_sec(event.timestamp.elapsed_seconds),
            "/carla_world",
            "/world"
        )

        for pose, frame in zip([
            self.ego_vehicle_actor.get_transform(),
            self.imu0.get_transform(),
            self.imu1.get_transform(),
            self.rgb_camera.get_transform()
        ], [
            '/vehicle_gt',
            '/imu0_gt',
            '/imu1_gt',
            '/cam0_gt'
        ]):
            ros_location, ros_quaternion = self.convert_carla_to_ros(pose)
            self.pub_gt_transform.sendTransform(
                ros_location,
                ros_quaternion,
                rospy.Time.from_sec(event.timestamp.elapsed_seconds),
                frame,
                "/carla_world"
            )

    def camera_listener(self, img):
        msg = Image()
        msg.header.stamp = rospy.Time.from_sec(img.timestamp)
        msg.height = img.height
        msg.width = img.width
        msg.encoding = "bgra8"
        msg.step = msg.width * 4
        msg.data = list(bytearray(img.raw_data))
        self.pub_image.publish(msg)

    def depth_camera_listener(self, img):
        img.convert(carla, carla.ColorConverter.Depth)
        msg = Image()
        msg.header.stamp = rospy.Time.from_sec(img.timestamp)
        msg.height = img.height
        msg.width = img.width
        msg.encoding = "32FC1"
        msg.step = msg.width * 4
        msg.data = list(img.raw_data)
        self.pub_depth.publish(msg)

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
        msg.header.stamp = rospy.Time.from_sec(imu_data.timestamp)
        msg.header.frame_id = "/imu0_gt"
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
    node = CarlaRosbridge()
    node.mainloop()
