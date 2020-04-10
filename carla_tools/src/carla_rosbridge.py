#!/usr/bin/env python3

import rospy
import carla
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
#from tf import transformations
import math

class CarlaRosbridge:
    def __init__(self, node_name="carla_rosbridge"):
        rospy.init_node(node_name)

        self.base_freq = rospy.get_param("~base_freq")
        self.camera_period = rospy.get_param("~camera_period")
        self.lidar_period = rospy.get_param("~lidar_period")

        self.should_do_depth = rospy.get_param("~should_do_depth")
        self.camera_params = rospy.get_param("~camera_params")

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

        self.carla_client = carla.Client(hostname, port)

        print(self.carla_client.replay_file(self.carla_logfile, 0, 30, 0))

        self.carla_world: carla.World = self.carla_client.get_world()
        settings = self.carla_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1 / self.base_freq
        self.carla_world.apply_settings(settings)

        # Need 1 tick to load all of the actors
        self.carla_world.tick()

        actors = self.carla_world.get_actors()
        self.ego_vehicle_actor = next(filter(lambda a: a.attributes.get('role_name') == 'hero', actors))

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
    def convert_param_to_carla(param):
        location = CarlaRosbridge.convert_ros_to_carla(param['x'], param['y'], param['z'])
        rotation = carla.Rotation(roll=param['roll'], pitch=param['pitch'], yaw=param['yaw'])
        return carla.Transform(location, rotation)

    @staticmethod
    def convert_ros_to_carla(x, y, z) -> carla.Location:
        return carla.Location(x=z, y=x, z=-y)

    @staticmethod
    def convert_carla_to_ros(x, y, z):
        return y, -z, x

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        https://computergraphics.stackexchange.com/questions/8195/

        All arguments in radians.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * \
             math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * \
             math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * \
             math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * \
             math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    # @staticmethod
    # def convert_transform_to_carla(position: np.ndarray, orientation: np.ndarray) -> carla.Transform:
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
        blueprint: carla.ActorBlueprint = self.carla_world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", "{}".format(self.camera_params['width']))
        blueprint.set_attribute("image_size_y", "{}".format(self.camera_params['height']))
        blueprint.set_attribute("fov", "{}".format(self.camera_params['fov']))
        blueprint.set_attribute("sensor_tick", "{:.4f}".format(self.camera_period / self.base_freq))

        transform = self.convert_param_to_carla(self.camera_transform)
        rgb_camera = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        rgb_camera.listen(self.camera_listener)
        return rgb_camera

    def setup_imu(self, transform_param, listener):
        blueprint: carla.ActorBlueprint = self.carla_world.get_blueprint_library().find("sensor.other.imu")
        blueprint.set_attribute("sensor_tick", "{:.4f}".format(1 / self.base_freq))
        # TODO: Noise

        transform = self.convert_param_to_carla(transform_param)
        imu = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        imu.listen(listener)
        return imu

    def mainloop(self):
        while not rospy.is_shutdown():
            self.carla_world.tick()
            # TODO: Potentially wait for all sensors to be published?

    def tick_listener(self, event: carla.WorldSnapshot):
        print("TICK!")
        carla_pose: carla.Transform = self.imu0.get_transform()
        carla_linear_velocity: carla.Vector3D = self.imu0.get_velocity()
        carla_angular_velocity: carla.Vector3D = self.imu0.get_angular_velocity()

        ros_location = self.convert_carla_to_ros(carla_pose.location.x, carla_pose.location.y, carla_pose.location.z)
        ros_rpy = (math.radians(carla_pose.rotation.roll), math.radians(carla_pose.rotation.pitch),
                   math.radians(carla_pose.rotation.yaw))
        ros_quaternion = self.euler_to_quaternion(*ros_rpy)
        ros_linear_velocity = self.convert_carla_to_ros(
            carla_linear_velocity.x, carla_linear_velocity.y, carla_linear_velocity.z)
        # Angular velocities need to be negated because Carla uses a left-hand coordinate system
        ros_angular_velocity = self.convert_carla_to_ros(
            -carla_angular_velocity.x, -carla_angular_velocity.y, -carla_angular_velocity.z)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.from_sec(event.timestamp.elapsed_seconds)
        pose_msg.header.frame_id = "/world"
        pose_msg.pose.position.x = ros_location[0]
        pose_msg.pose.position.y = ros_location[1]
        pose_msg.pose.position.z = ros_location[2]
        pose_msg.pose.orientation.x = ros_quaternion[0]
        pose_msg.pose.orientation.y = ros_quaternion[1]
        pose_msg.pose.orientation.z = ros_quaternion[2]
        pose_msg.pose.orientation.w = ros_quaternion[3]
        self.pub_gt_pose.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.from_sec(event.timestamp.elapsed_seconds)
        pose_msg.header.frame_id = "/world"
        twist_msg.twist.linear.x = ros_linear_velocity[0]
        twist_msg.twist.linear.y = ros_linear_velocity[1]
        twist_msg.twist.linear.z = ros_linear_velocity[2]
        twist_msg.twist.angular.x = ros_angular_velocity[0]
        twist_msg.twist.angular.y = ros_angular_velocity[1]
        twist_msg.twist.angular.z = ros_angular_velocity[2]
        self.pub_gt_twist.publish(twist_msg)

    def camera_listener(self, img: carla.Image):
        msg = Image()
        msg.header.stamp = rospy.Time.from_sec(img.timestamp)
        msg.height = img.height
        msg.width = img.width
        msg.encoding = "bgra8"
        msg.step = msg.width * 4
        msg.data = list(img.raw_data)
        self.pub_image.publish(msg)

    def depth_camera_listener(self, img: carla.Image):
        img.convert(carla, carla.ColorConverter.Depth)
        msg = Image()
        msg.header.stamp = rospy.Time.from_sec(img.timestamp)
        msg.height = img.height
        msg.width = img.width
        msg.encoding = "32FC1"
        msg.step = msg.width * 4
        msg.data = list(img.raw_data)
        self.pub_depth.publish(msg)

    def imu_listener(self, imu_data: carla.IMUMeasurement, publisher: rospy.Publisher):
        msg = Imu()
        msg.header.stamp = rospy.Time.from_sec(imu_data.timestamp)
        accel_x, accel_y, accel_z = self.convert_carla_to_ros(
            imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z
        )
        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z
        msg.linear_acceleration_covariance[0] = -1

        # Need to negate these because Carla is left-handed, but ROS/Rovio expect right-handed
        gyro_x, gyro_y, gyro_z = self.convert_carla_to_ros(
            -imu_data.gyroscope.x, -imu_data.gyroscope.y, -imu_data.gyroscope.z
        )
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        msg.angular_velocity_covariance[0] = -1

        msg.orientation.x = -1
        msg.orientation_covariance[0] = -1

        publisher.publish(msg)


if __name__ == "__main__":
    node = CarlaRosbridge()
    node.mainloop()
