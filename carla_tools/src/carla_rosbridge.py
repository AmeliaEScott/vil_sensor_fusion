#!/usr/bin/env python3

import rospy
import carla
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu


class CarlaRosbridge:
    def __init__(self, node_name="carla_rosbridge"):
        rospy.init_node(node_name)

        self.base_freq = rospy.get_param("~base_freq", 200)
        self.camera_period = rospy.get_param("~camera_period", 10)
        self.lidar_period = rospy.get_param("~lidar_period", 20)

        self.carla_logfile = rospy.get_param("~carla_logfile")

        hostname = rospy.get_param("~carla_hostname", "localhost")
        port = rospy.get_param("~carla_port", 2000)

        self.pub_image = rospy.Publisher("~cam0/image_raw", Image, queue_size=1)
        self.pub_depth = rospy.Publisher("~cam0/image_depth", Image, queue_size=1)

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

        blueprint: carla.ActorBlueprint = self.carla_world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", "720")
        blueprint.set_attribute("image_size_y", "480")
        blueprint.set_attribute("fov", "110")
        blueprint.set_attribute("sensor_tick", "{:.4f}".format(self.camera_period / self.base_freq))

        transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        self.rgb_camera = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        self.rgb_camera.listen(self.camera_listener)

        blueprint: carla.ActorBlueprint = self.carla_world.get_blueprint_library().find("sensor.camera.depth")
        blueprint.set_attribute("image_size_x", "720")
        blueprint.set_attribute("image_size_y", "480")
        blueprint.set_attribute("fov", "110")
        blueprint.set_attribute("sensor_tick", "{:.4f}".format(self.camera_period / self.base_freq))

        transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        self.depth_camera = self.carla_world.spawn_actor(blueprint, transform, attach_to=self.ego_vehicle_actor)
        self.depth_camera.listen(self.depth_camera_listener)

    def mainloop(self):
        while not rospy.is_shutdown():
            self.carla_world.tick()
            rospy.loginfo("Tick...")

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


if __name__ == "__main__":
    node = CarlaRosbridge()
    node.mainloop()
