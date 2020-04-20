#!/usr/bin/env python2

import carla
import rospy

node = rospy.init_node("carla_avoid_stoplights")

client = carla.Client("localhost", 2000)
world = client.get_world()

while not rospy.is_shutdown():
    try:
        actors = world.get_actors()
        ego_vehicle_actor = next(filter(lambda a: a.attributes.get('role_name') == 'ego_vehicle', actors).__iter__())
        break
    except StopIteration:
        world.wait_for_tick()

while not rospy.is_shutdown():
    if ego_vehicle_actor.is_at_traffic_light():
        traffic_light = ego_vehicle_actor.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
            # world.hud.notification("Traffic light changed! Good to go!")
            rospy.loginfo("Turned traffic light green")
            traffic_light.set_state(carla.TrafficLightState.Green)
