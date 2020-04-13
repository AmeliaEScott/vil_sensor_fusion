#!/usr/bin/env python3

import os
import time
import rospy
import carla
import signal
import subprocess
import argparse
import threading

carla_executable = [
    "/opt/carla/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping",
    "CarlaUE4",
    "-opengl"
]

parser = argparse.ArgumentParser()
parser.add_argument('--quality', type=str, required=False, choices=["none", "low", "epic"], default="epic")
parser.add_argument("--output", type=str, required=True, help="File to which the simulation data will be recorded")

args = parser.parse_args()

if args.quality in ["none", "low"]:
    carla_executable += ["-quality-level=Low"]
else:
    carla_executable += ["-quality-level=Epic"]

if args.quality == "none":
    control_executable = "no_rendering_mode.py"
else:
    control_executable = "manual_control.py"

if not args.output.startswith("/"):
    args.output = os.path.join(os.getcwd(), args.output)

control_executable = [
    "python3",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "carla_controllers", control_executable),
]

carla_server_proc = subprocess.Popen(
    carla_executable,
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)
time.sleep(2)



control_proc = subprocess.Popen(
    control_executable
)

time.sleep(2)

# RECORD
client = carla.Client("localhost", 2000)
world = client.get_world()
settings = world.get_settings()
settings.fixed_delta_seconds = 0.004
world.apply_settings(settings)

while True:
    try:
        actors = world.get_actors()
        ego_vehicle_actor = next(filter(lambda a: a.attributes.get('role_name') == 'hero', actors).__iter__())
        break
    except StopIteration:
        print("Waiting for hero vehicle to spawn...")
        time.sleep(1)

client.start_recorder(args.output)

control_proc.wait()
client.stop_recorder()

carla_server_proc.send_signal(signal.SIGINT)
carla_server_proc.wait()

# TODO: Convert to ROS
# This will involve stopping Carla and restarting, if the quality is low
