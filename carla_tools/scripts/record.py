import carla
import os
import sys

log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "carla_logs")

log_file = os.path.join(log_dir, sys.argv[1])

client = carla.Client("localhost", 2000)

world = client.get_world()
settings = world.get_settings()
settings.fixed_delta_seconds = 0.005
world.apply_settings(settings)

client.start_recorder(log_file)

input("Press enter to stop recording. ")

client.stop_recorder()
settings.fixed_delta_seconds = 0.0
world.apply_settings(settings)