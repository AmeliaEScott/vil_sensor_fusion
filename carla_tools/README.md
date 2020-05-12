# Package `carla_tools`

This package is a set of tools for getting data from [Carla](https://carla.readthedocs.io/en/latest/) into
a ROS bag.

Strictly speaking, you could configure the simulation to run simultaneously with the sensor fusion experiments,
but the simulation generally already runs at ~0.1x speed on my computer, so it is only practical to record
the simulation data first, then replay it at full speed for the sensor fusion algorithms.

## To run a simulation:

The following commands are all run within `carla_tools` as the working directory.

 1. Launch the Carla simulator
   - `./opt/carla/bin/CarlaUE4.sh`
   - This is only the simulator server. It provides a spectator camera, but does not set up a vehicle or sensors
     or anything else. That is all done by the ROS bridge in the next step.
 2. `roslaunch carla_tools carla_ros_bridge.launch bag_name:=data`
   - This will perform a few important steps:
     - Spawn a vehicle with sensors into Carla
     - Open up a window for manual control of that vehicle
     - Store the resulting data in `carla_tools/rosbags/data.bag`
   - To control the vehicle:
     - Use WASD keys for manual control
     - For autopilot, first disable manual control with `B`, then enable autopilot with `P`.
     - For help, press `H`.
   - Note: This launch file is configured to not start recording the ROS bag until 20 seconds after launch.
     This is because the vehicle starts out in the simulation a few meters above the ground, and that freefall at the
     start was causing Rovio to diverge. To adjust this, use the parameter `bag_start_delay:=<n>`, where `<n>` is
     a number of seconds.
 3. `./scripts/fix_rosbag_time.py rosbags/data.bag`
   - This will change the timing of the ROS bag so that all of the messages are published in real time. The results
     are stored in a new bag in the same directory, with `_timefixed` appended. (Carla tends
     to run slower than real time, but each message has a simulation time-stamp, so it is easy to speed it back
     up to real time.)
 
## Configuration

To configure the Carla simulation (Speed, sensors, etc.), see `carla_tools/config`.

 - `carla_ros_bridge_settings.yaml` configures the Carla ROS bridge
 - `sensors.json` configures which sensors are included in the simulation. 
   - I have not yet figured out how to include multiple separate IMUs.
   - All coordinates are in the left-handed Carla frame:
     - X forward
     - Y right
     - Z up
   - If you change the camera parameters, you will need to reconfigure Rovio.
     - While a simulation is running, get the `camera_info`:
       - `rostopic echo /carla/ego_vehicle/camera/rgb/front/camera_info`
     - Copy the value of `camera_info/K` into `vil_fusion/cfg/carla.yaml/camera_matrix`
   - If you move around any sensors, you will need to reconfigure the Rovio extrinsic parameters
     in `vil_fusion/cfg/rovio.info`