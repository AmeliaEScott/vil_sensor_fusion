
## Dependencies

The following packages should all exist in your `catkin_ws/src`:

 - [Rovio](https://github.com/ethz-asl/rovio)
 - [LOAM-Velodyne](https://github.com/laboshinl/loam_velodyne)
 - [Image pipeline](https://github.com/ros-perception/image_pipeline)
 - [kindr](https://github.com/ANYbotics/kindr)
 - [Carla ROS Bridge](https://github.com/carla-simulator/ros-bridge)

Additionally, you will need a version of PCL built from source. 
See [here](https://github.com/laboshinl/loam_velodyne/issues/71) for more information.

[This repo](https://bitbucket.org/scottti/vil_sensor_fusion_workspace/src) is a full Catkin workspace with all of
the necessary dependencies (and this package) as submodules. Just run this command to get it all:

    git clone --recursive git@bitbucket.org:scottti/vil_sensor_fusion_workspace.git

There is more information in that repository about getting the dependencies configured.

## Package carla_tools

This package is a set of tools for getting data from [Carla](https://carla.readthedocs.io/en/latest/) into
a ROS bag.

#### To run a simulation:

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
 
#### Configuration

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

## Package vil_fusion

This package is for running experiments on ROS bags. To do so:

 - `roslaunch vil_fusion vil_fusion_bag.launch bagfile:=<Path to ROS bag>`
   - `rovio:=false` to disable Rovio
   - `loam:=false` to disable Loam
   - `rviz:=true` to enable the Rviz configuration included with Loam
   - In the future, I will include an Rviz configuration that displays both Rovio and Loam odometry in a convenient way.

I have included a few sample ROS bags in `vil_fusion/sample_bags/`. These are stored with Git LFS, so you will need
to have Git LFS installed:

    git lfs install

Everything else should work normally after that. If you installed Git LFS only after cloning the repo, you can
run `git lfs fetch` to make sure you have all of the important files.