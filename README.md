### See the README files in `carla_tools` and `vil_fusion` for more information.

# Dependencies

The following packages should all exist in your `catkin_ws/src`:

 - [RSL Rovio Fork, branch `tim-experimental`](https://bitbucket.org/leggedrobotics/rovio/src/tim-experimental/)
 - [RSL LOAM-Velodyne Fork, branch `tim_experimental`](https://bitbucket.org/leggedrobotics/loam_velodyne/src/tim-experimental/)
 - [Image pipeline](https://github.com/ros-perception/image_pipeline)
 - [kindr](https://github.com/ANYbotics/kindr)
 - [Carla ROS Bridge, Fork with my IMU fix PR](https://github.com/ItsTimmy/carla-ros-bridge/tree/imu-frame-fix)
     - If this PR has been merged to master, then you can probably just use the `master` branch from the main repo.
 - confusion_dev (Ask Tim Sandy for read access)
 - [smb_confusor](https://bitbucket.org/leggedrobotics/smb_confusor/src/master/)

Additionally, you will need a version of PCL built from source. 
See [here](https://github.com/laboshinl/loam_velodyne/issues/71) for more information.

[This repo](https://bitbucket.org/scottti/vil_sensor_fusion_workspace/src) is a full Catkin workspace with all of
the necessary dependencies (and this package) as submodules. Just run this command to get it all:

    git clone --recursive git@bitbucket.org:scottti/vil_sensor_fusion_workspace.git

There is more information in that repository about getting the dependencies configured.
