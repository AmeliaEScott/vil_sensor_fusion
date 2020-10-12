# `gtsam_fusion` Package

This package represents the final fusion step from the thesis. It uses [GTSAM](https://gtsam.org)
to fuse odometry estimates from multiple sources.

There are four major components in this package:

- Graph Manager: Exposes an API for adding nodes and edges to the pose graph in a
  thread-safe way. See the documentation in `GraphManager.h`.
- Sensor Manager: One instance of `SensorManager` is responsible for one odometry source.
  Sensor manager instances listen to the relevant ROS topics, and reserve nodes and add
  between factors using the Graph Manager.
- IMU Manager: Maintains a buffer of IMU messages. When the Graph Manager needs to add an
  IMU factor to the graph, I queries the IMU manager for the preintegrated IMU measurements
  between the given time steps
- degenerate_odometry_filter: Uses the D-Optimality criterion to detect degenerate odometry.
  If degeneracy is detected, the message is discarded.
  
# Dependencies

The dependencies are listed in `package.xml`. However, there is one caveat: The version
of LOAM required is [this custom version and branch](https://github.com/ItsTimmy/cerberus_loam_dev/tree/feature/publish_covariance).
This fork has added a large number of customizations. The following are strictly required:

 - Covariance of ICP point cloud matching is published in odometry message
 - Hessian of ICP is published in `OptStatus` message