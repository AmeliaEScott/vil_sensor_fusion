#ifndef VIL_SENSOR_FUSION_VILCONFUSORNODE_H
#define VIL_SENSOR_FUSION_VILCONFUSORNODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

//#include "ImuState.h"

namespace vil_fusion {

    class VILConfusorNode {
    public:
        VILConfusorNode(ros::NodeHandle &node);
        ~VILConfusorNode();

    private:
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void loamOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void rovioOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void commonOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

        ros::NodeHandle &_node;
        ros::Subscriber _imu_sub;
        ros::Subscriber _loam_sub;
        ros::Subscriber _rovio_sub;
        ros::Publisher _pose_pub;
    };

}

#endif //VIL_SENSOR_FUSION_VILCONFUSORNODE_H
