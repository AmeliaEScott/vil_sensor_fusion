
#include "VILConfusorNode.h"

#include <ros/console.h>

namespace vil_fusion {

    VILConfusorNode::VILConfusorNode(ros::NodeHandle &node) :
        _node(node),
        _pose_pub(_node.advertise<geometry_msgs::PoseWithCovarianceStamped>("fused_pose", 1))
    {
        _imu_sub = _node.subscribe("imu", 10, &VILConfusorNode::imuCallback, this);
        _loam_sub = _node.subscribe("loam_odometry", 10, &VILConfusorNode::loamOdometryCallback, this);
        _rovio_sub = _node.subscribe("rovio_odometry", 10, &VILConfusorNode::rovioOdometryCallback, this);
    }

    VILConfusorNode::~VILConfusorNode() {

    }

    void VILConfusorNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        ROS_INFO("Got IMU message");
    }

    void VILConfusorNode::loamOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        ROS_INFO("Got LOAM odometry");
    }

    void VILConfusorNode::rovioOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        ROS_INFO("Got Rovio odometry");
    }

    void VILConfusorNode::commonOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        ROS_INFO("Common odometry callback");
    }

}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "vil_fusion_node");

    ros::NodeHandle nh("vil_fusion_node");

    vil_fusion::VILConfusorNode node(nh);

    ros::spin();
}