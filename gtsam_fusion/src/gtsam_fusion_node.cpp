
#include <ros/ros.h>
#include <gtsam_fusion/GraphManager.h>
#include <gtsam_fusion/SensorManagerRos.h>
#include <gtsam_fusion/ImuManagerRos.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

using namespace VILFusion;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gtsam_fusion");
    ros::NodeHandle nh(ros::this_node::getName());
    ros::NodeHandle imuHandle(nh, "imu");

    auto graphManager = std::make_shared<GraphManager>();
    std::vector<std::shared_ptr<SensorManagerRos>> sensorManagers;
    auto imuManager = std::make_shared<ImuManagerRos>(imuHandle, graphManager);

    XmlRpc::XmlRpcValue sensorsList;
    nh.getParam("sensors", sensorsList);
    for(auto & i : sensorsList)
    {
        ros::NodeHandle sensorHandle(nh, "sensors/" + i.first);
        std::string sensorType;
        sensorHandle.getParam("sensor_type", sensorType);
        ROS_INFO_STREAM("Found configuration for sensor " << i.first << ", in namespace " << sensorHandle.getNamespace());
        if(sensorType == "PointCloud2")
        {
            sensorManagers.emplace_back(std::make_shared<SensorManagerRos>(
                    graphManager, sensorHandle, sensor_msgs::PointCloud2()
            ));
        }
        else if(sensorType == "Image")
        {
            sensorManagers.emplace_back(std::make_shared<SensorManagerRos>(
                    graphManager, sensorHandle, sensor_msgs::Image()
            ));
        }
        else
        {
            ROS_WARN_STREAM("Sensor " << i.first << " has invalid type " << sensorType);
        }
    }

    auto odomPub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    graphManager->addOptimizationCallback([&odomPub](double time, gtsam::Pose3 &pose, gtsam::Velocity3 &vel, gtsam::imuBias::ConstantBias &bias){
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time(time);
        odom.header.frame_id = "gtsam_world_frame";
        odom.child_frame_id = "gtsam_odom_frame";

        odom.pose.pose.position.x = pose.translation().x();
        odom.pose.pose.position.y = pose.translation().y();
        odom.pose.pose.position.z = pose.translation().z();
        odom.pose.pose.orientation.w = pose.rotation().toQuaternion().w();
        odom.pose.pose.orientation.x = pose.rotation().toQuaternion().x();
        odom.pose.pose.orientation.y = pose.rotation().toQuaternion().y();
        odom.pose.pose.orientation.z = pose.rotation().toQuaternion().z();

        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        odomPub.publish(odom);
    });

    ROS_INFO("Done constructing stuff!");
    ros::spin();

    return 1;
}