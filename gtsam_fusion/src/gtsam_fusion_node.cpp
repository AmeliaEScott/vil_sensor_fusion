

#include <gtsam_fusion/GraphManager.h>
#include <gtsam_fusion/SensorManagerRos.h>
#include <gtsam_fusion/ImuManagerRos.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

using namespace VILFusion;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gtsam_fusion");

    ros::NodeHandle nh(ros::this_node::getName());
    ros::NodeHandle imuHandle(nh, "imu");

//    ros::Duration d(5.0);
//    d.sleep();

    auto imuManager = std::make_shared<ImuManagerRos>(imuHandle);
    auto graphManager = std::make_shared<GraphManager>(imuManager);
    std::vector<std::shared_ptr<SensorManagerRos>> sensorManagers;


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
    tf2_ros::TransformBroadcaster broadcaster;
    std::string staticFrame, odomFrame;
    nh.getParam("tf/static_frame", staticFrame);
    nh.getParam("tf/odom_frame", odomFrame);

    graphManager->addOptimizationCallback(
    [&odomPub, &broadcaster, &staticFrame, &odomFrame](double time, gtsam::Pose3 &pose, gtsam::Velocity3 &vel, gtsam::imuBias::ConstantBias &bias){
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time(time);
        odom.header.frame_id = staticFrame;
        odom.child_frame_id = odomFrame;

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

        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = staticFrame;
        transform.header.stamp = ros::Time(time);
        transform.child_frame_id = odomFrame;

        transform.transform.translation.x = pose.translation().x();
        transform.transform.translation.y = pose.translation().y();
        transform.transform.translation.z = pose.translation().z();
        transform.transform.rotation.x = pose.rotation().toQuaternion().x();
        transform.transform.rotation.y = pose.rotation().toQuaternion().y();
        transform.transform.rotation.z = pose.rotation().toQuaternion().z();
        transform.transform.rotation.w = pose.rotation().toQuaternion().w();
        broadcaster.sendTransform(transform);
    });

    ROS_INFO("Done constructing stuff!");
    ros::spin();

    return 1;
}