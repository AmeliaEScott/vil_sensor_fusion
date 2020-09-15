
#include <ros/ros.h>
#include <gtsam_fusion/GraphManager.h>
#include <gtsam_fusion/SensorManagerRos.h>
#include <gtsam_fusion/ImuManagerRos.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

using namespace VILFusion;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gtsam_fusion");
    ros::NodeHandle nh(ros::this_node::getName());

    std::vector<SensorManagerRos> sensorManagers;
    auto graphManager = std::make_shared<GraphManager>();

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
            sensorManagers.emplace_back(graphManager, sensorHandle, sensor_msgs::PointCloud2());
        }
        else if(sensorType == "Image")
        {
            sensorManagers.emplace_back(graphManager, sensorHandle, sensor_msgs::Image());
        }
        else
        {
            ROS_WARN_STREAM("Sensor " << i.first << " has invalid type " << sensorType);
        }
    }

    ROS_INFO("Done constructing stuff!");
    ros::spin();

    return 1;
}