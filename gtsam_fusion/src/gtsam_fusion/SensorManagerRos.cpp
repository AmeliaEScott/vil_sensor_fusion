
#include <gtsam_fusion/SensorManagerRos.h>


namespace VILFusion
{

    SensorManagerRos::SensorManagerRos(std::shared_ptr<GraphManager> graphManager, ros::NodeHandle &nodeHandle,
                                       const std::string &odometryTopic) :
           _graphManager(graphManager)
    {
        _odometrySubscriber = nodeHandle.subscribe(odometryTopic, 1, &SensorManagerRos::odometryCallback, this);
    }

    void SensorManagerRos::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        auto time = msg->header.stamp.toSec();
        gtsam::PoseBetweenFactor bf;
    }
}