#pragma once

#include <gtsam_fusion/GraphManager.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace VILFusion
{

    class SensorManagerRos
    {
    public:
        // Implemented in header because it's a template
        template<typename SensorType>
        static SensorManagerRos Construct(std::shared_ptr<GraphManager> graphManager, ros::NodeHandle &nodeHandle,
                                  const std::string &sensorTopic, const std::string &odometryTopic)
        {
            SensorManagerRos manager;
            manager._sensorSubscriber = nodeHandle.subscribe(sensorTopic, 1, &SensorManagerRos::sensorCallback<SensorType>, &manager);
            manager._odometrySubscriber = nodeHandle.subscribe(odometryTopic, 1, &SensorManagerRos::odometryCallback, &manager);
        };

    protected:
        ros::Subscriber _sensorSubscriber;
        ros::Subscriber _odometrySubscriber;

        SensorManagerRos();

        // Implemented in header because it's a template.
        template<class SensorType>
        void sensorCallback(const boost::shared_ptr<SensorType> &msg)
        {
            auto time = msg->header.stamp.toSec();
        };
        void odometryCallback(const nav_msgs::Odometry::ConstPtr &ms);
    }; // class SensorManagerRos

} // namespace VILFusion