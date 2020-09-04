#pragma once

#include <gtsam_fusion/GraphManager.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace VILFusion
{

    class SensorManagerRos
    {
    public:

        /**
         * The template is necessary because SensorManager needs to work with any sensor message type which has a
         * Header, but ROS in C++ requires the type of a topic to be specified at compile time.
         *
         * The `dummy` parameter is needed because, in C++, you cannot explicitly specify template parameters when using
         * a constructor; this can only be done through inference, based on arguments to the constructor.
         *
         * When you do construct an object with a template, that template is for the whole class, not just the
         * constructor. But here, we don't need the whole class to be templated.
         *
         * @tparam SensorType The message type of the raw sensor data ROS topic. You cannot explicitly specify this template
         *   parameter due to a limitation of C++ syntax. This type must have a header.
         * @param graphManager
         * @param nodeHandle
         * @param sensorTopic The name of the ROS topic for raw sensor data.
         * @param odometryTopic The name of the actual odometry estimate corresponding to this sensor.
         * @param dummy  This value is unused. It only exists to allow the compiler to
         *   infer the template parameter `SensorType`.
         */
         // Implemented in the header because it's a template.
        template<typename SensorType>
        SensorManagerRos(std::shared_ptr<GraphManager> graphManager, ros::NodeHandle &nodeHandle,
                         const std::string &sensorTopic, const std::string &odometryTopic,
                         SensorType dummy)
        {
            _sensorSubscriber = nodeHandle.subscribe(sensorTopic, 1, &SensorManagerRos::sensorCallback<SensorType>, this);
            _odometrySubscriber = nodeHandle.subscribe(odometryTopic, 1, &SensorManagerRos::odometryCallback, this);
        };

    protected:
        std::shared_ptr<GraphManager> _graphManager;

        ros::Subscriber _sensorSubscriber;
        ros::Subscriber _odometrySubscriber;

        double _mostRecentSensorTime = 0;

        SensorManagerRos(std::shared_ptr<GraphManager> graphManager, ros::NodeHandle &nodeHandle,
                         const std::string &odometryTopic);

        // Implemented in header because it's a template.
        template<class SensorType>
        void sensorCallback(const boost::shared_ptr<SensorType> &msg)
        {
            _mostRecentSensorTime = msg->header.stamp.toSec();

        };
        void odometryCallback(const nav_msgs::Odometry::ConstPtr &ms);
    }; // class SensorManagerRos

} // namespace VILFusion