#pragma once

#include <gtsam_fusion/GraphManager.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <deque>

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
                         SensorType dummy) :
             _graphManager(graphManager)
        {
            _sensorSubscriber = nodeHandle.subscribe(sensorTopic, 1, &SensorManagerRos::sensorCallback<SensorType>, this);
            _odometrySubscriber = nodeHandle.subscribe(odometryTopic, 1, &SensorManagerRos::odometryCallback, this);
        };

    protected:
        std::shared_ptr<GraphManager> _graphManager;

        ros::Subscriber _sensorSubscriber;
        ros::Subscriber _odometrySubscriber;

        // Use a queue instead of a map for efficiency.
        // I assume that odometry messages will always come in order. Therefore, when a message arrives, I can
        // forget all of the keys associated with times before that message.
        // A queue keeps everything in order, which makes deleting these keys easy and efficient.
        std::deque<std::tuple<ros::Time, Key>> _keysAndTimes;

        nav_msgs::Odometry::ConstPtr _lastValidOdom;
        Key _lastValidKey;

        SensorManagerRos(std::shared_ptr<GraphManager> graphManager, ros::NodeHandle &nodeHandle,
                         const std::string &odometryTopic);

        // Implemented in header because it's a template.
        template<class SensorType>
        void sensorCallback(const boost::shared_ptr<SensorType> &msg)
        {
            double time = msg->header.stamp.toSec();
            Key key = _graphManager->reserveNode(time);
            _keysAndTimes.emplace_back(std::make_tuple(msg->header.stamp, key));
        };
        void odometryCallback(const nav_msgs::Odometry::ConstPtr &ms);

        /**
         * Takes as input two Odometry messages which are based in the same fixed global frame. Outputs the pose
         * between those two messages.
         *
         * Assumes that the twist covariance represents the covariance of the pose between the two points.
         */
        virtual geometry_msgs::PoseWithCovariance poseDiff(const nav_msgs::Odometry::ConstPtr &before, const nav_msgs::Odometry::ConstPtr &after);
    }; // class SensorManagerRos

} // namespace VILFusion