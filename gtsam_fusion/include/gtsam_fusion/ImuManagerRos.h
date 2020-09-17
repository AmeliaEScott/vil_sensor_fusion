#pragma once

#include <gtsam_fusion/IMUManager.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace VILFusion
{
    class ImuManagerRos : public IMUManager
    {
    public:
        ImuManagerRos(ros::NodeHandle &nh, std::shared_ptr<GraphManager> graphManager);

    private:

        ros::Subscriber _imuSub;

        boost::shared_ptr<PreintegratedCombinedMeasurements::Params> getImuParams(ros::NodeHandle &nh);
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    };
}