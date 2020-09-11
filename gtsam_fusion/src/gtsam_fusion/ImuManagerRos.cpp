
#include <gtsam_fusion/ImuManagerRos.h>

namespace VILFusion
{
    ImuManagerRos::ImuManagerRos(ros::NodeHandle &nh, std::shared_ptr<GraphManager> graphManager, const std::string imuTopic) :
        IMUManager(graphManager, getImuParams(nh))
    {
        _imuSub = nh.subscribe(imuTopic, 10, &ImuManagerRos::imuCallback, this);
    }

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> ImuManagerRos::getImuParams(ros::NodeHandle &nh)
    {
        auto params = PreintegratedCombinedMeasurements::Params::MakeSharedD();
        // TODO: Read ROS params for covariance
        return params;
    }

    void ImuManagerRos::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        double time = msg->header.stamp.toSec();
        gtsam::Vector3 accel(
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z
                );
        gtsam::Vector3 gyro(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z
                );
        addIMUMeasurement(time, accel, gyro);
    }
}