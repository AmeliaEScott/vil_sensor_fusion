
#include <gtsam_fusion/ImuManagerRos.h>

namespace VILFusion
{
    ImuManagerRos::ImuManagerRos(ros::NodeHandle &nh) :
        IMUManager(getImuParams(nh))
    {
        std::string imuTopic;
        nh.getParam("topic", imuTopic);
        _imuSub = nh.subscribe(imuTopic, 100, &ImuManagerRos::imuCallback, this);
    }

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> ImuManagerRos::getImuParams(ros::NodeHandle &nh)
    {
        auto params = PreintegratedCombinedMeasurements::Params::MakeSharedU();
        // TODO: Read ROS params for covariance
        double biasAcc, biasOmega, accel, gyro, integration, biasAccInt;

        nh.getParam("cov_bias_acc", biasAcc);
        nh.getParam("cov_bias_omega", biasOmega);
        nh.getParam("cov_accel", accel);
        nh.getParam("cov_gyro", gyro);
        nh.getParam("cov_integration", integration);
        nh.getParam("cov_bias_acc_omega_int", biasAccInt);

        Matrix3 ident = Matrix3::Identity();
        params->setBiasAccCovariance(ident * biasAcc);
        params->setBiasOmegaCovariance(ident * biasOmega);
        params->setAccelerometerCovariance(ident * accel);
        params->setGyroscopeCovariance(ident * gyro);
        params->setIntegrationCovariance(ident * integration);
        params->setBiasAccOmegaInt(Matrix6::Identity() * biasAccInt);

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
