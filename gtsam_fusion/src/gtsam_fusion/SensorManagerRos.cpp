
#include <gtsam_fusion/SensorManagerRos.h>
#include <gtsam/inference/Symbol.h>


namespace VILFusion
{

    using gtsam::symbol_shorthand::X;  // Pose

    void SensorManagerRos::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        auto time = msg->header.stamp;
        std::tuple<ros::Time, Key> timeAndKey;
        bool found = false;
        while(!_keysAndTimes.empty() && !found)
        {
            std::tuple<ros::Time, Key> t = _keysAndTimes.front();
            _keysAndTimes.pop_front();
            if(std::get<0>(t) == time)
            {
                timeAndKey = t;
                found = true;
            }
        }
        if(!found)
        {
            ROS_WARN_STREAM("SensorManager: Received odometry at time " << msg->header.stamp.toSec() << ", but found no corresponding key.");
            return;
        }

        if(_lastValidOdom != nullptr)
        {
            auto deltaPose = poseDiff(_lastValidOdom, msg);

            auto key = std::get<1>(timeAndKey);
            gtsam::Pose3 pose(
                    gtsam::Rot3(
                            deltaPose.pose.orientation.w,
                            deltaPose.pose.orientation.x,
                            deltaPose.pose.orientation.y,
                            deltaPose.pose.orientation.z
                            ),
                    gtsam::Point3(
                            deltaPose.pose.position.x,
                            deltaPose.pose.position.y,
                            deltaPose.pose.position.z
                            )
                    );

            gtsam::Matrix66 cov;
            std::copy(deltaPose.covariance.data(), deltaPose.covariance.data() + deltaPose.covariance.size(), cov.data());
            auto noise = gtsam::noiseModel::Gaussian::Covariance(cov);

            ROS_DEBUG_STREAM("SensorManager: Added between factor: " << pose);
            _graphManager->addBetweenFactor(X(_lastValidKey), X(key), pose, noise);
        }
        else
        {
            ROS_DEBUG("SensorManager: No pose yet received.");
        }

        _lastValidOdom = msg;
        _lastValidKey = std::get<1>(timeAndKey);
    }

    geometry_msgs::PoseWithCovariance SensorManagerRos::poseDiff(const nav_msgs::Odometry::ConstPtr &before, const nav_msgs::Odometry::ConstPtr &after)
    {
        geometry_msgs::PoseWithCovariance out;

        // Both odometry messages are in the same fixed global frame. Therefore, position difference doesn't worry
        // about rotation.
        out.pose.position.x = after->pose.pose.position.x - before->pose.pose.position.x;
        out.pose.position.y = after->pose.pose.position.y - before->pose.pose.position.y;
        out.pose.position.z = after->pose.pose.position.z - before->pose.pose.position.z;

        Eigen::Quaterniond q1(
                before->pose.pose.orientation.w,
                before->pose.pose.orientation.x,
                before->pose.pose.orientation.y,
                before->pose.pose.orientation.z
                );
        Eigen::Quaterniond q2(
                after->pose.pose.orientation.w,
                after->pose.pose.orientation.x,
                after->pose.pose.orientation.y,
                after->pose.pose.orientation.z
        );

        auto qr = q2 * q1.inverse();
        out.pose.orientation.w = qr.w();
        out.pose.orientation.x = qr.x();
        out.pose.orientation.y = qr.y();
        out.pose.orientation.z = qr.z();

        std::copy(after->twist.covariance.data(),
                  after->twist.covariance.data() + after->twist.covariance.size(),
                  out.covariance.data());
        return out;
    }
}