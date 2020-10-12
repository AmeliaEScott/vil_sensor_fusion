
#include <gtsam_fusion/SensorManagerRos.h>
#include <gtsam/inference/Symbol.h>


namespace VILFusion
{

    using gtsam::symbol_shorthand::X;  // Pose

    void SensorManagerRos::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
//        ROS_INFO_STREAM("Odometry msg from " << _odometrySubscriber.getTopic() << " at time " << msg->header.stamp.toSec());
        if(!_hasReceivedOdometry)
        {
            _hasReceivedOdometry = true;
            return;
        }
        auto time = msg->header.stamp;
        std::tuple<ros::Time, Key> timeAndKey;
        bool found = false;
        while(!_keysAndTimes.empty() && !found)
        {
            std::tuple<ros::Time, Key> t = _keysAndTimes.front();
            if(std::get<0>(t) > time)
            {
                break;
            }
            _keysAndTimes.pop_front();
//            ROS_INFO_STREAM("Searching for t=" << time.toSec() << ", found " << std::get<0>(t).toSec());

            //if(std::get<0>(t) == time)
//            ROS_INFO_STREAM("Manager for " << _odometrySubscriber.getTopic() << " Searching for image from " << time.sec << "." << time.nsec << ". Diff: " << std::abs((std::get<0>(t) - time).toNSec()));
            if(std::abs((std::get<0>(t) - time).toNSec()) < 1000000)
            {
                timeAndKey = t;
                found = true;
//                ROS_INFO_STREAM("FOUND!");
            }
        }
        if(!found)
        {
            ROS_WARN_STREAM("SensorManager for " << _odometrySubscriber.getTopic() << ": Received odometry at time " << msg->header.stamp.toSec() << ", but found no corresponding key.");
            return;
        }

        if(_lastValidOdom != nullptr && (msg->header.stamp - _lastValidOdom->header.stamp).toSec() < _maxTimeSkip)
        {
            auto deltaPose = poseDiff(_lastValidOdom, msg);

//            auto dt = (msg->header.stamp.toSec() - _lastValidOdom->header.stamp.toSec());
//            auto dx = msg->pose.pose.position.x - _lastValidOdom->pose.pose.position.x;
//            std::cout << "SensorManager for " << _odometrySubscriber.getTopic() << ": Moved " << dx << "m in " << dt << "s (" << dx / dt << "m/s)\n";

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

            gtsam::Pose3 absPose(
                    gtsam::Rot3(
                            msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z
                            ),
                    gtsam::Point3(
                            msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z
                            )
                    );

            Matrix66 cov = Matrix66::Zero();
            if(_useOdomCovariance)
            {
                std::copy(deltaPose.covariance.data(), deltaPose.covariance.data() + deltaPose.covariance.size(), cov.data());
            }
            else
            {
                cov.diagonal() <<
                        _linearCovariance,
                        _linearCovariance,
                        _linearCovariance,
                        _angularCovariance,
                        _angularCovariance,
                        _angularCovariance;
            }
            auto noise = gtsam::noiseModel::Gaussian::Covariance(cov);


//            ROS_INFO_STREAM("SensorManager for " << _odometrySubscriber.getTopic() <<
//            ": Added factor between x" << _lastValidKey << "(Time " << _lastValidOdom->header.stamp.toSec() << ") and x"
//            << key << " (" << msg->header.stamp.toSec() << "): " << pose.translation() << ", " << pose.rotation());
            _graphManager->addBetweenFactor(X(_lastValidKey), X(key), pose, absPose, noise);

            if(_optimizeAfterOdom)
            {
//                ROS_DEBUG_STREAM("SensorManager for " << _odometrySubscriber.getTopic() << " optimizing...");
                _graphManager->solve();
            }
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

        Eigen::Vector3d x1(before->pose.pose.position.x, before->pose.pose.position.y, before->pose.pose.position.z);
        Eigen::Vector3d x2(after->pose.pose.position.x, after->pose.pose.position.y, after->pose.pose.position.z);

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

        auto dx = x2 - x1;
        auto dxr = q1.inverse() * dx;
        out.pose.position.x = dxr.x();
        out.pose.position.y = dxr.y();
        out.pose.position.z = dxr.z();

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