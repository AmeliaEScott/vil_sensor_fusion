#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <loam/OptStatus.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometry_filter");
    ros::NodeHandle nh(ros::this_node::getName());
    ROS_WARN_STREAM("################################### " << nh.getNamespace());
    double rotDegenThreshold, transDegenThreshold;
    nh.getParam("filter/rot_degen_threshold", rotDegenThreshold);
    nh.getParam("filter/trans_degen_threshold", transDegenThreshold);

    ROS_WARN_STREAM("######################### ROT: " << rotDegenThreshold << ", TRANS: " << transDegenThreshold);

    auto laserPub = nh.advertise<nav_msgs::Odometry>("laser_odom_output", 1);

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "laser_odom_input", 1);
    message_filters::Subscriber<loam::OptStatus> status_sub(nh, "laser_opt_status", 1);
    message_filters::TimeSynchronizer<nav_msgs::Odometry, loam::OptStatus> sync(odom_sub, status_sub, 10);
    // ROS insists that I use boost::bind. It doesn't work with a regular lambda, or even std::bind.
    sync.registerCallback(boost::bind<void>([&laserPub, &rotDegenThreshold, &transDegenThreshold](const nav_msgs::Odometry::ConstPtr &odom, const loam::OptStatus::ConstPtr &status){
        Eigen::Matrix<float, 6, 6> hessian;
        std::copy(status->hessian.data(), status->hessian.data() + status->hessian.size(), hessian.data());
        auto rotation = hessian.block<3, 3>(3, 3);
        auto translation = hessian.block<3, 3>(0, 0);

        auto rotDOpt = std::log(rotation.determinant());
        auto transDOpt = std::log(translation.determinant());
        // ROS_INFO_STREAM("Rot det: " << rotDOpt << ", transDet: " << transDOpt);

        if(rotDOpt < rotDegenThreshold || transDOpt < transDegenThreshold)
        {
            ROS_INFO_STREAM("Degeneracy! Rot DOpt: " << rotDOpt << ", Trans DOpt: " << transDOpt);
        }
        else
        {
            laserPub.publish(odom);
        }

    }, boost::placeholders::_1, boost::placeholders::_2));

    ros::spin();

    return 0;
}