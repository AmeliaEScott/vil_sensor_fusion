//
//#include "VILConfusorNode.h"
////#include "VILSensorEnumDefinition.h"
//
//#include <ros/console.h>
//#include <Eigen/Core>
//#include <confusion/ConFusor.h>
//#include <confusion/models/ImuMeas.h>
//#include <confusion/models/PoseMeas.h>
//#include <confusion/utilities/Pose.h>
//#include <confusion/SensorEnumDefinition.h>
//#include <confusion/ImuState.h>
//
//
//namespace vil_fusion {
//
//    VILConfusorNode::VILConfusorNode(ros::NodeHandle &node) :
//        _node(node),
//        _pose_pub(_node.advertise<geometry_msgs::PoseWithCovarianceStamped>("fused_pose", 1)),
//        _confusor(std::make_shared<ImuState>(referenceFrameOffsets_))
//    {
//        _imu_sub = _node.subscribe("imu", 10, &VILConfusorNode::imuCallback, this);
//        _loam_sub = _node.subscribe("loam_odometry", 10, &VILConfusorNode::loamOdometryCallback, this);
//        _rovio_sub = _node.subscribe("rovio_odometry", 10, &VILConfusorNode::rovioOdometryCallback, this);
//
//        referenceFrameOffsets_["world"] = std::make_shared<confusion::Pose<double>>();
//    }
//
//    VILConfusorNode::~VILConfusorNode() {
//
//    }
//
//    void VILConfusorNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
//        ROS_INFO("Got IMU message");
//        double t_imu_source = msg->header.stamp.toSec(); // - t_epoch_;
//        //t_imu_latest_ = t_imu_source;
//
//        Eigen::Vector3d a(msg->linear_acceleration.x,
//                          msg->linear_acceleration.y,
//                          msg->linear_acceleration.z);
//        Eigen::Vector3d w(msg->angular_velocity.x,
//                          msg->angular_velocity.y,
//                          msg->angular_velocity.z);
//
//        _confusor.addProcessMeasurement(std::make_shared<confusion::ImuMeas>(
//                t_imu_source, a, w
//                ));
//
//        std::cout << __FILE__ << ":" << __LINE__ << std::endl;
//        if(_confusor.assignMeasurements()){
//            std::cout << __FILE__ << ":" << __LINE__ << std::endl;
//            _confusor.optimize();
//            std::cout << __FILE__ << ":" << __LINE__ << std::endl;
//            _confusor.verbosePrint();
//            std::cout << __FILE__ << ":" << __LINE__ << std::endl;
//            std::cout << "\n\n\n###########################################################################\n\n\n";
//        }
//    }
//
//    void VILConfusorNode::loamOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//        commonOdometryCallback(msg, "world", POSEMEAS);
//    }
//
//    void VILConfusorNode::rovioOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//        commonOdometryCallback(msg, "world", POSEMEAS);
//    }
//
//    void VILConfusorNode::commonOdometryCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string reference_frame, const int meas_type) {
//        double t = msg->header.stamp.toSec(); // - t_epoch;
//
//        Eigen::Vector3d translate(
//                msg->pose.pose.position.x,
//                msg->pose.pose.position.y,
//                msg->pose.pose.position.z
//                );
//        Eigen::Quaterniond rot(
//                msg->pose.pose.orientation.w,
//                msg->pose.pose.orientation.x,
//                msg->pose.pose.orientation.y,
//                msg->pose.pose.orientation.z
//                );
//        auto pose = confusion::Pose<double>(translate, rot);
//        auto t_imu = confusion::Pose<double>();
//        auto T_w_ref = std::make_shared<confusion::Pose<double>>();
//
//        confusion::PoseMeasConfig config;
//        config.sensorOffset_rot_init_stddev = 1;
//        config.sensorOffset_trans_init_stddev = 1;
//        config.w_trans = 1;
//        config.w_rot = 1;
//        config.lossCoefficient = 1;
//
//        auto measurement = std::make_shared<confusion::PoseMeas>(
//                t,
//                reference_frame,
//                t_imu,
//                pose,
//                config,
//                meas_type
//                );
//        //measurement->assignExternalReferenceFrame(T_w_ref);
//        _confusor.addUpdateMeasurement(measurement);
//    }
//
//}

int main(int argc, char* argv[]) {
//    ros::init(argc, argv, "vil_fusion_node");
//
//    ros::NodeHandle nh("vil_fusion_node");
//
//    vil_fusion::VILConfusorNode node(nh);
//
//    ros::spin();
return 0;
}