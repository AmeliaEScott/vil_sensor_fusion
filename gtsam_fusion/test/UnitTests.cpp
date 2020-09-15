
#include <gtsam_fusion/GraphManager.h>
#include <gtsam_fusion/IMUManager.h>
#include <gtsam_fusion/SensorManagerRos.h>
#include <gtsam_fusion/ImuManagerRos.h>

#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Core>
#include <gtsam/global_includes.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>


TEST(ImuManagerTest, test1)
{
    auto graphManager = std::make_shared<VILFusion::GraphManager>();
    auto imuParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
    VILFusion::IMUManager imuManager(graphManager, imuParams);

    imuManager.addIMUMeasurement(
            0.0,
            gtsam::Vector3(0.0, 0.0, 0.0),
            gtsam::Vector3(0.0, 0.0, 0.0)
    );

    imuManager.addIMUMeasurement(
            0.1,
            gtsam::Vector3(0.1, 0.1, 0.1),
            gtsam::Vector3(0.1, 0.1, 0.1)
    );

    auto key = graphManager->reserveNode(0.13);
    auto key2 = graphManager->reserveNode(0.15);
//    std::cerr << "Key1: " << key << ", Key2: " << key2 << std::endl;

    imuManager.addIMUMeasurement(
            0.2,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );

//    std::cerr << "NUM FACTORS: " << graphManager->graph()->nrFactors() << std::endl;

//    testing::internal::CaptureStdout();
//    graphManager->graph()->print();
//    std::string output = testing::internal::GetCapturedStdout();
//    std::cerr << output;
    auto firstFactor = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(graphManager->graph()->at(3));

    auto actualNavState = firstFactor->preintegratedMeasurements().deltaXij();
    auto expectedDV = gtsam::Vector3(0.0175, 0.0175, 0.0175);
    auto expectedDP = gtsam::Vector3(0.0011875, 0.0011875, 0.0011875);
    for(int i = 0; i < 3; i++){
        EXPECT_FLOAT_EQ(actualNavState.pose().translation()[i], expectedDP[i]) << "Pose not equal at index " << i;
        EXPECT_FLOAT_EQ(actualNavState.velocity()[i], expectedDV[i]) << "Velocity not equal at index " << i;
    }
//    std::cerr << "Keys: " <<
//        gtsam::Symbol(firstFactor->key1()) << ", " <<
//        gtsam::Symbol(firstFactor->key2()) << ", " <<
//        gtsam::Symbol(firstFactor->key3()) << ", " <<
//        gtsam::Symbol(firstFactor->key4()) << ", " <<
//        gtsam::Symbol(firstFactor->key5()) << ", " <<
//        gtsam::Symbol(firstFactor->key6()) << ", " << std::endl;

    imuManager.addIMUMeasurement(
            0.3,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );
}

TEST(ImuManagerTest, test2)
{
    auto graphManager = std::make_shared<VILFusion::GraphManager>();
    auto imuParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
    VILFusion::IMUManager imuManager(graphManager, imuParams);

    imuManager.addIMUMeasurement(
            0.0,
            gtsam::Vector3(0.0, 0.0, 0.0),
            gtsam::Vector3(0.0, 0.0, 0.0)
    );

    imuManager.addIMUMeasurement(
            0.1,
            gtsam::Vector3(0.1, 0.1, 0.1),
            gtsam::Vector3(0.1, 0.1, 0.1)
    );

    auto key = graphManager->reserveNode(0.13);

    imuManager.addIMUMeasurement(
            0.2,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );

    imuManager.addIMUMeasurement(
            0.3,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );

    key = graphManager->reserveNode(0.35);

    imuManager.addIMUMeasurement(
            0.4,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );
    imuManager.addIMUMeasurement(
            0.5,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );

    key = graphManager->reserveNode(0.55);

    imuManager.addIMUMeasurement(
            0.6,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );

//    std::cerr << "NUM FACTORS: " << graphManager->graph()->nrFactors() << std::endl;

//    testing::internal::CaptureStdout();
//    graphManager->graph()->print();
//    std::string output = testing::internal::GetCapturedStdout();
//    std::cerr << output;

    // This does not work because the system is underdetermined.
    // graphManager->solve();
}

TEST(SensorManagerTest, sensorManagerInstantiation)
{
    ros::NodeHandle nh1("/gtsam_fusion/sensors/lidar");
    ros::NodeHandle nh2("/gtsam_fusion/sensors/vio");
    auto graphManager = std::make_shared<VILFusion::GraphManager>();
    auto sensorManager1 = VILFusion::SensorManagerRos(
            graphManager, nh1, sensor_msgs::PointCloud2()
    );
    auto sensorManager2 = VILFusion::SensorManagerRos(
            graphManager, nh2, sensor_msgs::Image()
    );
}

TEST(SensorManagerTest, sensorManagerTest1)
{
    ros::NodeHandle nh;
    ros::NodeHandle nhLidar("/gtsam_fusion/sensors/lidar");
    auto sensorPub = nh.advertise<sensor_msgs::PointCloud2>("/test/lidar", 5);
    auto odomPub = nh.advertise<nav_msgs::Odometry>("/test/lidarOdom", 5);

    auto graphManager = std::make_shared<VILFusion::GraphManager>();
    VILFusion::SensorManagerRos sensorManager(graphManager, nhLidar, sensor_msgs::PointCloud2());

    // For some reason, spinOnce() is not doing what I expect (Send messages and run the callbacks).
    // But when I sleep for 0 duration, it does do that.
    auto d = ros::Duration(0.0);
    d.sleep();

    sensor_msgs::PointCloud2 pointCloud;
    pointCloud.header.stamp.sec = 0;
    pointCloud.header.stamp.nsec = 0;
    sensorPub.publish(pointCloud);

    // Spin once to let message go through and reach SensorManager's callback
    d.sleep();
    ros::spinOnce();

    nav_msgs::Odometry odom;
    odom.header.stamp.sec = 0;
    odom.header.stamp.sec = 0;
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.w = 1;
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odomPub.publish(odom);

    d.sleep();
    ros::spinOnce();

    // Only one Odometry has been published, so SensorManager should still not have added a between factor.
    // The only factors are the prior factors.
    EXPECT_EQ(graphManager->graph()->nrFactors(), 3);

    pointCloud.header.stamp.sec = 0;
    pointCloud.header.stamp.nsec = 500000000;
    sensorPub.publish(pointCloud);
    d.sleep();
    ros::spinOnce();

    odom.header.stamp.sec = 0;
    odom.header.stamp.nsec = 500000000;
    odom.pose.pose.position.x = 1;
    odom.pose.pose.position.y = 1;
    odom.pose.pose.position.z = 1;
    odom.pose.pose.orientation.w = 0.5;
    odom.pose.pose.orientation.x = 0.5;
    odom.pose.pose.orientation.y = 0.5;
    odom.pose.pose.orientation.z = 0.5;
    odomPub.publish(odom);
    d.sleep();
    ros::spinOnce();

    // Now there should be one factor, between two keys.
    EXPECT_EQ(graphManager->graph()->nrFactors(), 4);

    std::tuple<double, uint64_t> lastPose = graphManager->getMostRecentPoseTime();
    EXPECT_DOUBLE_EQ(std::get<0>(lastPose), 0.5);
    EXPECT_EQ(std::get<1>(lastPose), 2);

    auto firstFactor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(graphManager->graph()->at(3));
    EXPECT_EQ(firstFactor->key1(), gtsam::symbol_shorthand::X(1));
    EXPECT_EQ(firstFactor->key2(), gtsam::symbol_shorthand::X(2));
    EXPECT_DOUBLE_EQ(firstFactor->measured().x(), 1.0);
    EXPECT_DOUBLE_EQ(firstFactor->measured().y(), 1.0);
    EXPECT_DOUBLE_EQ(firstFactor->measured().z(), 1.0);
}

TEST(IntegrationTest, integrationTest1)
{
    // This test sends:
    //  - 4 images
    //  - 2 lidar point clouds
    //  - A bunch of IMU messages in between
    ros::NodeHandle nh;
    ros::NodeHandle nhLidar("/gtsam_fusion/sensors/lidar");
    ros::NodeHandle nhImage("/gtsam_fusion/sensors/vio");
    auto graphManager = std::make_shared<VILFusion::GraphManager>();

    VILFusion::ImuManagerRos imuManager(nh, graphManager, "/test/imu");
    sensor_msgs::Imu imuMsg;
    auto imuPub = nh.advertise<sensor_msgs::Imu>("/test/imu", 10);

    VILFusion::SensorManagerRos lidarManager(
            graphManager,
            nhLidar,
            sensor_msgs::PointCloud2());
    sensor_msgs::PointCloud2 lidarMsg;
    auto lidarPub = nh.advertise<sensor_msgs::PointCloud2>("/test/lidar", 10);
    nav_msgs::Odometry lidarOdomMsg;
    auto lidarOdomPub = nh.advertise<nav_msgs::Odometry>("/test/lidarOdom", 10);

    VILFusion::SensorManagerRos imageManager(
            graphManager,
            nhImage,
            sensor_msgs::Image());
    sensor_msgs::Image imageMsg;
    auto imagePub = nh.advertise<sensor_msgs::Image>("/test/image", 10);
    nav_msgs::Odometry imageOdomMsg;
    auto imageOdomPub = nh.advertise<nav_msgs::Odometry>("/test/imageOdom", 10);

    ros::Duration d(0, 0);

    for(double time : {0.1, 0.15, 0.2, 0.25})
    {
        imuMsg.header.stamp = ros::Time(time);
        imuPub.publish(imuMsg);
        d.sleep();
        ros::spinOnce();
    }

    imageMsg.header.stamp = ros::Time(0.27);
    imagePub.publish(imageMsg);
    d.sleep();
    ros::spinOnce();
    imageOdomMsg.header.stamp = ros::Time(0.27);
    imageOdomPub.publish(imageOdomMsg);
    d.sleep();
    ros::spinOnce();

    for(double time : {0.3, 0.35, 0.4, 0.45})
    {
        imuMsg.header.stamp = ros::Time(time);
        imuPub.publish(imuMsg);
        d.sleep();
        ros::spinOnce();
    }

    imageMsg.header.stamp = ros::Time(0.47);
    imagePub.publish(imageMsg);
    d.sleep();
    ros::spinOnce();
    imageOdomMsg.header.stamp = ros::Time(0.47);
    imageOdomPub.publish(imageOdomMsg);
    d.sleep();
    ros::spinOnce();

    for(double time : {0.5, 0.55, 0.6, 0.65})
    {
        imuMsg.header.stamp = ros::Time(time);
        imuPub.publish(imuMsg);
        d.sleep();
        ros::spinOnce();
    }

    lidarMsg.header.stamp = ros::Time(0.67);
    lidarPub.publish(lidarMsg);
    d.sleep();
    ros::spinOnce();
    lidarOdomMsg.header.stamp = ros::Time(0.67);
    lidarOdomPub.publish(lidarOdomMsg);
    d.sleep();
    ros::spinOnce();

    for(double time : {0.7, 0.75, 0.8, 0.85})
    {
        imuMsg.header.stamp = ros::Time(time);
        imuPub.publish(imuMsg);
        d.sleep();
        ros::spinOnce();
    }

    imageMsg.header.stamp = ros::Time(0.87);
    imagePub.publish(imageMsg);
    d.sleep();
    ros::spinOnce();
    imageOdomMsg.header.stamp = ros::Time(0.87);
    imageOdomPub.publish(imageOdomMsg);
    d.sleep();
    ros::spinOnce();

    for(double time : {0.9, 0.95, 1.0, 1.05})
    {
        imuMsg.header.stamp = ros::Time(time);
        imuPub.publish(imuMsg);
        d.sleep();
        ros::spinOnce();
    }

    imageMsg.header.stamp = ros::Time(1.07);
    imagePub.publish(imageMsg);
    d.sleep();
    ros::spinOnce();
    imageOdomMsg.header.stamp = ros::Time(1.07);
    imageOdomPub.publish(imageOdomMsg);
    d.sleep();
    ros::spinOnce();

    for(double time : {1.1, 1.15, 1.2, 1.25})
    {
        imuMsg.header.stamp = ros::Time(time);
        imuPub.publish(imuMsg);
        d.sleep();
        ros::spinOnce();
    }

    lidarMsg.header.stamp = ros::Time(1.27);
    lidarPub.publish(lidarMsg);
    d.sleep();
    ros::spinOnce();
    lidarOdomMsg.header.stamp = ros::Time(1.27);
    lidarOdomPub.publish(lidarOdomMsg);
    d.sleep();
    ros::spinOnce();

    for(double time : {1.3, 1.35})
    {
        imuMsg.header.stamp = ros::Time(time);
        imuPub.publish(imuMsg);
        d.sleep();
        ros::spinOnce();
    }

    // If IMU manager changes to avoid that first factor (From 0->1), then this should become 12.
    EXPECT_EQ(graphManager->graph()->size(), 13);

    graphManager->solve();

    EXPECT_EQ(graphManager->graph()->size(), 0);

    std::ofstream file;
    file.open("/home/timothy/Code/catkin_ws/GOOD_COOL_GRAPH.dot", std::ios::out | std::ios::binary | std::ios::trunc);
    graphManager->graph()->saveGraph(file);
    file.close();

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    // Is it a bad idea to use ROS params in unit tests? Probably.
    // Do I have time to do this better? Absolutely not!
    ros::param::set("/gtsam_fusion/sensors/lidar/sensor_topic", "/test/lidar");
    ros::param::set("/gtsam_fusion/sensors/lidar/odom_topic", "/test/lidarOdom");
    ros::param::set("/gtsam_fusion/sensors/lidar/use_odom_covariance", false);
    ros::param::set("/gtsam_fusion/sensors/lidar/covariance_linear", 0.1);
    ros::param::set("/gtsam_fusion/sensors/lidar/covariance_angular", 0.01);
    ros::param::set("/gtsam_fusion/sensors/vio/sensor_topic", "/test/image");
    ros::param::set("/gtsam_fusion/sensors/vio/odom_topic", "/test/imageOdom");
    ros::param::set("/gtsam_fusion/sensors/vio/use_odom_covariance", false);
    ros::param::set("/gtsam_fusion/sensors/vio/covariance_linear", 0.1);
    ros::param::set("/gtsam_fusion/sensors/vio/covariance_angular", 0.01);
    return RUN_ALL_TESTS();
}