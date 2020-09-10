
#include <gtsam_fusion/GraphManager.h>
#include <gtsam_fusion/IMUManager.h>
#include <gtsam_fusion/SensorManagerRos.h>
#include <gtsam_fusion/GraphTest.h>

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
    std::cerr << "Key1: " << key << ", Key2: " << key2 << std::endl;

    imuManager.addIMUMeasurement(
            0.2,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );

    auto firstFactor = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(graphManager->graph()->at(0));

    auto actualNavState = firstFactor->preintegratedMeasurements().deltaXij();
    auto expectedDV = gtsam::Vector3(0.0175, 0.0175, 0.0175);
    auto expectedDP = gtsam::Vector3(0.0011875, 0.0011875, 0.0011875);
    for(int i = 0; i < 3; i++){
        EXPECT_FLOAT_EQ(actualNavState.pose().translation()[i], expectedDP[i]) << "Pose not equal at index " << i;
        EXPECT_FLOAT_EQ(actualNavState.velocity()[i], expectedDV[i]) << "Velocity not equal at index " << i;
    }
    std::cerr << "Keys: " <<
        gtsam::Symbol(firstFactor->key1()) << ", " <<
        gtsam::Symbol(firstFactor->key2()) << ", " <<
        gtsam::Symbol(firstFactor->key3()) << ", " <<
        gtsam::Symbol(firstFactor->key4()) << ", " <<
        gtsam::Symbol(firstFactor->key5()) << ", " <<
        gtsam::Symbol(firstFactor->key6()) << ", " << std::endl;

    imuManager.addIMUMeasurement(
            0.3,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );
}

TEST(SensorManagerTest, sensorManagerInstantiation)
{
    ros::NodeHandle nh;
    auto graphManager = std::make_shared<VILFusion::GraphManager>();
    auto sensorManager1 = VILFusion::SensorManagerRos(
        graphManager, nh, "testSensorTopic1", "testOdometryTopic1", sensor_msgs::PointCloud2()
    );
    auto sensorManager2 = VILFusion::SensorManagerRos(
        graphManager, nh, "testSensorTopic2", "testOdometryTopic2", sensor_msgs::Image()
    );
}

TEST(SensorManagerTest, sensorManagerTest1)
{
    ros::NodeHandle nh;
    auto sensorPub = nh.advertise<sensor_msgs::PointCloud2>("/test/pointcloud", 5);
    auto odomPub = nh.advertise<nav_msgs::Odometry>("/test/odom", 5);

    auto graphManager = std::make_shared<VILFusion::GraphManager>();
    VILFusion::SensorManagerRos sensorManager(graphManager, nh, "/test/pointcloud", "/test/odom", sensor_msgs::PointCloud2());

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
    EXPECT_EQ(graphManager->graph()->nrFactors(), 0);

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
    EXPECT_EQ(graphManager->graph()->nrFactors(), 1);

    std::tuple<double, uint64_t> lastPose = graphManager->getMostRecentPoseTime();
    EXPECT_DOUBLE_EQ(std::get<0>(lastPose), 0.5);
    EXPECT_EQ(std::get<1>(lastPose), 2);

    auto firstFactor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(graphManager->graph()->at(0));
    EXPECT_EQ(firstFactor->key1(), gtsam::symbol_shorthand::X(1));
    EXPECT_EQ(firstFactor->key2(), gtsam::symbol_shorthand::X(2));
    EXPECT_DOUBLE_EQ(firstFactor->measured().x(), 1.0);
    EXPECT_DOUBLE_EQ(firstFactor->measured().y(), 1.0);
    EXPECT_DOUBLE_EQ(firstFactor->measured().z(), 1.0);

    std::ofstream file;
    file.open("/home/timothy/Code/catkin_ws/GOOD_COOL_GRAPH.pdf", std::ios::out | std::ios::binary);
    graphManager->graph()->saveGraph(file);
    file.close();
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}