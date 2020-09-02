
#include <gtsam_fusion/GraphManager.h>
#include <gtsam_fusion/IMUManager.h>
#include <gtsam_fusion/SensorManagerRos.h>
#include <gtsam_fusion/GraphTest.h>

#include <gtest/gtest.h>
#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Core>
#include <gtsam/global_includes.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

TEST(GraphManagerTest, iHopeThisWorks)
{
    ASSERT_TRUE(true);
    std::cerr << "It didn't segfault yet" << std::endl;
}

TEST(GraphManagerTest, segfaultTest)
{
    std::cerr << "GTSAM EIGEN VERSION: " << GTSAM_EIGEN_VERSION_WORLD << "." << GTSAM_EIGEN_VERSION_MAJOR << "." << GTSAM_EIGEN_VERSION_MINOR << \
        ", EIGEN VERSION: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

    GraphTest test;
    std::cerr << "Successfully constructed GraphTest" << std::endl;
}

TEST(GraphManagerTest, test1)
{
    auto graphManager = std::make_shared<VILFusion::GraphManager>();
    VILFusion::IMUManager imuManager(graphManager);

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

    auto key = graphManager->reserveNode(0.15);

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

    imuManager.addIMUMeasurement(
            0.3,
            gtsam::Vector3(0.2, 0.2, 0.2),
            gtsam::Vector3(0.2, 0.2, 0.2)
    );
}

TEST(GraphManagerTest, sensorManagerInstantiation)
{
    ros::NodeHandle nh;
    auto graphManager = std::make_shared<VILFusion::GraphManager>();
    auto sensorManager1 = std::shared_ptr<VILFusion::SensorManagerRos>(VILFusion::SensorManagerRos::New<sensor_msgs::PointCloud2>(
            graphManager, nh, "testSensorTopic1", "testOdometryTopic1"
            ));
    auto sensorManager2 = std::shared_ptr<VILFusion::SensorManagerRos>(VILFusion::SensorManagerRos::New<sensor_msgs::Image>(
            graphManager, nh, "testSensorTopic2", "testOdometryTopic2"
            ));
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}