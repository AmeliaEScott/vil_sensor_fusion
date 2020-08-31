
#include <gtsam_fusion/GraphManager.h>
#include <gtsam_fusion/IMUManager.h>
#include <gtsam_fusion/GraphTest.h>
//
#include <gtest/gtest.h>
// #include <ros/ros.h>
#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Core>
#include <gtsam/global_includes.h>

TEST(GraphManagerTest, aaahhh)
{
    std::cerr << "GTSAM EIGEN VERSION: " << GTSAM_EIGEN_VERSION_WORLD << "." << GTSAM_EIGEN_VERSION_MAJOR << "." << GTSAM_EIGEN_VERSION_MINOR << \
        ", EIGEN VERSION: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;

    GraphTest test;
    std::cerr << "Successfully constructed GraphTest" << std::endl;
}

//TEST(GraphManagerTest, test1)
//{
//    auto graphManager = std::make_shared<VILFusion::GraphManager>();
//    VILFusion::IMUManager imuManager(graphManager);
//
//    imuManager.addIMUMeasurement(VILFusion::IMUManager::IMUMeasurement(
//            0.0,
//            gtsam::Vector3(0.0, 0.0, 0.0),
//            gtsam::Vector3(0.0, 0.0, 0.0)
//    ));
//
//    imuManager.addIMUMeasurement(VILFusion::IMUManager::IMUMeasurement(
//            0.1,
//            gtsam::Vector3(0.1, 0.1, 0.1),
//            gtsam::Vector3(0.1, 0.1, 0.1)
//    ));
//
//    auto key = graphManager->reserveNode(0.15);
//
//    imuManager.addIMUMeasurement(VILFusion::IMUManager::IMUMeasurement(
//            0.2,
//            gtsam::Vector3(0.2, 0.2, 0.2),
//            gtsam::Vector3(0.2, 0.2, 0.2)
//    ));
//
//    auto firstFactor = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(graphManager->graph()->at(0));
//
//    auto actualNavState = firstFactor->preintegratedMeasurements().deltaXij();
//    auto expectedDV = gtsam::Vector3(0.0175, 0.0175, 0.0175);
//    auto expectedDP = gtsam::Vector3(0.0011875, 0.0011875, 0.0011875);
//    for(int i = 0; i < 3; i++){
//        EXPECT_FLOAT_EQ(actualNavState.pose().translation()[i], expectedDP[i]) << "Pose not equal at index " << i;
//        EXPECT_FLOAT_EQ(actualNavState.velocity()[i], expectedDV[i]) << "Velocity not equal at index " << i;
//    }
//
//    imuManager.addIMUMeasurement(VILFusion::IMUManager::IMUMeasurement(
//            0.3,
//            gtsam::Vector3(0.2, 0.2, 0.2),
//            gtsam::Vector3(0.2, 0.2, 0.2)
//    ));
//
//    std::cerr << "Made it to the end of the test!" << std::endl;
//}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}