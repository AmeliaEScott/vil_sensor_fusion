
#include <gtsam_fusion/GraphManager.h>
#include <gtsam_fusion/GraphTest.h>

#include <gtest/gtest.h>
// #include <ros/ros.h>
#include <iostream>
#include <gtsam/navigation/CombinedImuFactor.h>


TEST(GraphManagerTest, test1)
{
    VILFusion::GraphManager manager;
    //GraphTest test;

    EXPECT_TRUE(false) << "1";

//    manager.addIMUMeasurement(
//            0.0,
//            gtsam::Vector3(0.0, 0.0, 0.0),
//            gtsam::Vector3(0.0, 0.0, 0.0)
//            );
//    EXPECT_TRUE(false) << "2";
//    manager.addIMUMeasurement(
//            0.1,
//            gtsam::Vector3(0.1, 0.1, 0.1),
//            gtsam::Vector3(0.1, 0.1, 0.1)
//            );
//    EXPECT_TRUE(false) << "3";
//    auto key = manager.reserveNode(0.15);
//    EXPECT_TRUE(false) << "4";
//    manager.addIMUMeasurement(
//            0.2,
//            gtsam::Vector3(0.2, 0.2, 0.2),
//            gtsam::Vector3(0.2, 0.2, 0.2)
//            );
//    EXPECT_TRUE(false) << "5";
//    manager.addIMUMeasurement(
//            0.3,
//            gtsam::Vector3(0.2, 0.2, 0.2),
//            gtsam::Vector3(0.2, 0.2, 0.2)
//    );
//    EXPECT_TRUE(false) << "6";
//
//    int nothing = 1;

//    auto imuParams = PreintegratedCombinedMeasurements::Params::MakeSharedD();
//    auto preint = std::make_shared<PreintegratedCombinedMeasurements>(
//            imuParams, imuBias::ConstantBias()
//    );
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}