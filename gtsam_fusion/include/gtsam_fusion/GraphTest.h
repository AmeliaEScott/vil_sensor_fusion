#pragma once
#include <gtsam/navigation/CombinedImuFactor.h>
class GraphTest
{
public:
    GraphTest()
//    {
//        auto imuParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
//        preint = gtsam::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParams);
//    }
    ;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preint;
};
