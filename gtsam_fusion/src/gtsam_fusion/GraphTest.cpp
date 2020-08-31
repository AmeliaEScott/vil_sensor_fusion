
#include <gtsam_fusion/GraphTest.h>

GraphTest::GraphTest()
{
    auto imuParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
    preint = gtsam::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParams);
}