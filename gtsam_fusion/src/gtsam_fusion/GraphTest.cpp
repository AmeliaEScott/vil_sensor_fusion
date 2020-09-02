
#include <gtsam_fusion/GraphTest.h>

GraphTest::GraphTest()
{
    auto imuParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();
    preint = boost::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuParams);
}