
#include <gtsam_fusion/GraphTest.h>

GraphTest::GraphTest()
{
    auto imuParams = PreintegratedCombinedMeasurements::Params::MakeSharedD();
    preint = std::make_shared<PreintegratedCombinedMeasurements>(imuParams);
}