#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>

using namespace gtsam;

class GraphTest
{
public:
    GraphTest();


private:
    std::shared_ptr<PreintegratedCombinedMeasurements> preint;
};
