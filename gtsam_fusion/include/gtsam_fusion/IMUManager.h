#pragma once

#include <gtsam_fusion/GraphManager.h>
#include <gtsam/navigation/ImuFactor.h>

namespace VILFusion
{
    class IMUManager
    {
    public:

        explicit IMUManager(std::shared_ptr<GraphManager> graphManager);

        void addIMUMeasurement(double time, const Vector3 accel, const Vector3 gyro);
    protected:

        std::shared_ptr<GraphManager> _graphManager;

        boost::shared_ptr<PreintegratedCombinedMeasurements> _integrator = nullptr;
        double _lastMeasurementTime = 0;
        Key _lastPoseKey = 0;
        Vector3 _lastMeasurementAccel = gtsam::Vector3::Zero();
        Vector3 _lastMeasurementGyro = gtsam::Vector3::Zero();
    }; // class IMUManager

} // namespace VILFusion