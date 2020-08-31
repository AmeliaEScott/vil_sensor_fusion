#pragma once

#include <gtsam_fusion/GraphManager.h>
#include <gtsam/navigation/ImuFactor.h>

namespace VILFusion
{
    class IMUManager
    {
    public:
        struct IMUMeasurement {
            IMUMeasurement(double time_, Vector3 accel_, Vector3 gyro_):
                time(time_),
                accel(accel_),
                gyro(gyro_)
            {};

            double time;
            Vector3 accel;
            Vector3 gyro;
        };

        IMUManager(std::shared_ptr<GraphManager> graphManager);

        void addIMUMeasurement(const IMUMeasurement &measurement);
    protected:

        std::shared_ptr<GraphManager> _graphManager;

        boost::shared_ptr<PreintegratedCombinedMeasurements> _integrator = nullptr;
        IMUMeasurement _lastMeasurement = IMUMeasurement(
                0,
                gtsam::Vector3::Zero(),
                gtsam::Vector3::Zero()
        );
    }; // class IMUManager

} // namespace VILFusion