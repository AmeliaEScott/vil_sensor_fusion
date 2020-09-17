#pragma once

#include <gtsam_fusion/GraphManager.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/base/Matrix.h>

#include <mutex>
#include <deque>

namespace VILFusion
{
    using namespace gtsam;

    class IMUManager
    {
    public:
        struct Measurement {
            double time;
            Vector3 accel;
            Vector3 gyro;
        };

        explicit IMUManager(std::shared_ptr<GraphManager> graphManager,
                            boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams);

        void addIMUMeasurement(double time, const Vector3 accel, const Vector3 gyro);

        CombinedImuFactor getFactor(double startTime, double endTime, uint64_t currentIndex, imuBias::ConstantBias bias);
    protected:
        using LockGuard = std::lock_guard<std::mutex>;

        std::shared_ptr<GraphManager> _graphManager;
        std::mutex _bufferMutex;
        std::deque<Measurement> _buffer;
        std::shared_ptr<PreintegratedCombinedMeasurements> _integrator = nullptr;
        double _lastMeasurementTime = 0;
        Key _lastPoseKey = 0;
        Vector3 _lastMeasurementAccel = gtsam::Vector3::Zero();
        Vector3 _lastMeasurementGyro = gtsam::Vector3::Zero();
    }; // class IMUManager

} // namespace VILFusion