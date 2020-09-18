#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
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

        explicit IMUManager(boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams);

        void addIMUMeasurement(double time, const Vector3 accel, const Vector3 gyro);

        CombinedImuFactor getFactor(double startTime, double endTime, uint64_t currentIndex, imuBias::ConstantBias bias);
        CombinedImuFactor getFactor(double endTime, uint64_t currentIndex, imuBias::ConstantBias bias);
    protected:
        using LockGuard = std::lock_guard<std::mutex>;

        std::mutex _bufferMutex;
        std::deque<Measurement> _buffer;
        PreintegratedCombinedMeasurements _integrator;
    }; // class IMUManager

} // namespace VILFusion