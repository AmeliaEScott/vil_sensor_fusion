
#include <gtsam_fusion/IMUManager.h>

#include <gtsam/inference/Symbol.h>

namespace VILFusion
{

    using gtsam::symbol_shorthand::X;  // Pose
    using gtsam::symbol_shorthand::V;  // Velocity
    using gtsam::symbol_shorthand::B;  // IMU Bias

    IMUManager::IMUManager(boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams) :
        _integrator(imuParams, imuBias::ConstantBias(Vector6::Zero()))
    {
        _integrator.resetIntegration();
    }

    void IMUManager::addIMUMeasurement(double time, const Vector3 accel, const Vector3 gyro)
    {
        LockGuard guard(_bufferMutex);
        _buffer.push_back(Measurement {
            .time = time, .accel = accel, .gyro = gyro
        });
    }

    CombinedImuFactor IMUManager::getFactor(double startTime, double endTime, uint64_t currentIndex, imuBias::ConstantBias bias)
    {
        LockGuard _lockGuard(_bufferMutex);

//        std::cout << "######## IMU retrieving factor between " << startTime << " and " << endTime << " for index " << currentIndex << std::endl;

        Measurement prevMeas;
        // Get rid of any old measurements. Should be only 1, in theory.
        while(!_buffer.empty() && _buffer.front().time <= startTime)
        {
            prevMeas = _buffer.front();
//            std::cout << "Skipping measurement at t=" << prevMeas.time << std::endl;
            _buffer.pop_front();
        }

        _integrator.resetIntegrationAndSetBias(bias);

        prevMeas.time = startTime;
        // Integrate all of the easy measurements (that don't require interpolation).
        while(!_buffer.empty() && _buffer.front().time < endTime)
        {
            auto newMeas = _buffer.front();
            _buffer.pop_front();
            _integrator.integrateMeasurement(
                    newMeas.accel, newMeas.gyro, newMeas.time - prevMeas.time
                    );
            prevMeas = newMeas;
        }

        // The final measurement requires interpolation.
        if(!_buffer.empty())
        {
            auto finalMeas = _buffer.front();
            const double interpolationFactor = (endTime - prevMeas.time) / (finalMeas.time - prevMeas.time);
            const Vector3 interpAccel = (interpolationFactor * finalMeas.accel) + ((1.0 - interpolationFactor) * prevMeas.accel);
            const Vector3 interpGyro = (interpolationFactor * finalMeas.gyro) + ((1.0 - interpolationFactor) * prevMeas.gyro);

            _integrator.integrateMeasurement(interpAccel, interpGyro, endTime - prevMeas.time);

        }

        return CombinedImuFactor(
                X(currentIndex - 1), V(currentIndex - 1),
                X(currentIndex), V(currentIndex),
                B(currentIndex - 1), B(currentIndex),
                _integrator
                );
    }

    CombinedImuFactor IMUManager::getFactor(double endTime, uint64_t currentIndex, imuBias::ConstantBias bias)
    {
        return getFactor(_buffer.front().time, endTime, currentIndex, bias);
    }
}