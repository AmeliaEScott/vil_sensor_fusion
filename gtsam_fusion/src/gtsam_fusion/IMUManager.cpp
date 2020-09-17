
#include <gtsam_fusion/IMUManager.h>

#include <gtsam/inference/Symbol.h>

namespace VILFusion
{

    using gtsam::symbol_shorthand::X;  // Pose
    using gtsam::symbol_shorthand::V;  // Velocity
    using gtsam::symbol_shorthand::B;  // IMU Bias

    IMUManager::IMUManager(std::shared_ptr<GraphManager> graphManager,
                           boost::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams):
        _graphManager(std::move(graphManager))
    {
//        _graphManager->addOptimizationCallback([this](double time, Pose3 &pose, Velocity3 &vel, imuBias::ConstantBias &bias){
//            _bias = bias;
//        });
        _integrator = std::make_shared<PreintegratedCombinedMeasurements>(imuParams);
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

        Measurement prevMeas;
        // Get rid of any old measurements. Should be only 1, in theory.
        while(!_buffer.empty() && _buffer.front().time <= startTime)
        {
            prevMeas = _buffer.front();
            _buffer.pop_front();
        }

        _integrator->resetIntegrationAndSetBias(bias);

        prevMeas.time = startTime;
        // Integrate all of the easy measurements (that don't require interpolation).
        while(!_buffer.empty() && _buffer.front().time <= endTime)
        {
            auto newMeas = _buffer.front();
            _buffer.pop_front();
            _integrator->integrateMeasurement(
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

            _integrator->integrateMeasurement(interpAccel, interpGyro, endTime - prevMeas.time);
        }

        return CombinedImuFactor(
                X(currentIndex - 1), V(currentIndex - 1),
                X(currentIndex), V(currentIndex),
                B(currentIndex - 1), B(currentIndex),
                *_integrator
                );
    }

//    void IMUManager::addIMUMeasurement(double time, const Vector3 accel, const Vector3 gyro)
//    {
//        if(_lastMeasurementTime <= 0)
//        {
//            _lastMeasurementTime = time;
//            return;
//        }
//        auto lastPose = _graphManager->getMostRecentPoseTime();
//        double lastPoseTime = std::get<0>(lastPose);
//        Key lastPoseKey = std::get<1>(lastPose);
//        std::cout << "######IMU######: last pose time: " << lastPoseTime << ", last meas time: " << _lastMeasurementTime << ", cur time: " << time << ", key: " << lastPoseKey << std::endl;
//        if(_lastMeasurementTime <= lastPoseTime && time > lastPoseTime)
//        {
////            std::cout << "##################### IMU FACTOR ADDING #######################\n";
//            // There has been a new pose measurement inserted in the graph between the last IMU measurement and this one.
//            const double interpolationFactor = (lastPoseTime - _lastMeasurementTime) /
//                                               (time - _lastMeasurementTime);
//
//            const Vector3 interpolatedAccel = (interpolationFactor * _lastMeasurementAccel) +
//                                              ((1.0 - interpolationFactor) * accel);
//            const Vector3 interpolatedGyro = (interpolationFactor * _lastMeasurementGyro) +
//                                             ((1.0 - interpolationFactor) * gyro);
//
//            const double dt = lastPoseTime - _lastMeasurementTime;
//            if(dt > 0)
//            {
////                std::cout << "Integrating interpolated accel: " << interpolatedAccel << ", gyro: " << interpolatedGyro << ", dt: " << dt << std::endl;
//                _integrator->integrateMeasurement(interpolatedAccel, interpolatedGyro, dt);
//            }
//
//            CombinedImuFactor factor(
//                    X(_lastPoseKey), V(_lastPoseKey),
//                    X(lastPoseKey), V(lastPoseKey),
//                    B(_lastPoseKey), B(lastPoseKey),
//                    *_integrator
//            );
//            _graphManager->addFactor(factor);
//
//            _integrator->resetIntegrationAndSetBias(_graphManager->getBias());
//
//            _lastMeasurementTime = lastPoseTime;
//            _lastMeasurementAccel = interpolatedAccel;
//            _lastMeasurementGyro = interpolatedGyro;
//            _lastPoseKey = lastPoseKey;
//        }
//
////        std::cout << "Integrating accel: " << accel << ", gyro: " << gyro << ", dt: " << time - _lastMeasurementTime << std::endl;
//        _integrator->integrateMeasurement(
//                accel,
//                gyro,
//                time - _lastMeasurementTime
//        );
//
//        _lastMeasurementTime = time;
//        _lastMeasurementAccel = accel;
//        _lastMeasurementGyro = gyro;
//    }
}