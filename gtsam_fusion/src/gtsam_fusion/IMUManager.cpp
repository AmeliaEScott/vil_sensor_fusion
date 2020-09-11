
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
        // TODO: Add calibration
        _integrator = std::make_shared<PreintegratedCombinedMeasurements>(imuParams);
    }

    void IMUManager::addIMUMeasurement(double time, const Vector3 accel, const Vector3 gyro)
    {
        auto lastPose = _graphManager->getMostRecentPoseTime();
        double lastPoseTime = std::get<0>(lastPose);
        Key lastPoseKey = std::get<1>(lastPose);
        if(_lastMeasurementTime < lastPoseTime && time > lastPoseTime)
        {
            // There has been a new pose measurement inserted in the graph between the last IMU measurement and this one.
            const double interpolationFactor = (lastPoseTime - _lastMeasurementTime) /
                                               (time - _lastMeasurementTime);

            const Vector3 interpolatedAccel = (interpolationFactor * _lastMeasurementAccel) +
                                              ((1.0 - interpolationFactor) * accel);
            const Vector3 interpolatedGyro = (interpolationFactor * _lastMeasurementGyro) +
                                             ((1.0 - interpolationFactor) * gyro);

            const double dt = lastPoseTime - _lastMeasurementTime;
            _integrator->integrateMeasurement(interpolatedAccel, interpolatedGyro, dt);

            CombinedImuFactor factor(
                    X(_lastPoseKey), V(_lastPoseKey),
                    X(lastPoseKey), V(lastPoseKey),
                    B(_lastPoseKey), B(lastPoseKey),
                    *_integrator
            );
            _graphManager->addFactor(factor);

            // TODO: Do the optimization and possibly change the next line to resetIntegrationAndSetBias()

            _integrator->resetIntegration();

            _lastMeasurementTime = lastPoseTime;
            _lastMeasurementAccel = interpolatedAccel;
            _lastMeasurementGyro = interpolatedGyro;
            _lastPoseKey = lastPoseKey;
        }

        _integrator->integrateMeasurement(
                accel,
                gyro,
                time - _lastMeasurementTime
        );

        _lastMeasurementTime = time;
        _lastMeasurementAccel = accel;
        _lastMeasurementGyro = gyro;
    }

}