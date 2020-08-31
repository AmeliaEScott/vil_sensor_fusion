
#include <gtsam_fusion/IMUManager.h>

#include <gtsam/inference/Symbol.h>

namespace VILFusion
{

    using gtsam::symbol_shorthand::X;  // Pose
    using gtsam::symbol_shorthand::V;  // Velocity
    using gtsam::symbol_shorthand::B;  // IMU Bias

    IMUManager::IMUManager(std::shared_ptr<GraphManager> graphManager):
        _graphManager(std::move(graphManager))
    {
        // TODO: Add calibration
        auto imuParams = PreintegratedCombinedMeasurements::Params::MakeSharedD();
        _integrator = gtsam::make_shared<PreintegratedCombinedMeasurements>(imuParams);
    }

    void IMUManager::addIMUMeasurement(const IMUMeasurement &measurement)
    {
        auto lastPose = _graphManager->getMostRecentPoseTime();
        double lastPoseTime = std::get<0>(lastPose);
        Key lastPoseKey = std::get<1>(lastPose);
        if(_lastMeasurement.time < lastPoseTime && measurement.time > lastPoseTime)
        {
            // There has been a new pose measurement inserted in the graph between the last IMU measurement and this one.
            const double interpolationFactor = (lastPoseTime - _lastMeasurement.time) /
                                               (measurement.time - _lastMeasurement.time);

            const Vector3 interpolatedAccel = (interpolationFactor * _lastMeasurement.accel) +
                                              ((1.0 - interpolationFactor) * measurement.accel);
            const Vector3 interpolatedGyro = (interpolationFactor * _lastMeasurement.gyro) +
                                             ((1.0 - interpolationFactor) * measurement.gyro);

            const double dt = lastPoseTime - _lastMeasurement.time;
            _integrator->integrateMeasurement(interpolatedAccel, interpolatedGyro, dt);

            CombinedImuFactor factor(
                    X(lastPoseKey - 1), V(lastPoseKey - 1),
                    X(lastPoseKey), V(lastPoseKey),
                    B(lastPoseKey - 1), B(lastPoseKey),
                    *_integrator
            );
            _graphManager->addFactor(factor);

            // TODO: Do the optimization and possibly change the next line to resetIntegrationAndSetBias()

            _integrator->resetIntegration();

            _lastMeasurement = IMUMeasurement(
                    lastPoseTime,
                    interpolatedAccel,
                    interpolatedGyro
                    );
        }

        _integrator->integrateMeasurement(
                measurement.accel,
                measurement.gyro,
                measurement.time - _lastMeasurement.time
        );

        _lastMeasurement = measurement;
    }

}