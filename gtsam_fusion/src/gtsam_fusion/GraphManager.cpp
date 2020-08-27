
#include <gtsam_fusion/GraphManager.h>

#include <gtsam/inference/Symbol.h>

namespace VILFusion
{

    using symbol_shorthand::X;  // Pose
    using symbol_shorthand::V;  // Velocity
    using symbol_shorthand::B;  // IMU Bias

    GraphManager::GraphManager()
    {
        auto imuParams = PreintegratedCombinedMeasurements::Params::MakeSharedD();
        _preintegratedIMUData.integrator = std::make_shared<PreintegratedCombinedMeasurements>(imuParams);

        _graph = std::make_shared<GraphType>();
    }

    std::shared_ptr<const GraphManager::GraphType> GraphManager::graph()
    {
        return _graph;
    }

    Key GraphManager::reserveNode(double time)
    {
        // Automatically unlocks at end of scope (function)
        LockGuard lockGuard(_graphMutex);

        _currentKey++;
        _preintegratedIMUData.lastPoseTime = time;

        return X(_currentKey);
    }

    void GraphManager::addBetweenFactor(Key previousKey, Key currentKey, Pose3 betweenPose)
    {
        LockGuard lockGuard(_graphMutex);
        // TODO
    }

    void GraphManager::addIMUMeasurement(double time, const Vector3 &accelerometer, const Vector3 &gyroscope)
    {
        LockGuard lockGuard(_graphMutex);

        if(_preintegratedIMUData.lastIMUTime < _preintegratedIMUData.lastPoseTime && time > _preintegratedIMUData.lastPoseTime)
        {
            // There has been a new pose measurement inserted in the graph between the last IMU measurement and this one.
            const double interpolationFactor = (_preintegratedIMUData.lastPoseTime - _preintegratedIMUData.lastIMUTime) /
                    (time - _preintegratedIMUData.lastIMUTime);

            const Vector3 interpolatedAccel = (interpolationFactor * _preintegratedIMUData.lastAccelerometer) +
                    ((1.0 - interpolationFactor) * accelerometer);
            const Vector3 interpolatedGyro = (interpolationFactor * _preintegratedIMUData.lastGyroscope) +
                    ((1.0 - interpolationFactor) * gyroscope);

            const double dt = _preintegratedIMUData.lastPoseTime - _preintegratedIMUData.lastIMUTime;
            _preintegratedIMUData.integrator->integrateMeasurement(interpolatedAccel, interpolatedGyro, dt);

            CombinedImuFactor factor(
                    X(_currentKey - 1), V(_currentKey - 1),
                    X(_currentKey), V(_currentKey),
                    B(_currentKey - 1), B(_currentKey),
                    *_preintegratedIMUData.integrator
                    );
            _graph->add(factor);

            // TODO: Do the optimization and possibly change the next line to resetIntegrationAndSetBias()

            _preintegratedIMUData.integrator->resetIntegration();

            _preintegratedIMUData.lastIMUTime = _preintegratedIMUData.lastPoseTime;
            _preintegratedIMUData.lastAccelerometer = interpolatedAccel;
            _preintegratedIMUData.lastGyroscope = interpolatedGyro;
        }

        _preintegratedIMUData.integrator->integrateMeasurement(
                accelerometer,
                gyroscope,
                time - _preintegratedIMUData.lastIMUTime
                );

        _preintegratedIMUData.lastIMUTime = time;
        _preintegratedIMUData.lastAccelerometer = accelerometer;
        _preintegratedIMUData.lastGyroscope = gyroscope;
    }
}