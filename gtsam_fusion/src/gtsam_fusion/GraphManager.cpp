
#include <gtsam_fusion/GraphManager.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

namespace VILFusion
{

    using symbol_shorthand::X;  // Pose
    using symbol_shorthand::V;  // Velocity
    using symbol_shorthand::B;  // IMU Bias

    GraphManager::GraphManager()
    {
        _graph = boost::make_shared<GraphType>();
        _values.clear();

        auto pose = Pose3(Rot3(1.0, 0.0, 0.0, 0.0), Point3(0, 0, 0));
        Velocity3 vel = Velocity3::Zero();

        _values.insert(X(0), pose);
        _values.insert(V(0), vel);
        _values.insert(B(0), imuBias::ConstantBias(Vector6::Zero()));

        auto pose_noise_model = noiseModel::Diagonal::Sigmas(
                (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
                        .finished());  // rad,rad,rad,m, m, m
        auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
        auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

        _graph->addPrior(X(0), pose, pose_noise_model);
        _graph->addPrior(V(0), vel, velocity_noise_model);
        _graph->addPrior(B(0), imuBias::ConstantBias(Vector6::Zero()), bias_noise_model);

        // Set ISAM2 parameters and create ISAM2 solver object
        ISAM2Params isam_params;
        isam_params.factorization = ISAM2Params::CHOLESKY;
        isam_params.relinearizeSkip = 10;

        _isam2 = ISAM2(isam_params);
    }

    boost::shared_ptr<const GraphManager::GraphType> GraphManager::graph()
    {
        return _graph;
    }

    uint64_t GraphManager::reserveNode(double time)
    {
        // Automatically unlocks at end of scope (function)
        LockGuard lockGuard(_graphMutex);

        _currentKey++;
        _lastPoseTime = time;

        return _currentKey;
    }

    std::tuple<double, uint64_t> GraphManager::getMostRecentPoseTime()
    {
        LockGuard lockGuard(_graphMutex);
        return std::make_tuple(_lastPoseTime, _currentKey);
    }

    NavState GraphManager::getMostRecentEstimate()
    {
        LockGuard lockGuard(_graphMutex);
        return _mostRecentEstimate;
    }

    void GraphManager::addBetweenFactor(Key previousKey, Key currentKey, const Pose3 &betweenPose, const SharedNoiseModel &noiseModel)
    {
        LockGuard lockGuard(_graphMutex);
        BetweenFactor<Pose3> factor(previousKey, currentKey, betweenPose, noiseModel);
        _graph->add(factor);
    }

    void GraphManager::addFactor(const CombinedImuFactor &factor)
    {
        LockGuard lockGuard(_graphMutex);
        _graph->add(factor);
        // TODO: Make this better!
        _mostRecentEstimate = factor.preintegratedMeasurements().predict(
                _mostRecentEstimate, imuBias::ConstantBias(Vector6::Zero()));
        _values.insert(X(_currentKey), _mostRecentEstimate.pose());
        _values.insert(V(_currentKey), _mostRecentEstimate.velocity());
        _values.insert(B(_currentKey), imuBias::ConstantBias(Vector6::Zero()));
    }

    void GraphManager::addOptimizationCallback(OptimizationCallback callback)
    {
        _callbacks.push_back(callback);
    }

    void GraphManager::solve()
    {
        _graphMutex.lock();
        auto graph = _graph->clone();
        Values values(_values);

        auto lastKeyIndex = _currentKey;
        auto lastPoseTime = _lastPoseTime;

        _graph->resize(0);
        _values.clear();
        _graphMutex.unlock();

        LockGuard lockGuard(_optimizerMutex);
        _isam2.update(graph, values);
        auto result = _isam2.calculateEstimate();

        auto lastPose = result.at<Pose3>(X(lastKeyIndex));
        auto lastVelocity = result.at<Velocity3>(V(lastKeyIndex));
        auto lastBias = result.at<imuBias::ConstantBias>(B(lastKeyIndex));

        for(auto &callback : _callbacks)
        {
            callback(lastPose, lastVelocity, lastBias);
        }

    }
}