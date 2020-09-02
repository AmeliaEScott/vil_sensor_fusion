
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
    }
}