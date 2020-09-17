/**
 * Design decisions:
 *
 *  - This file is completely divorced from ROS and from real time - thus, it can be be used outside of ROS, and in
 *    any sort of simulated-time system
 *  - All public functions are thread-safe - when used within ROS, messages may come from separate threads
 *  - Assume that IMU is fast enough that there will always be at least 1 IMU measurement between any two
 *    measurements from other sensors. This could be a false assumption if, for example, your camera and LiDAR are
 *    perfectly synchronized. However, in this case, the only issue would be that an IMU between factor which
 *    could have been interpolated is instead just missed.
 */

#pragma once

#include <gtsam_fusion/ImuManagerRos.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <mutex>
#include <atomic>
#include <tuple>
#include <map>

using namespace gtsam;

namespace VILFusion
{

    class GraphManager
    {
    public:
        using GraphType = NonlinearFactorGraph;
        using OptimizationCallback = std::function<void(double, Pose3&, Vector3&, imuBias::ConstantBias&)>;

        GraphManager(std::shared_ptr<ImuManagerRos> imuManager);

        boost::shared_ptr<const GraphType> graph();

        /**
         * Reserves a node in the pose graph at a certain time.
         *
         * A sensor manager should call this function when a sensor measurement is first received. After some time,
         * when the odometry estimate becomes available, that same manager should then call GraphManager::addFactor.
         *
         * Thread-safe.
         *
         * @param time time, in seconds, at which the sensor measurement was taken.
         * @return The index at which this node was reserved. To convert to a key, use
         *      one of gtsam::symbol_shorthand::{X, V, B}.
         */
        uint64_t reserveNode(double time);

        /**
         * Returns the most recent pose node reserved by GraphManager::reserveNode.
         *
         * @return Tuple of (time, key):
         *      - time is in Seconds
         *      - key is just a raw index. To convert to the appropriate Key for the graph,
         *        use one of gtsam::symbol_shorthand::{X, V, B}.
         */
        std::tuple<double, uint64_t> getMostRecentPoseTime();

        /**
         * TODO document this
         * @return
         */
        NavState getMostRecentEstimate();

        /**
         * Adds a between factor, which contains the difference in pose between two points in time.
         *
         * A sensor manager should call reserveNode every time a raw sensor measurement is received, and keep track
         * of the two most recent gtsam::Key it gets from this function. Then, when an odometry estimate is received,
         * it should call addBetweenFactor with these two most recent Keys.
         *
         * @param previousKey The Key received from GraphManager::reserveNode
         * @param currentKey The Key received from GraphManager::reserveNode for the most recent sensor reading
         * @param betweenPose The change in pose between the points in time indicated by previousKey and currentKey
         * @param noiseModel The noise model for this particular pose measurement
         */
        void addBetweenFactor(Key previousKey, Key currentKey, const Pose3 &betweenPose, const Pose3 &poseEstimate, const SharedNoiseModel &noiseModel);

        void addFactor(const CombinedImuFactor &factor);

        void solve();

        void addOptimizationCallback(OptimizationCallback callback);

        std::tuple<Pose3, Velocity3, imuBias::ConstantBias> getState();

        imuBias::ConstantBias getBias();

    private:
        using LockGuard = std::lock_guard<std::mutex>;

        std::shared_ptr<ImuManagerRos> _imuManager;

        std::mutex _graphMutex;
        std::mutex _stateMutex;
        boost::shared_ptr<GraphType> _graph;
        Values _values;
        uint64_t _currentKey = 0;
        double _lastPoseTime = -1;
        NavState _mostRecentEstimate;
        ISAM2 _isam2;

        std::deque<CombinedImuFactor> _imuQueue;

        struct {
            Pose3 pose = Pose3(Rot3(1, 0, 0, 0), Point3::Zero());
            Velocity3 vel = Velocity3::Zero();
            imuBias::ConstantBias bias = imuBias::ConstantBias(Vector6::Zero());
        } _currentState;

        std::vector<OptimizationCallback> _callbacks;

        void emptyImuQueue();


    }; // class GraphManager

} // namespace VILFusion