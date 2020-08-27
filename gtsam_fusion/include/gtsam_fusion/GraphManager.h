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

#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <mutex>

using namespace gtsam;

namespace VILFusion
{

    class GraphManager
    {
    public:
        using GraphType = NonlinearFactorGraph;

        GraphManager();

        std::shared_ptr<const GraphType> graph();

        /**
         * Reserves a node in the pose graph at a certain time.
         *
         * A sensor manager should call this function when a sensor measurement is first received. After some time,
         * when the odometry estimate becomes available, that same manager should then call GraphManager::addFactor.
         *
         * Thread-safe.
         *
         * @param time time, in seconds, at which the sensor measurement was taken.
         * @return The GTSAM Key at which this node was reserved.
         */
        Key reserveNode(double time);

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
         */
        void addBetweenFactor(Key previousKey, Key currentKey, Pose3 betweenPose);

        void addIMUMeasurement(double time, const Vector3 &accelerometer, const Vector3 &gyroscope);


    private:
        using LockGuard = std::lock_guard<std::mutex>;

        std::mutex _graphMutex;
        std::shared_ptr<GraphType> _graph;
        Key _currentKey = 0;

        struct {
            std::shared_ptr<PreintegratedCombinedMeasurements> integrator = nullptr;
            double lastIMUTime = 0;
            double lastPoseTime = 0;
            Vector3 lastAccelerometer = Vector3::Zero();
            Vector3 lastGyroscope = Vector3::Zero();
        } _preintegratedIMUData;

    }; // class GraphManager

} // namespace VILFusion