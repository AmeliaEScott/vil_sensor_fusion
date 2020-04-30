#ifndef VIL_FUSION_IMUSTATE_H
#define VIL_FUSION_IMUSTATE_H

#include <ceres/covariance.h>

#include <confusion/models/ImuChain.h>
#include <confusion/models/TagMeas.h>
#include <confusion/models/PoseMeas.h>
#include <confusion/utilities/Pose.h>
#include <confusion/utilities/imu_utils.h>
#include <confusion/State.h>

#include <eigen3/Eigen/Core>

namespace vil_fusion {

    class ImuState : public confusion::State {
    public:
        explicit ImuState(confusion::Pose<double> imu_to_lidar, double t = 0.0);
        explicit ImuState(const ImuState *other);

        bool initFirstState(
                const std::vector<std::deque<std::shared_ptr<confusion::ProcessMeasurement>>> &processMeasBuffer,
                const std::vector<std::deque<std::shared_ptr<confusion::UpdateMeasurement>>> &updateMeasBuffer,
                confusion::StaticParameterVector &staticParameters
                ) override;

        std::shared_ptr<confusion::State> createNextState(
                const std::vector<std::deque<std::shared_ptr<confusion::ProcessMeasurement>>> &processMeasBuffer,
                const std::vector<std::deque<std::shared_ptr<confusion::UpdateMeasurement>>> &updateMeasBuffer,
                confusion::StaticParameterVector &staticParameters
                ) override;

    protected:

        void setup_parameters();

        confusion::Pose<double> imu_to_lidar_;

        confusion::Pose<double> pose_;
        Eigen::Vector3d angVel_;
        Eigen::Vector3d linVel_;
        Eigen::Vector3d accelBias_;
        Eigen::Vector3d gyroBias_;

    };


}

#endif //VIL_FUSION_IMUSTATE_H
