#include "ImuState.h"
#include "VILSensorEnumDefinition.h"

namespace vil_fusion {

    ImuState::ImuState(const confusion::Pose<double> imu_to_lidar, double t)
        : confusion::State(t, NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS),
        imu_to_lidar_(imu_to_lidar)
    {

        setup_parameters();

        //Set the IMU biases to zero because they are not set in the initialize function
        //so that previously found bises can be re-used when tracking is restarted
        accelBias_.setZero();
        gyroBias_.setZero();
    }

    ImuState::ImuState(const ImuState *other) :
        confusion::State(other->t(), NUM_PROCESS_SENSORS, NUM_UPDATE_SENSORS),
        pose_(other->pose_),
        angVel_(other->angVel_),
        linVel_(other->linVel_),
        accelBias_(other->accelBias_),
        gyroBias_(other->gyroBias_)
    {
        setup_parameters();
    }

    void ImuState::setup_parameters() {
        processChains_[IMU] = std::make_shared<confusion::ImuChain>();

        parameters_.emplace_back(confusion::Parameter(
                pose_.trans.data(),
                3,
                "imu_translation"
        ));
        parameters_.emplace_back(confusion::Parameter(
                pose_.rot.coeffs().data(),
                4,
                "imu_rotation",
                std::make_shared<confusion::QuatParam>()
        ));

        parameters_.emplace_back(confusion::Parameter(linVel_.data(), 3, "imu_lin_vel"));
        parameters_.emplace_back(confusion::Parameter(accelBias_.data(), 3, "imu_bias_accel"));
        parameters_.emplace_back(confusion::Parameter(gyroBias_.data(), 3, "imu_bias_gyro"));
    }

    bool ImuState::initFirstState(
            const std::vector<std::deque<std::shared_ptr<confusion::ProcessMeasurement>>> &processMeasBuffer,
            const std::vector<std::deque<std::shared_ptr<confusion::UpdateMeasurement>>> &updateMeasBuffer,
            confusion::StaticParameterVector &staticParameters
    ) {
        if(processMeasBuffer[IMU].empty()) {
            return false;
        }

        std::shared_ptr<confusion::PoseMeas> pose_measurement;

        if(!updateMeasBuffer[LOAM_POSE].empty()) {
            pose_measurement = std::dynamic_pointer_cast<confusion::PoseMeas>(updateMeasBuffer[LOAM_POSE].front());
        } else if (!updateMeasBuffer[ROVIO_POSE].empty()) {
            pose_measurement = std::dynamic_pointer_cast<confusion::PoseMeas>(updateMeasBuffer[ROVIO_POSE].front());
        } else {
            return false;
        }

        pose_ = pose_measurement->getMeasuredImuPose();
        linVel_.setZero();
        angVel_.setZero();
        accelBias_.setZero();
        gyroBias_.setZero();
    }

    std::shared_ptr<confusion::State> ImuState::createNextState(
            const std::vector<std::deque<std::shared_ptr<confusion::ProcessMeasurement>>> &processMeasBuffer,
            const std::vector<std::deque<std::shared_ptr<confusion::UpdateMeasurement>>> &updateMeasBuffer,
            confusion::StaticParameterVector &staticParameters
    ) {
        bool foundGoodUpdate = false;
        double t_des;
        size_t measType = 0;
        while (measType < NUM_UPDATE_SENSORS) {
            if (!updateMeasBuffer[measType].empty() && processMeasBuffer[IMU].back()->t() >= updateMeasBuffer[measType].front()->t()) {
                t_des = updateMeasBuffer[measType].front()->t();
                foundGoodUpdate = true;
                break;
            }
            ++measType;
        }

        if (!foundGoodUpdate) {
            std::cout << "[ImuState::createNextState] Didn't receive a process measurement to initialize the next state. Will wait and try again." << std::endl;
            return nullptr;
        }

        auto new_state = std::make_shared<ImuState>(this);

        auto firstMeasPtr = std::dynamic_pointer_cast<confusion::ImuMeas>(processMeasBuffer[IMU].front());
        Eigen::Vector3d g_w = firstMeasPtr->imuCalibration_->g_w_;

        confusion::forwardPropagateImuMeas(
                processMeasBuffer[IMU],
                t_,
                t_des,
                g_w,
                new_state->pose_,
                new_state->linVel_,
                new_state->accelBias_,
                new_state->gyroBias_
                );

        new_state->t_ = t_des;
        return std::dynamic_pointer_cast<State>(new_state);
    }

}