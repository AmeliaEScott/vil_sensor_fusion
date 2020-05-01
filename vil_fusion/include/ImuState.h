#ifndef VIL_FUSION_IMUSTATE_H
#define VIL_FUSION_IMUSTATE_H

#include <confusion/models/ImuChain.h>
#include <confusion/models/PoseMeas.h>
#include <confusion/utilities/Pose.h>
#include <confusion/utilities/imu_utils.h>
#include <confusion/State.h>

#include "VILSensorEnumDefinition.h"

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

//        bool addUpdateMeasDerived(
//                std::shared_ptr<confusion::UpdateMeasurement> measPtr,
//                confusion::StaticParameterVector &staticParameters
//                ) override;

    protected:

        void setup_parameters();

        confusion::Pose<double> imu_to_lidar_;

        confusion::Pose<double> pose_;
        Eigen::Vector3d angVel_;
        Eigen::Vector3d linVel_;
        Eigen::Vector3d accelBias_;
        Eigen::Vector3d gyroBias_;

    };

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
        std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        bool foundGoodUpdate = false;
        std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        double t_des;
        size_t measType = 0;
        while (measType < NUM_UPDATE_SENSORS) {
            std::cout << __FILE__ << ":" << __LINE__ << std::endl;
            if (!updateMeasBuffer[measType].empty() && processMeasBuffer[IMU].back()->t() >= updateMeasBuffer[measType].front()->t()) {
                std::cout << __FILE__ << ":" << __LINE__ << std::endl;
                t_des = updateMeasBuffer[measType].front()->t();
                foundGoodUpdate = true;
                break;
            }
            std::cout << __FILE__ << ":" << __LINE__ << std::endl;
            ++measType;
        }

        if (!foundGoodUpdate) {
            std::cout << __FILE__ << ":" << __LINE__ << std::endl;
            std::cout << "[ImuState::createNextState] Didn't receive a process measurement to initialize the next state. Will wait and try again." << std::endl;
            std::cout << __FILE__ << ":" << __LINE__ << std::endl;
            return nullptr;
        }

        std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        auto new_state = std::make_shared<ImuState>(this);
        std::cout << __FILE__ << ":" << __LINE__ << std::endl;

        auto firstMeasPtr = std::dynamic_pointer_cast<confusion::ImuMeas>(processMeasBuffer[IMU].front());
        std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        Eigen::Vector3d g_w = firstMeasPtr->imuCalibration_->g_w_;

        std::cout << __FILE__ << ":" << __LINE__ << std::endl;
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
        std::cout << __FILE__ << ":" << __LINE__ << std::endl;

        new_state->t_ = t_des;
        std::cout << __FILE__ << ":" << __LINE__ << std::endl;
        return std::dynamic_pointer_cast<State>(new_state);
    }

//    bool ImuState::addUpdateMeasDerived(
//            std::shared_ptr<confusion::UpdateMeasurement> measPtr,
//            confusion::StaticParameterVector &staticParameters
//    ) {
//        auto poseMeasPtr = std::dynamic_pointer_cast<confusion::PoseMeas>(measPtr);
//        auto offset = std::make_shared<confusion::Pose<double>>();
//        poseMeasPtr->assignExternalReferenceFrame(offset);
//    }

}

#endif //VIL_FUSION_IMUSTATE_H
