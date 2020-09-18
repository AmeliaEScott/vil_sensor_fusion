
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>

#include <iostream>

int main(int argc, char* argv[])
{
    auto params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
    auto cov = gtsam::Matrix3::Identity() * 0.0001;
    params->setBiasAccOmegaInt(gtsam::Matrix6::Identity() * 0.0001);
    params->setIntegrationCovariance(cov);
    params->setBiasAccCovariance(cov);
    params->setBiasOmegaCovariance(cov);
    params->setAccelerometerCovariance(cov);
    params->setGyroscopeCovariance(cov);

    gtsam::PreintegratedCombinedMeasurements integrator(params);

    for(int i = 0; i < 10; i++)
    {
        integrator.integrateMeasurement(
                gtsam::Vector3(i * 0.01, i * 0.02, i * 0.03 + 9.81),
                gtsam::Vector3(i * 0.004, i * 0.005, i * 0.006),
                0.01
                );
    }

    std::cout << "Noise model from PreintegratedCombinedMeasurements: \n" << integrator.preintMeasCov() << std::endl;

    gtsam::CombinedImuFactor factor(
            0, 1, 2, 3, 4, 5, integrator
            );

    std::cout << "Noise model from CombinedImuFactor: \n";
    auto noise = boost::static_pointer_cast<gtsam::noiseModel::Gaussian>(factor.noiseModel());
    std::cout << noise->covariance() << "\n\n\nInformation:\n" << noise->information() << "\n";

    return 1;
}