
#include <gtsam_fusion/GraphManager.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace VILFusion
{

    using symbol_shorthand::X;  // Pose
    using symbol_shorthand::V;  // Velocity
    using symbol_shorthand::B;  // IMU Bias

    GraphManager::GraphManager(std::shared_ptr<ImuManagerRos> imuManager) : _imuManager(imuManager)
    {
        _graph = boost::make_shared<GraphType>();
        _values.clear();

        auto pose = Pose3(Rot3(1.0, 0.0, 0.0, 0.0), Point3(0, 0, 0));
        Velocity3 vel = Velocity3::Zero();

        _values.insert(X(0), pose);
        _values.insert(V(0), vel);
        _values.insert(B(0), imuBias::ConstantBias(Vector6::Zero()));

        auto pose_noise_model = noiseModel::Diagonal::Sigmas(
                (Vector(6) << 0.000001, 0.000001, 0.000001, 0.00005, 0.00005, 0.00005)
                        .finished());  // rad,rad,rad,m, m, m
        auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.00001);  // m/s
        auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-7);

        _graph->addPrior(X(0), pose, pose_noise_model);
        _graph->addPrior(V(0), vel, velocity_noise_model);
        _graph->addPrior(B(0), imuBias::ConstantBias(Vector6::Zero()), bias_noise_model);

        // Set ISAM2 parameters and create ISAM2 solver object
        ISAM2Params isam_params;
        isam_params.factorization = ISAM2Params::QR;
        isam_params.relinearizeThreshold = 0.0001;
        isam_params.relinearizeSkip = 1;

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
        CombinedImuFactor imuFactor;
        if(_currentKey > 1)
        {
            imuFactor = _imuManager->getFactor(_lastPoseTime, time, _currentKey, getBias());
        }
        else
        {
            imuFactor = _imuManager->getFactor(time, _currentKey, getBias());
        }
        _imuQueue.push_back(imuFactor);
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

    void GraphManager::addBetweenFactor(Key previousKey, Key currentKey, const Pose3 &betweenPose, const Pose3 &poseEstimate, const SharedNoiseModel &noiseModel)
    {
        LockGuard lockGuard(_graphMutex);
        BetweenFactor<Pose3> factor(previousKey, currentKey, betweenPose, noiseModel);
        _graph->add(factor);
    }

    void GraphManager::addFactor(const CombinedImuFactor &factor)
    {
        LockGuard lockGuard(_graphMutex);
        _imuQueue.push_back(factor);
    }

    void GraphManager::addOptimizationCallback(OptimizationCallback callback)
    {
        _callbacks.push_back(callback);
    }

    void GraphManager::solve()
    {
//        std::cout << "############################# OPTIMIZING!!!! #####################################" << std::endl;
        _graphMutex.lock();
        emptyImuQueue();

        auto graph = _graph->clone();
        Values values(_values);

        auto lastKeyIndex = _currentKey;
        auto lastPoseTime = _lastPoseTime;

        _graph->resize(0);
        _values.clear();
        _graphMutex.unlock();

        LockGuard lockGuard(_stateMutex);

//        std::cout << "\n\n\n###################### GRAPH: ############################\n";
//        graph.print();
//        std::cout << "\n\n ######################### VALUES: ############################\n";
//        values.print();
//        std::cout << "\n\n########################################\n";


        _isam2.update(graph, values);
        auto result = _isam2.calculateEstimate();
//        gtsam::LevenbergMarquardtOptimizer optimizer(graph, values);
//        auto result = optimizer.optimize();

        _currentState.pose = result.at<Pose3>(X(lastKeyIndex));
        _currentState.vel = result.at<Velocity3>(V(lastKeyIndex));
        _currentState.bias = result.at<imuBias::ConstantBias>(B(lastKeyIndex));

        for(auto &callback : _callbacks)
        {
            callback(lastPoseTime, _currentState.pose, _currentState.vel, _currentState.bias);
        }

//        std::cout << "\n\n########################OPTIMIZED!!!#########################\n";
    }

    void GraphManager::emptyImuQueue()
    {
        while(!_imuQueue.empty())
        {
            auto factor = _imuQueue.front();
            _imuQueue.pop_front();

            _graph->add(factor);

            auto currentState = NavState(_currentState.pose, _currentState.vel);
            auto nextState = factor.preintegratedMeasurements().predict(currentState, _currentState.bias);
            _currentState.pose = nextState.pose();
            _currentState.vel = nextState.velocity();

//            std::cerr << "CLEARING IMU QUEUE: Adding " << Symbol(factor.key3()) << ", " << Symbol(factor.key4()) << ", " << Symbol(factor.key6()) << std::endl;
            _values.insert(factor.key3(), _currentState.pose);
            _values.insert(factor.key4(), _currentState.vel);
            _values.insert(factor.key6(), _currentState.bias);
        }
    }

    std::tuple<Pose3, Velocity3, imuBias::ConstantBias> GraphManager::getState()
    {
        LockGuard guard(_stateMutex);
        return std::make_tuple(
                _currentState.pose,
                _currentState.vel,
                _currentState.bias
                );
    }

    imuBias::ConstantBias GraphManager::getBias()
    {
        LockGuard guard(_stateMutex);
        return _currentState.bias;
    }
}