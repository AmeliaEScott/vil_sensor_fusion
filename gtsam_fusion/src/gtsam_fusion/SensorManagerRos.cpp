
#include <gtsam_fusion/SensorManagerRos.h>

namespace VILFusion
{

    SensorManagerRos::SensorManagerRos()
    {
        // TODO
    }

    void SensorManagerRos::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        auto time = msg->header.stamp.toSec();
    }
}