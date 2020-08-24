
#include <iostream>
#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>

int main(int argc, char *argv[])
{
    Eigen::Matrix<double, 2, 2> vec;
    gtsam::Point3 point3;
    std::cout << "TEST!!" << vec << point3 << std::endl;


    return 1;
}