//
// Created by sunny on 18. 11. 25.
//
#include <Eigen/Dense>
using namespace Eigen;
#ifndef HW4_BASICMATH_H
#define HW4_BASICMATH_H
Eigen::Vector3d QuaternionToAngleAxis(Eigen::Quaterniond qt);

/// convert log(quaternion) -> quaternion
Eigen::Quaterniond AngleAxisToQuaternion(Eigen::Vector3d angleAxis);
Eigen::Matrix3d AngleAxisToMatrix(Eigen::Vector3d angleAxis);
 
#endif //HW4_BASICMATH_H
