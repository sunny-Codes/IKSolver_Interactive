//
// Created by sunny on 18. 11. 25.
//
#include "BasicMath.h"

Eigen::Vector3d QuaternionToAngleAxis(Eigen::Quaterniond qt)
{
    double angle = atan2(qt.vec().norm(), qt.w());
    return angle * qt.vec().normalized();
}

/// convert log(quaternion) -> quaternion
Eigen::Quaterniond AngleAxisToQuaternion(Eigen::Vector3d angleAxis)
{
    Eigen::Quaterniond qt;
    qt.vec() = angleAxis.normalized() * sin(angleAxis.norm());
    qt.w() = cos(angleAxis.norm());
    return qt;
}