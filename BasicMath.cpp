//
// Created by sunny on 18. 11. 25.
//
#include "BasicMath.h"

Eigen::Vector3d QuaternionToAngleAxis(Eigen::Quaterniond qt)
{
    if(qt.w()==0) return 3.141592/2.0*Vector3d(qt.x(),qt.y(),qt.z());
    if(qt.w()==1) return Vector3d(0,0,0);

    double angle = atan2(qt.vec().norm(), qt.w());
    return angle * qt.vec().normalized();
}

/// convert log(quaternion) -> quaternion
Eigen::Quaterniond AngleAxisToQuaternion(Eigen::Vector3d angleAxis)
{
    if(angleAxis.norm()<1e-4) return Quaterniond(1,0,0,0);
    Eigen::Quaterniond qt;
    qt.vec() = angleAxis.normalized() * sin(angleAxis.norm());
    qt.w() = cos(angleAxis.norm());
    return qt;
}

Eigen::Matrix3d AngleAxisToMatrix(Eigen::Vector3d angleAxis){
    return AngleAxisToQuaternion(angleAxis).toRotationMatrix();
}
