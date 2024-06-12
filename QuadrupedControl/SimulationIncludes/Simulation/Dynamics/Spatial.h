#ifndef SPATIAL_H
#define SPATIAL_H

#include "Simulation/Dynamics/Types.h"
#include "Simulation/Dynamics/Utilities.h"
#include <eigen3/Eigen/Dense>



MathTypes::Mat6 CreateSpatialForm(const MathTypes::Mat3& R, const MathTypes::Vec3& r);
MathTypes::Mat6 SpatialRotation(float q, COORD_AXIS axis);
MathTypes::Mat6 JointRotationMatrix(float q, COORD_AXIS axis);
MathTypes::Vec6 JointMotionSubspace(JOINT_TYPE jointType, COORD_AXIS axis);
MathTypes::Vec6 CrossProductMotion(const MathTypes::Vec6& v1, const MathTypes::Vec6& v2);
MathTypes::Vec6 CrossProductForce(const MathTypes::Vec6& v1, const MathTypes::Vec6& v2);
MathTypes::Mat4 SpatialToHomog(const SpatialTransform& X);
MathTypes::Mat3 SpatialToRotMat(const MathTypes::Mat6& X);
MathTypes::Vec3 SpatialToTranslation(const MathTypes::Mat6& X);

#endif