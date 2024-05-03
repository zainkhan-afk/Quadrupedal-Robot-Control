#ifndef SPATIAL_H
#define SPATIAL_H

#include "Quadruped/Types.h"
#include "Utilities.h"
#include <eigen3/Eigen/Dense>




MathTypes::Mat6 CreateSpatialForm(const MathTypes::Mat3& R, const MathTypes::Vec3& r);
MathTypes::Mat6 SpatialRotation(float q, int axis);
MathTypes::Mat6 JointRotationMatrix(float q, int axis);
MathTypes::Vec6 JointMotionSubspace(JOINT_TYPE jointType, COORD_AXIS axis);

#endif