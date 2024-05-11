#ifndef SPATIAL_H
#define SPATIAL_H

#include "Quadruped/Types.h"
#include "Quadruped/Utilities.h"
#include <eigen3/Eigen/Dense>
#include "Quadruped/QuadrupedCommon.h"



MathTypes::Mat6 CreateSpatialForm(const MathTypes::Mat3& R, const MathTypes::Vec3& r);
MathTypes::Mat6 SpatialRotation(float q, COORD_AXIS axis);
MathTypes::Mat6 JointRotationMatrix(float q, COORD_AXIS axis);
MathTypes::Vec6 JointMotionSubspace(JOINT_TYPE jointType, COORD_AXIS axis);
MathTypes::Vec6 CrossProductMotion(const MathTypes::Vec6& v1, const MathTypes::Vec6& v2);
MathTypes::Vec6 CrossProductForce(const MathTypes::Vec6& v1, const MathTypes::Vec6& v2);
QUADRUPED_API MathTypes::Mat4 SpatialToHomog(const MathTypes::Mat6& X);
QUADRUPED_API MathTypes::Mat3 SpatialToRotMat(const MathTypes::Mat6& X);
QUADRUPED_API MathTypes::Vec3 SpatialToTranslation(const MathTypes::Mat6& X);

#endif