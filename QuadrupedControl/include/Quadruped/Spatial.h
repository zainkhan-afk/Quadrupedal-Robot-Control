#ifndef SPATIAL_H
#define SPATIAL_H

#include "Quadruped/Types.h"
#include "Utilities.h"
#include <eigen3/Eigen/Dense>




dtypes::Mat6 CreateSpatialForm(const dtypes::Mat3& R, const dtypes::Vec3& r);

dtypes::Mat6 SpatialRotation(float q, int axis);

dtypes::Mat6 JointRotationMatrix(float q, int axis);

#endif