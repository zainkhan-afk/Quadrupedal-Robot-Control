#ifndef SPATIAL_H
#define SPATIAL_H

#include "Quadruped/Types.h"
#include "Utilities.h"
#include <eigen3/Eigen/Dense>




auto CreateSpatialForm(const dtypes::Mat3& R, const dtypes::Vec3& r) {

	dtypes::Mat6 X = dtypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	X.template bottomLeftCorner<3, 3>() = -R * VectorToSkewMat(r);

	return X;
}

dtypes::Mat6 SpatialRotation(float q, int axis) {
	dtypes::Mat3 R = GetRotationMatrix(q, axis);
	dtypes::Mat6 X = dtypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	return X;
}

dtypes::Mat6 JointRotationMatrix(float q, int axis) {
    dtypes::Mat6 X = dtypes::Mat6::Zero();
    X = SpatialRotation(axis, q);
    return X;
}

#endif