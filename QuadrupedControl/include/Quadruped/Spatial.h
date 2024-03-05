#ifndef SPATIAL_H
#define SPATIAL_H

#include "Quadruped/Types.h"
#include "Utilities.h"
#include <eigen3/Eigen/Dense>

template<typename T>
VecSp<T> GetSpatialInertia()
{

}


template <typename T>
MatSp<T> createSpatialForm(const Mat3<T>& R, const Vec3<T>& r) {
    Mat6<typename T::Scalar> X = Mat6<typename T::Scalar>::Zero();
    X.template topLeftCorner<3, 3>() = R;
    X.template bottomRightCorner<3, 3>() = R;
    X.template bottomLeftCorner<3, 3>() = -R * vectorToSkewMat(r);
    return X;
}

#endif