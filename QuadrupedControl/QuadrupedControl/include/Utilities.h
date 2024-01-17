#ifndef UTILITIES_H
#define UTILITIES_H

#include "Types.h"
#define PI 3.1415926535

int GetLegSign(int leg);

template <typename T>
Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T>& v);

template <typename T>
Mat3<T> GetRotationMatrix(T angle, int axis);


#endif