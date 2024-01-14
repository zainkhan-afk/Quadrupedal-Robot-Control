#ifndef UTILITIES_H
#define UTILITIES_H

#include "Types.h"
#define PI 3.1415926535

int GetLegSign(int leg)
{
	if (leg == 0 || leg == 2)
	{
		return -1;
	}
	else
	{
		return 1;
	}
}

template <typename T>
Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T>& v) {
	Mat3<T::Scalar> m;
	m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
	return m;
}


#endif