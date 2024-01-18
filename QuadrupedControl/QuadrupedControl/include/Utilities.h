#ifndef UTILITIES_H
#define UTILITIES_H

#include "Types.h"
#define PI 3.1415926535

//int GetLegSign(int leg)
//{
//	if (leg == 0 || leg == 2)
//	{
//		return -1;
//	}
//	else
//	{
//		return 1;
//	}
//}

template <typename T>
Mat3<T> vectorToSkewMat(const Vec3<T>& v) {
	Mat3<T> m;
	m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
	return m;
}

template <typename T>
Mat3<T> GetRotationMatrix(T angle, int axis)
{
	// Axis can be 0, 1 or 2 for x, y or z respectively.
	Mat3<T> rotation;

	T s = std::sin(angle);
	T c = std::cos(angle);

	if (axis == 0)
	{
		rotation << 1, 0, 0, 0, c, s, 0, -s, c;
	}
	else if (axis == 1)
	{
		rotation << c, 0, -s, 0, 1, 0, s, 0, c;
	}
	else if (axis == 2)
	{
		rotation << c, s, 0, -s, c, 0, 0, 0, 1;
	}

	return rotation;
}


#endif