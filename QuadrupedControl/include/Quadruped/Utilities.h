#ifndef UTILITIES_H
#define UTILITIES_H

#include "Quadruped/Types.h"
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


dtypes::Mat3 VectorToSkewMat(const dtypes::Vec3& v);


dtypes::Mat3 GetRotationMatrix(float angle, int axis);


dtypes::Vec3 MatToSkewVec(const dtypes::Mat3& m);


dtypes::Vec3 GetLegSignedVector(const dtypes::Vec3& v, int legID);


#endif