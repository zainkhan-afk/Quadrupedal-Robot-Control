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


MathTypes::Mat3 VectorToSkewMat(const MathTypes::Vec3& v);
MathTypes::Mat3 GetRotationMatrix(float angle, COORD_AXIS);
MathTypes::Vec3 MatToSkewVec(const MathTypes::Mat3& m);
MathTypes::Vec3 GetLegSignedVector(const MathTypes::Vec3& v, int legID);
MathTypes::Mat3 RotationMatrixFromQuat(MathTypes::Vec4);


#endif