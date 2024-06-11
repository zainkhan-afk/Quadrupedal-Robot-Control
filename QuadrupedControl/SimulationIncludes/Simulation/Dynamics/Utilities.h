#ifndef UTILITIES_H
#define UTILITIES_H

#include "Simulation/Dynamics/Types.h"
#include "Simulation/Dynamics/QuadrupedCommon.h"
#include "Simulation/Dynamics/SpatialTransform.h"

#define _USE_MATH_DEFINES
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
MathTypes::Vec3 SkewMatToVecor(const MathTypes::Mat3& m);
MathTypes::Mat3 GetRotationMatrix(double angle, COORD_AXIS);
MathTypes::Vec3 GetLegSignedVector(const MathTypes::Vec3& v, int legID);

MathTypes::Vec4 RotationMatrixToQuat(MathTypes::Mat3);
MathTypes::Vec3 RotationMatrixToEuler(MathTypes::Mat3);
MathTypes::Mat3 QuatToRotationMatrix(MathTypes::Vec4);
MathTypes::Vec3 QuatToEuler(MathTypes::Vec4);
MathTypes::Vec4 EulerToQuat(MathTypes::Vec3);

#endif