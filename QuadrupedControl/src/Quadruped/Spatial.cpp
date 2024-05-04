#include "pch.h"
#include "Quadruped/Spatial.h"



MathTypes::Mat6 CreateSpatialForm(const MathTypes::Mat3& R, const MathTypes::Vec3& r) {

	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	X.template bottomLeftCorner<3, 3>() = -R * VectorToSkewMat(r);

	return X;
}

MathTypes::Mat6 SpatialRotation(float q, COORD_AXIS axis) {
	MathTypes::Mat3 R = GetRotationMatrix(q, axis);
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	return X;
}

MathTypes::Mat6 JointRotationMatrix(float q, COORD_AXIS axis) {
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X = SpatialRotation(q, axis);
	return X;
}

MathTypes::Vec6 JointMotionSubspace(JOINT_TYPE jointType, COORD_AXIS axis) {
    //TODO: Need to implement JointMotionSpace generation for joints other than single axis movement types. 
    
    MathTypes::Vec6 S = MathTypes::Vec6::Zero();

	int baseIdx = 0;
	int offsetIdx = 0;

	if (jointType == JOINT_TYPE::REVOLUTE) { baseIdx = 3; }
	else if (jointType == JOINT_TYPE::REVOLUTE) { baseIdx = 0; }
	else { return S; }

	if (axis == COORD_AXIS::X) { offsetIdx = 0; }
	else if (axis == COORD_AXIS::Y) { offsetIdx = 1; }
	else if (axis == COORD_AXIS::Z) { offsetIdx = 2; }
	else { return S; }

	S[baseIdx + offsetIdx] = 1;

    return S;
}

MathTypes::Vec6 CrossProductMotion(const MathTypes::Vec6& v1, const MathTypes::Vec6& v2) {
	MathTypes::Vec6 v;

	v << v1(1) * v2(2) - v1(2) * v2(1),
		 v1(2) * v2(0) - v1(0) * v2(2),
		 v1(0) * v2(1) - v1(1) * v2(0),
		 v1(1) * v2(5) - v1(2) * v2(4) + v1(4) * v2(2) - v1(5) * v2(1),
		 v1(2) * v2(3) - v1(0) * v2(5) - v1(3) * v2(2) + v1(5) * v2(0),
		 v1(0) * v2(4) - v1(1) * v2(3) + v1(3) * v2(1) - v1(4) * v2(0);

	return v;
}

MathTypes::Vec6 CrossProductForce(const MathTypes::Vec6& v1, const MathTypes::Vec6& v2) {
	MathTypes::Vec6 v;

	v << v2(2) * v1(1) - v2(1) * v1(2) - v2(4) * v1(5) + v2(5) * v1(4),
		 v2(0) * v1(2) - v2(2) * v1(0) + v2(3) * v1(5) - v2(5) * v1(3),
		 v2(1) * v1(0) - v2(0) * v1(1) - v2(3) * v1(4) + v2(4) * v1(3),
		 v2(5) * v1(1) - v2(4) * v1(2),
		 v2(3) * v1(2) - v2(5) * v1(0),
		 v2(4) * v1(0) - v2(3) * v1(1);

	return v;
}


QUADRUPED_API MathTypes::Mat4 SpatialToHomog(const MathTypes::Mat6& X)
{
	MathTypes::Mat4 H = MathTypes::Mat4::Zero();
	H.template topLeftCorner<3, 3>() = SpatialToRotMat(X);
	H.template topRightCorner<3, 1>() = SpatialToTranslation(X);
	H(3, 3) = 1;
	return H;
}


MathTypes::Mat3 SpatialToRotMat(const MathTypes::Mat6& X)
{
	MathTypes::Mat3 R = X.template topLeftCorner<3, 3>();
	return R;
}

MathTypes::Vec3 SpatialToTranslation(const MathTypes::Mat6& X)
{
	MathTypes::Mat3 R = SpatialToRotMat(X);
	MathTypes::Vec3 r = -SkewMatToVecor(R.transpose() * X.template bottomLeftCorner<3, 3>());
	return r;
}