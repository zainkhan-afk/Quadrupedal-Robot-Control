#include "pch.h"
#include "Quadruped/Spatial.h"



MathTypes::Mat6 CreateSpatialForm(const MathTypes::Mat3& R, const MathTypes::Vec3& r) {

	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	X.template bottomLeftCorner<3, 3>() = -R * VectorToSkewMat(r);

	return X;
}

MathTypes::Mat6 SpatialRotation(float q, int axis) {
	MathTypes::Mat3 R = GetRotationMatrix(q, axis);
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	return X;
}

MathTypes::Mat6 JointRotationMatrix(float q, int axis) {
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X = SpatialRotation(axis, q);
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