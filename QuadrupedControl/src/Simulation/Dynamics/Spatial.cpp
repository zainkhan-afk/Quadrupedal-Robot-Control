#include "Simulation/Dynamics/Spatial.h"



MathTypes::Mat6 CreateSpatialForm(const MathTypes::Mat3& R, const MathTypes::Vec3& r) {

	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	X.template bottomLeftCorner<3, 3>() = -R * VectorToSkewMat(r);
	//X.template bottomLeftCorner<3, 3>() = VectorToSkewMat(r) * R;

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

	if (jointType == JOINT_TYPE::PRISMATIC) { baseIdx = 3; }
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

	//MathTypes::Mat6 v_sp = MathTypes::Mat6::Zero();

	//v_sp.template topLeftCorner<3, 3>() = VectorToSkewMat(MathTypes::Vec3(v1[0], v1[1], v1[2]));
	//v_sp.template bottomRightCorner<3, 3>() = VectorToSkewMat(MathTypes::Vec3(v1[0], v1[1], v1[2]));
	//v_sp.template bottomLeftCorner<3, 3>() = VectorToSkewMat(MathTypes::Vec3(v1[3], v1[4], v1[5]));

	//v = v_sp * v2;
	v << v1(1) * v2(2) - v1(2) * v2(1),
		 v1(2) * v2(0) - v1(0) * v2(2),
		 v1(0) * v2(1) - v1(1) * v2(0),
		 v1(1) * v2(5) - v1(2) * v2(4) + v1(4) * v2(2) - v1(5) * v2(1),
		 v1(2) * v2(3) - v1(0) * v2(5) - v1(3) * v2(2) + v1(5) * v2(0),
		 v1(0) * v2(4) - v1(1) * v2(3) + v1(3) * v2(1) - v1(4) * v2(0);


	/*v = MathTypes::Vec6(
		-v1[2] * v2[1] + v1[1] * v2[2],
		v1[2] * v2[0] - v1[0] * v2[2],
		-v1[1] * v2[0] + v1[0] * v2[1],
		-v1[5] * v2[1] + v1[4] * v2[2] - v1[2] * v2[4] + v1[1] * v2[5],
		v1[5] * v2[0] - v1[3] * v2[2] + v1[2] * v2[3] - v1[0] * v2[5],
		-v1[4] * v2[0] + v1[3] * v2[1] - v1[1] * v2[3] + v1[0] * v2[4]
	);*/

	return v;
}

MathTypes::Vec6 CrossProductForce(const MathTypes::Vec6& v1, const MathTypes::Vec6& v2) {
	MathTypes::Vec6 v;

	MathTypes::Mat6 v_sp = MathTypes::Mat6::Zero();


	//v_sp.template topLeftCorner<3, 3>() = VectorToSkewMat(MathTypes::Vec3(v1[0], v1[1], v1[2]));
	//v_sp.template bottomRightCorner<3, 3>() = VectorToSkewMat(MathTypes::Vec3(v1[0], v1[1], v1[2]));
	//v_sp.template topRightCorner<3, 3>() = VectorToSkewMat(MathTypes::Vec3(v1[3], v1[4], v1[5]));

	//v = v_sp * v2;

	v << v2(2) * v1(1) - v2(1) * v1(2) - v2(4) * v1(5) + v2(5) * v1(4),
		 v2(0) * v1(2) - v2(2) * v1(0) + v2(3) * v1(5) - v2(5) * v1(3),
		 v2(1) * v1(0) - v2(0) * v1(1) - v2(3) * v1(4) + v2(4) * v1(3),
		 v2(5) * v1(1) - v2(4) * v1(2),
		 v2(3) * v1(2) - v2(5) * v1(0),
		 v2(4) * v1(0) - v2(3) * v1(1);


	/*v = MathTypes::Vec6(
		-v1[2] * v2[1] + v1[1] * v2[2] - v1[5] * v2[4] + v1[4] * v2[5],
		v1[2] * v2[0] - v1[0] * v2[2] + v1[5] * v2[3] - v1[3] * v2[5],
		-v1[1] * v2[0] + v1[0] * v2[1] - v1[4] * v2[3] + v1[3] * v2[4],
		-v1[2] * v2[4] + v1[1] * v2[5],
		v1[2] * v2[3] - v1[0] * v2[5],
		-v1[1] * v2[3] + v1[0] * v2[4]
	);*/

	return v;
}


MathTypes::Mat4 SpatialToHomog(const SpatialTransform& X)
{
	MathTypes::Mat4 H = MathTypes::Mat4::Zero();
	H.template topLeftCorner<3, 3>() = X.GetRotation();
	H.template topRightCorner<3, 1>() = X.GetTranslation();
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
	MathTypes::Vec3 r = -SkewMatToVecor(-R.transpose() * X.template bottomLeftCorner<3, 3>());
	//MathTypes::Vec3 r = SkewMatToVecor(X.template bottomLeftCorner<3, 3>() * R.transpose());
	return r;
}