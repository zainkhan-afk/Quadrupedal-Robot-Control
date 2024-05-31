#include "pch.h"
#include "Quadruped/SpatialTransform.h"
#include "Quadruped/Utilities.h"
#include "Quadruped/Spatial.h"

SpatialTransform::SpatialTransform()
{
	R = MathTypes::Mat3::Identity();
	p = MathTypes::Vec3::Zero();
}

SpatialTransform::SpatialTransform(const MathTypes::Mat3& _R, const MathTypes::Vec3& _p)
{
	R = _R;
	p = _p;
}

SpatialTransform::SpatialTransform(const MathTypes::Vec3& _R, const MathTypes::Vec3& _p)
{
	MathTypes::Mat3 Rx = GetRotationMatrix(_R[0], COORD_AXIS::X);
	MathTypes::Mat3 Ry = GetRotationMatrix(_R[1], COORD_AXIS::Y);
	MathTypes::Mat3 Rz = GetRotationMatrix(_R[2], COORD_AXIS::Z);

	R = Rx * Ry * Rz;
	p = _p;
}

SpatialTransform::~SpatialTransform()
{}

MathTypes::Mat6 SpatialTransform::GetSpatialForm()
{
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	// X.template topLeftCorner<3, 3>() = R;
	// X.template bottomRightCorner<3, 3>() = R;
	// X.template bottomLeftCorner<3, 3>() = -R * VectorToSkewMat(p); // This was in the book

	X.block<3, 3>(0, 0) = R;
	X.block<3, 3>(0, 3) = MathTypes::Mat3::Zero();
	X.block<3, 3>(3, 0) = -R * VectorToSkewMat(p);
	X.block<3, 3>(3, 3) = R;

	return X;
}

MathTypes::Mat6 SpatialTransform::GetSpatialFormTranspose()
{
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	/*X.template topLeftCorner<3, 3>() = R.transpose();
	X.template bottomRightCorner<3, 3>() = R.transpose();
	X.template topRightCorner<3, 3>() = (-R * VectorToSkewMat(p)).transpose();*/

	X.block<3, 3>(0, 0) = R.transpose();
	X.block<3, 3>(0, 3) = -(R * VectorToSkewMat(p)).transpose();
	X.block<3, 3>(3, 0) = MathTypes::Mat3::Zero();
	X.block<3, 3>(3, 3) = R.transpose();

	return X;
}

MathTypes::Mat6 SpatialTransform::GetSpatialFormForce()
{
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	//X.template topLeftCorner<3, 3>() = R;
	//X.template bottomRightCorner<3, 3>() = R;
	//X.template topRightCorner<3, 3>() = -R * VectorToSkewMat(p);
	
	X.block<3, 3>(0, 0) = R;
	X.block<3, 3>(0, 3) = -R * VectorToSkewMat(p);
	X.block<3, 3>(3, 0) = MathTypes::Mat3::Zero();
	X.block<3, 3>(3, 3) = R;

	return X;
}

MathTypes::Mat3 SpatialTransform::GetRotation() { return R; }
MathTypes::Vec3  SpatialTransform::GetTranslation() { return p; }


MathTypes::Vec6 SpatialTransform::Apply(MathTypes::Vec6 spVec)
{
	MathTypes::Vec3 v_rxw(
		spVec[3] - p[1] * spVec[2] + p[2] * spVec[1],
		spVec[4] - p[2] * spVec[0] + p[0] * spVec[2],
		spVec[5] - p[0] * spVec[1] + p[1] * spVec[0]
	);
	return MathTypes::Vec6(
		R(0, 0) * spVec[0] + R(0, 1) * spVec[1] + R(0, 2) * spVec[2],
		R(1, 0) * spVec[0] + R(1, 1) * spVec[1] + R(1, 2) * spVec[2],
		R(2, 0) * spVec[0] + R(2, 1) * spVec[1] + R(2, 2) * spVec[2],
		R(0, 0) * v_rxw[0] + R(0, 1) * v_rxw[1] + R(0, 2) * v_rxw[2],
		R(1, 0) * v_rxw[0] + R(1, 1) * v_rxw[1] + R(1, 2) * v_rxw[2],
		R(2, 0) * v_rxw[0] + R(2, 1) * v_rxw[1] + R(2, 2) * v_rxw[2]
	);
}

MathTypes::Vec6 SpatialTransform::ApplyTranspose(MathTypes::Vec6 spVec)
{
	MathTypes::Vec3 v_rxw(
		R(0, 0) * spVec[3] + R(1, 0) * spVec[4] + R(2, 0) * spVec[5],
		R(0, 1) * spVec[3] + R(1, 1) * spVec[4] + R(2, 1) * spVec[5],
		R(0, 2) * spVec[3] + R(1, 2) * spVec[4] + R(2, 2) * spVec[5]
	);
	return MathTypes::Vec6(
		R(0, 0) * spVec[0] + R(1, 0) * spVec[1] + R(2, 0) * spVec[2] - p[2] * v_rxw[1] + p[1] * v_rxw[2],
		R(0, 1) * spVec[0] + R(1, 1) * spVec[1] + R(2, 1) * spVec[2] + p[2] * v_rxw[0] - p[0] * v_rxw[2],
		R(0, 2) * spVec[0] + R(1, 2) * spVec[1] + R(2, 2) * spVec[2] - p[1] * v_rxw[0] + p[0] * v_rxw[1],
		v_rxw[0],
		v_rxw[1],
		v_rxw[2]
	);
}

#pragma region Operators

SpatialTransform SpatialTransform::operator*(const SpatialTransform& rhs) const
{

	//MathTypes::Mat3 newR = R * rhs.R;
	//MathTypes::Vec3 newp = p + R * rhs.p;

	MathTypes::Mat3 newR = R * rhs.R;
	MathTypes::Vec3 newp = rhs.p + rhs.R.transpose() * p;

	return SpatialTransform(newR, newp);
}

#pragma endregion
