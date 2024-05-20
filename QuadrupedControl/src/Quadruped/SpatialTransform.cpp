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
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	X.template bottomLeftCorner<3, 3>() = -R * VectorToSkewMat(p);

	return X;
}

MathTypes::Mat6 SpatialTransform::GetSpatialFormTranspose()
{
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R.transpose();
	X.template bottomRightCorner<3, 3>() = R.transpose();
	X.template topRightCorner<3, 3>() = (-R * VectorToSkewMat(p)).transpose();

	return X;
}

MathTypes::Mat6 SpatialTransform::GetSpatialFormForce()
{
	MathTypes::Mat6 X = MathTypes::Mat6::Zero();
	X.template topLeftCorner<3, 3>() = R;
	X.template bottomRightCorner<3, 3>() = R;
	X.template topRightCorner<3, 3>() = -R * VectorToSkewMat(p);

	return X;
}

MathTypes::Mat3 SpatialTransform::GetRotation() { return R; }
MathTypes::Vec3  SpatialTransform::GetTranslation() { return p; }

#pragma region Operators

SpatialTransform SpatialTransform::operator*(const SpatialTransform& rhs) const
{

	MathTypes::Mat3 newR = R * rhs.R;
	MathTypes::Vec3 newp = rhs.p + rhs.R.transpose() * p;
	return SpatialTransform(newR, newp);

	/*MathTypes::Mat3 newR = R * rhs.R;
	MathTypes::Vec3 newp = p + R * rhs.p;
	return SpatialTransform(newR, newp);*/
}



#pragma endregion
