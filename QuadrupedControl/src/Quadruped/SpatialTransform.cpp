#include "pch.h"
#include "Quadruped/SpatialTransform.h"
#include "Quadruped/Utilities.h"

SpatialTransform::SpatialTransform()
{
	R = MathTypes::Mat3::Identity();
	p = MathTypes::Vec3::Zero();
}

SpatialTransform::SpatialTransform(const MathTypes::Mat3& _R, const MathTypes::Vec3& _p) : R(-R), p(_p)
{
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

SpatialTransform SpatialTransform::operator*(const SpatialTransform& rhs) const
{

	return SpatialTransform(R * rhs.R, rhs.p + rhs.R.transpose() * p);
}
