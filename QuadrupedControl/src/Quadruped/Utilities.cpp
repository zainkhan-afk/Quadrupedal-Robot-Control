#include "pch.h"
#include "Quadruped/Utilities.h"


dtypes::Mat3 VectorToSkewMat(const dtypes::Vec3& v) {
	dtypes::Mat3 m;
	m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
	return m;
}


dtypes::Mat3 GetRotationMatrix(float angle, int axis)
{
	// Axis can be 0, 1 or 2 for x, y or z respectively.
	dtypes::Mat3 rotation;

	float s = std::sin(angle);
	float c = std::cos(angle);

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


dtypes::Vec3 MatToSkewVec(const dtypes::Mat3& m)
{
	return 0.5 * dtypes::Vec3(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), (m(1, 0) - m(0, 1)));
}



dtypes::Vec3 GetLegSignedVector(const dtypes::Vec3& v, int legID) {
	switch (legID) {
	case 0:
		return dtypes::Vec3(v[0], -v[1], v[2]);
	case 1:
		return dtypes::Vec3(v[0], v[1], v[2]);
	case 2:
		return dtypes::Vec3(-v[0], -v[1], v[2]);
	case 3:
		return dtypes::Vec3(-v[0], v[1], v[2]);
	default:
		throw std::runtime_error("Invalid leg id!");
	}
}

