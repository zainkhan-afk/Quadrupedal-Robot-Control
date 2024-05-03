#include "pch.h"
#include "Quadruped/Utilities.h"


MathTypes::Mat3 VectorToSkewMat(const MathTypes::Vec3& v) {
	MathTypes::Mat3 m;
	m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
	return m;
}


MathTypes::Mat3 GetRotationMatrix(float angle, COORD_AXIS axis)
{
	// Axis can be 0, 1 or 2 for x, y or z respectively.
	MathTypes::Mat3 rotation;

	float s = std::sin(angle);
	float c = std::cos(angle);

	if (axis == COORD_AXIS::X)
	{
		rotation << 1, 0, 0, 0, c, s, 0, -s, c;
	}
	else if (axis == COORD_AXIS::Y)
	{
		rotation << c, 0, -s, 0, 1, 0, s, 0, c;
	}
	else if (axis == COORD_AXIS::Z)
	{
		rotation << c, s, 0, -s, c, 0, 0, 0, 1;
	}

	return rotation;
}


MathTypes::Vec3 MatToSkewVec(const MathTypes::Mat3& m)
{
	return 0.5 * MathTypes::Vec3(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), (m(1, 0) - m(0, 1)));
}



MathTypes::Vec3 GetLegSignedVector(const MathTypes::Vec3& v, int legID) {
	switch (legID) {
	case 0:
		return MathTypes::Vec3(v[0], -v[1], v[2]);
	case 1:
		return MathTypes::Vec3(v[0], v[1], v[2]);
	case 2:
		return MathTypes::Vec3(-v[0], -v[1], v[2]);
	case 3:
		return MathTypes::Vec3(-v[0], v[1], v[2]);
	default:
		throw std::runtime_error("Invalid leg id!");
	}
}

