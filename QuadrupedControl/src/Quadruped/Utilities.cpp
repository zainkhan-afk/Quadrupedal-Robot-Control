#include "pch.h"
#include "Quadruped/Utilities.h"
#include <math.h>

MathTypes::Mat3 VectorToSkewMat(const MathTypes::Vec3& v) {
	MathTypes::Mat3 m;
	m <<    0, -v[2],  v[1], 
		 v[2],     0, -v[0], 
		-v[1],  v[0],    0;
	return m;
}

MathTypes::Vec3 SkewMatToVecor(const MathTypes::Mat3& m)
{
	MathTypes::Vec3 v;
	v << m(2, 1) - m(1, 2),
   		 m(0, 2) - m(2, 0),
		 m(1, 0) - m(0, 1);

	return 0.5f * v;
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



MathTypes::Vec3 GetLegSignedVector(const MathTypes::Vec3& v, int legID) {
	switch (legID) {
	case 0:
		return MathTypes::Vec3( v[0], -v[1], v[2]);
	case 1:
		return MathTypes::Vec3( v[0],  v[1], v[2]);
	case 2:
		return MathTypes::Vec3(-v[0], -v[1], v[2]);
	case 3:
		return MathTypes::Vec3(-v[0],  v[1], v[2]);
	default:
		throw std::runtime_error("Invalid leg id!");
	}
}


MathTypes::Vec4 RotationMatrixToQuat(MathTypes::Mat3 R)
{
	MathTypes::Vec4 q;
	MathTypes::Mat3 R_T = R.transpose();




	return q;
	Quat<typename T::Scalar> q;
	Mat3<typename T::Scalar> r = r1.transpose();
	typename T::Scalar tr = r.trace();
	if (tr > 0.0) {
		typename T::Scalar S = sqrt(tr + 1.0) * 2.0;
		q(0) = 0.25 * S;
		q(1) = (r(2, 1) - r(1, 2)) / S;
		q(2) = (r(0, 2) - r(2, 0)) / S;
		q(3) = (r(1, 0) - r(0, 1)) / S;
	}
	else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
		typename T::Scalar S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
		q(0) = (r(2, 1) - r(1, 2)) / S;
		q(1) = 0.25 * S;
		q(2) = (r(0, 1) + r(1, 0)) / S;
		q(3) = (r(0, 2) + r(2, 0)) / S;
	}
	else if (r(1, 1) > r(2, 2)) {
		typename T::Scalar S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
		q(0) = (r(0, 2) - r(2, 0)) / S;
		q(1) = (r(0, 1) + r(1, 0)) / S;
		q(2) = 0.25 * S;
		q(3) = (r(1, 2) + r(2, 1)) / S;
	}
	else {
		typename T::Scalar S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
		q(0) = (r(1, 0) - r(0, 1)) / S;
		q(1) = (r(0, 2) + r(2, 0)) / S;
		q(2) = (r(1, 2) + r(2, 1)) / S;
		q(3) = 0.25 * S;
	}
	return q;
}

MathTypes::Mat3 QuatToRotationMatrix(MathTypes::Vec4 quat)
{
	MathTypes::Mat3 R;

	R <<	1 - 2 * (quat(2) * quat(2) + quat(3) * quat(3)),		2 * (quat(1) * quat(2) - quat(0) * quat(3)),		2 * (quat(1) * quat(3) + quat(0) * quat(2)),
				2 * (quat(1) * quat(2) + quat(0) * quat(3)),	1 - 2 * (quat(1) * quat(1) + quat(3) * quat(3)),		2 * (quat(2) * quat(3) - quat(0) * quat(1)),
				2 * (quat(1) * quat(3) - quat(0) * quat(2)),		2 * (quat(2) * quat(3) + quat(0) * quat(1)),	1 - 2 * (quat(1) * quat(1) + quat(2) * quat(2));
	R.transposeInPlace();
	return R;
}


MathTypes::Vec3 QuatToEuler(MathTypes::Vec4 q)
{
	MathTypes::Vec3 euler = MathTypes::Vec3::Zero();
	
	// roll (x-axis rotation)
	float sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
	float cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
	euler[0] = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	float sinp = std::sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]));
	float cosp = std::sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]));
	euler[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2;

	// yaw (z-axis rotation)
	float siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
	float cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
	euler[2] = std::atan2(siny_cosp, cosy_cosp);

	return euler;
}

MathTypes::Vec4 EulerToQuat(MathTypes::Vec3 euler)
{
	MathTypes::Vec4 quat = MathTypes::Vec4::Zero();


	double cr = cos(euler[0] * 0.5);
	double sr = sin(euler[0] * 0.5);
	double cp = cos(euler[1] * 0.5);
	double sp = sin(euler[1] * 0.5);
	double cy = cos(euler[2] * 0.5);
	double sy = sin(euler[2] * 0.5);

	
	quat[0] = cr * cp * cy + sr * sp * sy;
	quat[1] = sr * cp * cy - cr * sp * sy;
	quat[2] = cr * sp * cy + sr * cp * sy;
	quat[3] = cr * cp * sy - sr * sp * cy;

	return quat;
}