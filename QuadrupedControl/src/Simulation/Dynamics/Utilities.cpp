#include "Simulation/Dynamics/Utilities.h"
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

MathTypes::Mat3 GetRotationMatrix(double angle, COORD_AXIS axis)
{
	// Axis can be 0, 1 or 2 for x, y or z respectively.
	MathTypes::Mat3 rotation = MathTypes::Mat3::Identity();

	double s = std::sin(angle);
	double c = std::cos(angle);

	if (axis == COORD_AXIS::X)
	{
		rotation << 1, 0,  0, 
					0, c, -s, 
					0, s,  c;

	}
	else if (axis == COORD_AXIS::Y)
	{
		rotation <<  c, 0, s, 
					 0, 1, 0, 
					-s, 0, c;
	}
	else if (axis == COORD_AXIS::Z)
	{
		rotation << c, -s, 0, 
					s,  c, 0, 
					0,  0, 1;
	}

	return rotation;
}



MathTypes::Vec3 GetLegSignedVector(const MathTypes::Vec3& v, int legID) {
	switch (legID) {
	case 0:
		return MathTypes::Vec3( v[0], -v[1], v[2]);
		break;
	case 1:
		return MathTypes::Vec3( v[0],  v[1], v[2]);
		break;
	case 2:
		return MathTypes::Vec3(-v[0], -v[1], v[2]);
		break;
	case 3:
		return MathTypes::Vec3(-v[0],  v[1], v[2]);
		break;
	default:
		throw std::runtime_error("Invalid leg id!");
		break;
	}
}


MathTypes::Vec4 RotationMatrixToQuat(MathTypes::Mat3 R)
{
	MathTypes::Vec4 q;

	double t = R.trace();
	
	if (t > 0.0f)
	{

		t = sqrt(t + 1.0);
		q[3] = 0.5 * t;
		t = 0.5 / t;
		q[0] = (R(2, 1) - R(1, 2)) * t;
		q[1] = (R(0, 2) - R(2, 0)) * t;
		q[2] = (R(1, 0) - R(0, 1)) * t;

	}

	else {
		int i = 0;
		if (R(1, 1) > R(0, 0)) { i = 1; }
		if (R(2, 2) > R(i, i)) { i = 2; }
						
		int j = (i + 1) % 3;
		int k = (j + 1) % 3;
		
		t = sqrt(R(i, i) - R(j, j) - R(k, k) + 1);
		q[i] = 0.5 * t;
		t = 0.5 / t;
		q[3] = (R(k, j) - R(j, k)) * t;
		q[j] = (R(j, i) + R(i, j)) * t;
		q[k] = (R(k, i) + R(i, k)) * t;
	}
	
	/*MathTypes::Mat3 r = R.transpose();
	double tr = r.trace();
	if (tr > 0.0) {
		double S = sqrt(tr + 1.0) * 2.0;
		q(0) = 0.25 * S;
		q(1) = (r(2, 1) - r(1, 2)) / S;
		q(2) = (r(0, 2) - r(2, 0)) / S;
		q(3) = (r(1, 0) - r(0, 1)) / S;
	} else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
		double S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
		q(0) = (r(2, 1) - r(1, 2)) / S;
		q(1) = 0.25 * S;
		q(2) = (r(0, 1) + r(1, 0)) / S;
		q(3) = (r(0, 2) + r(2, 0)) / S;
	} else if (r(1, 1) > r(2, 2)) {
		double S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
		q(0) = (r(0, 2) - r(2, 0)) / S;
		q(1) = (r(0, 1) + r(1, 0)) / S;
		q(2) = 0.25 * S;
		q(3) = (r(1, 2) + r(2, 1)) / S;
	} else {
		double S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
		q(0) = (r(1, 0) - r(0, 1)) / S;
		q(1) = (r(0, 2) + r(2, 0)) / S;
		q(2) = (r(1, 2) + r(2, 1)) / S;
		q(3) = 0.25 * S;
	}*/
	
	return q;
}


MathTypes::Vec3 RotationMatrixToEuler(MathTypes::Mat3 R)
{
	MathTypes::Vec4 q = RotationMatrixToQuat(R);
	return QuatToEuler(q);
}


MathTypes::Mat3 QuatToRotationMatrix(MathTypes::Vec4 quat)
{
	MathTypes::Mat3 R;

	R <<	1 - 2 * (quat(2) * quat(2) + quat(3) * quat(3)),		2 * (quat(1) * quat(2) - quat(0) * quat(3)),		2 * (quat(0) * quat(2) + quat(1) * quat(3)),
				2 * (quat(1) * quat(2) + quat(0) * quat(3)),	1 - 2 * (quat(1) * quat(1) + quat(3) * quat(3)),		2 * (quat(2) * quat(3) - quat(0) * quat(1)),
				2 * (quat(1) * quat(3) - quat(0) * quat(2)),		2 * (quat(2) * quat(3) + quat(0) * quat(1)),	1 - 2 * (quat(1) * quat(1) + quat(2) * quat(2));
	R.transposeInPlace();
	return R;
}


MathTypes::Vec3 QuatToEuler(MathTypes::Vec4 q)
{

	MathTypes::Vec3 euler = MathTypes::Vec3::Zero();
	
	//float w = q[0];
	//float x = q[1];
	//float y = q[2];
	//float z = q[3];

	float x = q[0];
	float y = q[1];
	float z = q[2];
	float w = q[3];

	double t0 = +2.0 * (w * x + y * z);
	double t1 = +1.0 - 2.0 * (x * x + y * y);
	euler[0] = std::atan2(t0, t1);

	double t2 = +2.0 * (w * y - z * x);

	if (t2 > 1.0) { t2 = 1.0; }
	else if (t2 < -1.0) { t2 = -1.0; }
	euler[1] = std::asin(t2);

	double t3 = +2.0 * (w * z + x * y);
	double t4 = +1.0 - 2.0 * (y * y + z * z);
	euler[2] = std::atan2(t3, t4);

	/*double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
	euler(2) =
		std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
			q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	euler(1) = std::asin(as);
	euler(0) =
		std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
			q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);*/
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

