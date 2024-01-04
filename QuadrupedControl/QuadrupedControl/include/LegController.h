#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "Quadruped.h"

template<typename T>
class LegController
{
public:
	LegController() {}
	~LegController() {}

	Vec3<T> ForwardKinematics(Quadruped<T>& quad, int leg);
	Vec3<T> InverseKinematics(Vec3<T> pos, int leg);
	Mat3<T> GetLegJacobian(Quadruped<T>& quad, int leg);
};

#endif