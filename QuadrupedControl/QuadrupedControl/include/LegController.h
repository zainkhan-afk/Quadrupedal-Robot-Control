#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "Quadruped.h"

template<typename T>
class LegController
{
public:
	LegController() {}
	LegController(T _l1, T _l2, T _l3, T _l4);
	~LegController() {}

	Vec3<T> ForwardKinematics(int leg);
	Vec3<T> InverseKinematics(Vec3<T> pos, int leg);
	Mat3<T> GetLegJacobian(int leg);

private:
	T l1, l2, l3, l4;
};

#endif