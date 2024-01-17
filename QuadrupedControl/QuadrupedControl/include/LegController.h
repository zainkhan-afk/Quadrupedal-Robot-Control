#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H


#include "Types.h"

template<typename T>
class LegController
{
public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	LegController();
	LegController(T abdLinkLength, T hipLinkLength, T kneeLinkLength, T kneeLinkYOffset);
	~LegController() {}

	Vec3<T> ForwardKinematics(Vec3<T> q, int leg);
	Vec3<T> InverseKinematics(Vec3<T> pos, int leg);
	Mat3<T> GetLegJacobian(Vec3<T> q, int leg);

private:
	T l1, l2, l3, l4, l14;
};

#endif