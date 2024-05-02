#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H


#include "Quadruped/Types.h"


class LegController
{
public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	LegController();
	LegController(float abdLinkLength, float hipLinkLength, float kneeLinkLength, float kneeLinkYOffset);
	~LegController() {}

	dtypes::Vec3 ForwardKinematics(dtypes::Vec3 q, int leg);
	dtypes::Vec3 InverseKinematics(dtypes::Vec3 pos, int leg);
	dtypes::Mat3 GetLegJacobian(dtypes::Vec3 q, int leg);

private:
	float l1, l2, l3, l4, l14;
};

#endif