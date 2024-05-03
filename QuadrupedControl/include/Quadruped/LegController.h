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

	MathTypes::Vec3 ForwardKinematics(MathTypes::Vec3 q, int leg);
	MathTypes::Vec3 InverseKinematics(MathTypes::Vec3 pos, int leg);
	MathTypes::Mat3 GetLegJacobian(MathTypes::Vec3 q, int leg);

private:
	float l1, l2, l3, l4, l14;
};

#endif