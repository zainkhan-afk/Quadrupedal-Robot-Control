#ifndef SPATIALINERTIA_H
#define SPATIALINERTIA_H

#include "Quadruped/Types.h"


class SpatialInertia
{
public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SpatialInertia();
	SpatialInertia(float mass, const MathTypes::Vec3& com, const MathTypes::Mat3& inertia);
	SpatialInertia(const MathTypes::Mat4&);
	~SpatialInertia();

	MathTypes::Mat4 GetPseudoInertia();
	SpatialInertia FlipAlongAxis(int axis);

	void AddInertia(MathTypes::Mat6 otherInertia);
	void SetInertia(MathTypes::Mat6 otherInertia);
	MathTypes::Mat6 GetInertia();
private:
	MathTypes::Mat6 inertia;

};

#endif