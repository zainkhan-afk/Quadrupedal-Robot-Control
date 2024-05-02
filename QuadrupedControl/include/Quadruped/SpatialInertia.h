#ifndef SPATIALINERTIA_H
#define SPATIALINERTIA_H

#include "Quadruped/Types.h"


class SpatialInertia
{
public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SpatialInertia();
	SpatialInertia(float mass, const dtypes::Vec3& com, const dtypes::Mat3& inertia);
	SpatialInertia(const dtypes::Mat4&);
	~SpatialInertia();

	dtypes::Mat4 GetPseudoInertia();
	SpatialInertia FlipAlongAxis(int axis);

	void AddInertia(dtypes::MatSp otherInertia);
	void SetInertia(dtypes::MatSp otherInertia);
	dtypes::MatSp GetInertia();
private:
	dtypes::MatSp inertia;

};

#endif