#ifndef SPATIALINERTIA_H
#define SPATIALINERTIA_H

#include "Quadruped/Types.h"

template<typename T>
class SpatialInertia
{
public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SpatialInertia();
	SpatialInertia(T mass, const Vec3<T>& com, const Mat3<T>& inertia);
	~SpatialInertia();

	void AddInertia(MatSp<T> otherInertia);
	void SetInertia(MatSp<T> otherInertia);
	MatSp<T> GetInertia();
private:
	MatSp<T> inertia;

};

#endif