#ifndef SPATIALINERTIA_H
#define SPATIALINERTIA_H

#include "Types.h"

template<typename T>
class SpatialInertia
{
	SpatialInertia();
	SpatialInertia(T mass, const Vec3<T>& com, const Mat3<T>& inertia);
	~SpatialInertia();
private:
	MatSp<T> intertia;

};

#endif