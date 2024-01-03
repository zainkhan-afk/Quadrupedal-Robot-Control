#ifndef STATE_H
#define STATE_H

#include "Types.h"

template <typename T>
struct State
{
	Vec3<T> bodyPosition;
	Quats<T> bodyOrientation;

	VecSp<T> bodyVelocity;

	Vec12<T> q;
	Vec12<T> qDot;
};

template <typename T>
struct StateDot
{
	Vec3<T> bodyPositionDot;
	VecSp<T> bodyVelocityDDot;

	Vec12<T> qDDot;
};

#endif