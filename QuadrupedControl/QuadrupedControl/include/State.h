#ifndef STATE_H
#define STATE_H

#include "Types.h"

template <typename T>
struct State
{
	Vec3 bodyPosition;
	Quat bodyOrientation;

	VecSp bodyVelocity;

	Vec12 q;
	Vec12 qDot;
};

struct State_dot
{
	Vec3 bodyPositionDot;
	VecSp bodyVelocityDDot;

	Vec12 qDDot;
};

#endif