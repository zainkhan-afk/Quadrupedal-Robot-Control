#ifndef STATE_H
#define STATE_H

#include "Quadruped/Types.h"
#include <iostream>

template <typename T>
struct State
{
	Vec3<T> bodyPosition;
	Quat<T> bodyOrientation;

	VecSp<T> bodyVelocity;

	Vec12<T> q;
	Vec12<T> qDot;

	void PrintState()
	{
		std::cout << "Position - (" << bodyPosition[0] << ", " << bodyPosition[1] << ", " << bodyPosition[2] << ")\n";
	}
};

template <typename T>
struct StateDot
{
	Vec3<T> bodyPositionDot;
	VecSp<T> bodyVelocityDDot;

	Vec12<T> qDDot;
};

#endif