#ifndef STATE_H
#define STATE_H

#include "Quadruped/Types.h"
#include <iostream>


struct State
{
	dtypes::Vec3 bodyPosition;
	dtypes::Quat bodyOrientation;

	dtypes::VecSp bodyVelocity;

	dtypes::Vec12 q;
	dtypes::Vec12 qDot;

	void PrintState()
	{
		std::cout << "Position - (" << bodyPosition[0] << ", " << bodyPosition[1] << ", " << bodyPosition[2] << ")\n";
	}
};


struct StateDot
{
	dtypes::Vec3 bodyPositionDot;
	dtypes::VecSp bodyVelocityDDot;

	dtypes::Vec12 qDDot;
};

#endif