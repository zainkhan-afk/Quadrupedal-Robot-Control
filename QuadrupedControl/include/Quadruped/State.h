#ifndef STATE_H
#define STATE_H

#include "Quadruped/Types.h"
#include "Quadruped/QuadrupedCommon.h"
#include <iostream>


struct QUADRUPED_API State
{
	MathTypes::Vec3 bodyPosition = MathTypes::Vec3::Zero();
	MathTypes::Vec4 bodyOrientation = MathTypes::Vec4::Zero();

	MathTypes::Vec6 bodyVelocity = MathTypes::Vec6::Zero();

	MathTypes::Vec12 q = MathTypes::Vec12::Zero();;
	MathTypes::Vec12 qDot = MathTypes::Vec12::Zero();

	void PrintState()
	{
		std::cout << "Position - (" << bodyPosition[0] << ", " << bodyPosition[1] << ", " << bodyPosition[2] << ")\n";
	}
};


struct QUADRUPED_API StateDot
{
	MathTypes::Vec3 bodyPositionDot = MathTypes::Vec3::Zero();;
	MathTypes::Vec6 bodyVelocityDDot = MathTypes::Vec6::Zero();;

	MathTypes::Vec12 qDDot = MathTypes::Vec12::Zero();;
};

#endif