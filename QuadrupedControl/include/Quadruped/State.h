#ifndef STATE_H
#define STATE_H

#include "Quadruped/Types.h"
#include "Quadruped/QuadrupedCommon.h"
#include <iostream>


struct QUADRUPED_API State
{
	dtypes::Vec3 bodyPosition = dtypes::Vec3::Zero();
	dtypes::Vec4 bodyOrientation = dtypes::Vec4::Zero();

	dtypes::Vec6 bodyVelocity = dtypes::Vec6::Zero();

	dtypes::Vec12 q = dtypes::Vec12::Zero();;
	dtypes::Vec12 qDot = dtypes::Vec12::Zero();

	void PrintState()
	{
		std::cout << "Position - (" << bodyPosition[0] << ", " << bodyPosition[1] << ", " << bodyPosition[2] << ")\n";
	}
};


struct QUADRUPED_API StateDot
{
	dtypes::Vec3 bodyPositionDot = dtypes::Vec3::Zero();;
	dtypes::Vec6 bodyVelocityDDot = dtypes::Vec6::Zero();;

	dtypes::Vec12 qDDot = dtypes::Vec12::Zero();;
};

#endif