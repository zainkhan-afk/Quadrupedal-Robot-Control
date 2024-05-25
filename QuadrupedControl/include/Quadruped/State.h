#ifndef STATE_H
#define STATE_H

#include "Quadruped/Types.h"
#include "Quadruped/QuadrupedCommon.h"
#include <iostream>


struct QUADRUPED_API State
{
	const State& operator=(const State& rhs)
	{
		this->bodyPosition = rhs.bodyPosition;
		this->bodyOrientation = rhs.bodyOrientation;
		this->bodyVelocity = rhs.bodyVelocity;
		this->q = rhs.q;
		this->qDot = rhs.qDot;
		
		return *this;
	}
	MathTypes::Vec3 bodyPosition = MathTypes::Vec3::Zero();
	MathTypes::Vec3 bodyOrientation = MathTypes::Vec3::Zero();

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
	const StateDot& operator=(const StateDot& rhs)
	{
		this->bodyPositionDot = rhs.bodyPositionDot;
		this->bodyVelocityDDot = rhs.bodyVelocityDDot;
		this->qDDot = rhs.qDDot;

		return *this;
	}

	MathTypes::Vec3 bodyPositionDot = MathTypes::Vec3::Zero();;
	MathTypes::Vec6 bodyVelocityDDot = MathTypes::Vec6::Zero();;

	MathTypes::Vec12 qDDot = MathTypes::Vec12::Zero();;
};

#endif