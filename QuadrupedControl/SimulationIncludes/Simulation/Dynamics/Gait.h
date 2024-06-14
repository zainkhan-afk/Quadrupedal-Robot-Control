#ifndef GAIT_H
#define GAIT_H

#include "Simulation/Dynamics/Types.h"
#include "Simulation/Dynamics/RobotDynamics.h"

#define _USE_MATH_DEFINES

class Gait
{
public:
	Gait();
	~Gait();

	State step(LegController& legController, State state, double dt);

private:
	std::vector<MathTypes::Vec3> gaitMatrix;
	double t = 0.0;
	double gaitTime = 2.0;
	double stepSize = 0.25;
	double stepHeight = 0.1;
};


#endif // !GAIT_H