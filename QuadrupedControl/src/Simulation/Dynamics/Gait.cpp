#include "Simulation/Dynamics/Gait.h"
#include <math.h>

Gait::Gait()
{
	gaitMatrix.push_back(MathTypes::Vec3::Zero());
	gaitMatrix.push_back(MathTypes::Vec3::Zero());
	gaitMatrix.push_back(MathTypes::Vec3::Zero());
	gaitMatrix.push_back(MathTypes::Vec3::Zero());

	gaitMatrix[0][0] = 0.1;
	gaitMatrix[0][1] = 0.9;
	gaitMatrix[0][2] = 0.0;

	gaitMatrix[1][0] = 0.1;
	gaitMatrix[1][1] = 0.9;
	gaitMatrix[1][2] = 0.0;

	gaitMatrix[2][0] = 0.0;
	gaitMatrix[2][1] = 0.9;
	gaitMatrix[2][2] = 0.1;

	gaitMatrix[3][0] = 0.0;
	gaitMatrix[3][1] = 0.9;
	gaitMatrix[3][2] = 0.1;
}

Gait::~Gait()
{

}

State Gait::step(LegController& legController, State state, double dt)
{
	for (int i = 0; i < 4; i++)
	{
		double seg1 = gaitMatrix[i][0] * gaitTime;
		double seg2 = gaitMatrix[i][1] * gaitTime + seg1;
		double seg3 = gaitMatrix[i][2] * gaitTime + seg2;

		MathTypes::Vec3 pos = MathTypes::Vec3::Zero();

		if (t < seg1 && t > 0)
		{
			double swingT = t / (gaitMatrix[i][0] * gaitTime) + gaitMatrix[i][2];
			pos[0] = swingT * stepSize - stepSize / 2.0;
			pos[1] = 0.004;
			pos[2] = -0.3 + stepHeight * sin(swingT * 2 * M_PI * (gaitMatrix[i][0] + gaitMatrix[i][2]));
		}

		else if (t < seg2 && t >= seg1)
		{
			double contactT = (t - seg1) / (gaitMatrix[i][1] * gaitTime);
			pos[0] = -contactT * stepSize + stepSize / 2.0;
			pos[1] = 0.004;
			pos[2] = -0.3;
		}

		else if (t < seg3 && t >= seg2)
		{
			double swingT = (t - seg2) / (gaitMatrix[i][2] * gaitTime);
			pos[0] = swingT * stepSize - stepSize / 2.0;
			pos[1] = 0.004;
			pos[2] = -0.3 + stepHeight * sin(swingT * 2 * M_PI * (gaitMatrix[i][0] + gaitMatrix[i][2]));
		}

		MathTypes::Vec3 angles = legController.InverseKinematics(pos, i);
		state.q[i * 3 + 0] = angles[0];
		state.q[i * 3 + 1] = angles[1];
		state.q[i * 3 + 2] = angles[2];
	}

	t += dt;
	if (t >= gaitTime) { t = 0.0; }

	return state;
}