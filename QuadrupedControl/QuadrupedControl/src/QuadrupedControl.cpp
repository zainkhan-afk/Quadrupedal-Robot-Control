#include <iostream>

#include "QuadrupedControl.h"
#include "State.h"


QuadrupedControl::QuadrupedControl()
{
}

void QuadrupedControl::GetTorques(double imuData[], double motorData[], double* torques)
{
	robot.SetState(imuData, motorData);
	State<float> state = robot.GetState();
	state.PrintState();
	//std::cout << "Getting Torques." << std::endl;
	for (int i = 0; i < 12; i++)
	{
		torques[i] = 0;
	}
}