#include <iostream>
#include "Quadruped/Quadruped.h"
#include "Quadruped/State.h"

int main()
{
	std::cout << "Starting Quadruped Test." << std::endl;
	Quadruped robot;
	robot.Initialize();

	State state;

	state = robot.StepDynamicsModel(state);

	//robotVis.f1();

	return 0;
}