#include <iostream>
#include "Quadruped/Quadruped.h"
#include "Quadruped/State.h"

int main()
{
	std::cout << "Starting Quadruped Test." << std::endl;
	Quadruped robot;
	robot.Initialize();

	State state;

	std::cout << "Initial State: \n";
	state.PrintState();

	for (int i = 0; i < 10; i++) {
		std::cout << "\nTimestep: " << i << std::endl;
		state = robot.StepDynamicsModel(state);
		state.PrintState();
	}


	//robotVis.f1();

	return 0;
}