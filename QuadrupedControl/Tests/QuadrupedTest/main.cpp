#include <iostream>
#include "Quadruped/Quadruped.h"

int main()
{
	std::cout << "Starting Quadruped Test." << std::endl;
	Quadruped<float> robot;
	robot.Initialize();


	robot.CustomTests();

	//robotVis.f1();

	return 0;
}