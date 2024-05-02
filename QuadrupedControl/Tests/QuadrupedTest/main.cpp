#include <iostream>
#include "Quadruped/Quadruped.h"

int main()
{
	std::cout << "Starting Quadruped Test." << std::endl;
	Quadruped robot;
	robot.Initialize();

	//robotVis.f1();

	return 0;
}