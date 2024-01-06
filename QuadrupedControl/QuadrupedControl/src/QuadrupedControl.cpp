#include <iostream>

#include "QuadrupedControl.h"



QuadrupedControl::QuadrupedControl()
{
}

void QuadrupedControl::GetTorques(double* torques)
{
	for (int i = 0; i < 12; i++)
	{
		torques[i] = 0;
	}
}

//void SetRobotData(double* imuData, double* legData)
//{
//
//}
//
//int main()
//{
//	std::cout << "Main Func." << std::endl;
//
//	QuadrupedControl* controller = NULL;
//	//controller = new QuadrupedControl();
//
//	return 0;
//}