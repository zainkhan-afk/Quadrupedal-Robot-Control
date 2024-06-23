//#include <iostream>
//
//#include "QuadrupedControl.h"
//#include "State.h"
//
//
//QuadrupedControl::QuadrupedControl()
//{
//}
//
//void QuadrupedControl::Initialize()
//{
//	robot.Initialize();
//}
//
//void QuadrupedControl::GetTorques(double imuData[], double motorData[], double* torques)
//{
//	robot.SetState(imuData, motorData);
//	State<float> state = robot.GetState();
//	state.PrintState();
//
//	for (int i = 0; i < 12; i++)
//	{
//		torques[i] = 0;
//	}
//}
//
//void QuadrupedControl::MoveLegsTo(double imuData[], double motorData[], double* q)
//{
//	robot.SetState(imuData, motorData);
//	Vec12<float> motorPositions = robot.LegPositionForState();
//
//	for (int i = 0; i < 12; i++)
//	{
//		q[i] = motorPositions[i];
//	}
//}