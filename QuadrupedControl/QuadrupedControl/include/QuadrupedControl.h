#ifndef QUARDUPEDCONTROL_H
#define QUARDUPEDCONTROL_H


#include "Quadruped.h"
#include "Types.h"



class QuadrupedControl
{
public:
	QuadrupedControl();

	void GetTorques(double* torques);
private:
	Quadruped<float> robot;
};




extern "C" 
{

	QuadrupedControl* controller = NULL;

	// first step, init the controller
	__declspec(dllexport) void Init(void)
	{
		if (controller != NULL) {
			delete controller;
		}
		controller = new QuadrupedControl();
	}

	//// after init controller and pre work, the mpc calculator can work
	__declspec(dllexport) double* GetTorques(double imuData[], double motorData[])
	{
		double eff[12] = { 0.0f };
		controller->GetTorques(eff);

		return eff;
	}
}

#endif