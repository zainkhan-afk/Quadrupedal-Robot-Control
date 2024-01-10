#ifndef QUARDUPEDCONTROL_H
#define QUARDUPEDCONTROL_H


#include "Quadruped.h"
#include "Types.h"



class QuadrupedControl
{
public:
	QuadrupedControl();
	void Initialize();

	void GetTorques(double imuData[], double motorData[], double* torques);
private:
	Quadruped<float> robot;
};




extern "C" 
{
	QuadrupedControl* controller = NULL;

	__declspec(dllexport) void Init(void)
	{
		if (controller != NULL) {
			delete controller;
		}
		controller = new QuadrupedControl();
		controller->Initialize();
	}

	__declspec(dllexport) double* GetTorques(double imuData[], double motorData[])
	{
		double eff[12] = { 0.0f };
		controller->GetTorques(imuData, motorData, eff);

		return eff;
	}
}

#endif