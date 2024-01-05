#ifndef QUARDUPEDCONTROL_H
#define QUARDUPEDCONTROL_H


#include "Quadruped.h"
#include "Types.h"



class QuadrupedControl
{
public:
	QuadrupedControl();
private:
	Quadruped<float> robot;
};




extern "C" {

	QuadrupedControl* controller = NULL;

	// first step, init the controller
	void Init() {
		if (controller != NULL) {
			delete controller;
		}
		controller = new QuadrupedControl();
	}

	// after init controller and pre work, the mpc calculator can work
	double* torque_calculator(double imuData[], double motorData[]) {
		double eff[12] = { 0.0 };
		return eff;
	}
}

#endif