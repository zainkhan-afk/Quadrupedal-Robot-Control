#ifndef QUARDUPED_H
#define QUARDUPED_H

#include "State.h"
#include "LegController.h"

/*
* The quadruped has 4 legs, the legs are arranged in the follwing way when viewed from top.
* 
*						Front
* 
* 
* 
* 
*						Head
*				Leg 2			Leg 1
* 
* 
*Left											Right
* 
* 
*				Leg 3			Leg 4
* 
* 
* 
* 
* 
*						Back
*/


template <typename T>
class Quadruped
{
public:
	Quadruped();
	~Quadruped();

	void Initialize();
	void Stand();


	void SetFloatingBaseStateFromIMU(double IMUData[]);
	void SetJointsStateFromSensors(double jointStateData[]);
	void SetState(State<T>& newState);
	void SetState(double IMUData[], double jointStateData[]);

	State<T> GetState();
	Vec12<T> LegPositionForState();

private:
	struct 
	{
		int numDoF{ 18 }, numActuatedDoF{ 12 }, numUnActuatedDoF{ 6 };

		T bodyLength{ 0.38 }, bodyWidth{ 0.098 }, bodyHeight{ 0.1 };
		T abdGearRatio{ 6 }, hipGearRatio{ 6 }, kneeGearRatio{ 9.33 };
		
		T abdLinkLength{ 0.062 }, hipLinkLength{ 0.209 }, kneeLinkLength{ 0.195 }, kneeLinkYOffset{ 0.004 }, maxLegLength{ 0.409 };

		T bodyMass{ 3.3 };

	} robotParameters;

	State<T> state;
	StateDot<T> stateDot;
	LegController<T> legController;
};


#endif
