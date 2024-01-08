#ifndef QUARDUPED_H
#define QUARDUPED_H

#include "State.h"
#include "LegController.h"

template <typename T>
class Quadruped
{
public:
	Quadruped();
	~Quadruped();

	void SetFloatingBaseStateFromIMU(double IMUData[]);
	void SetJointsStateFromSensors(double jointStateData[]);
	void SetState(State<T>& newState);
	void SetState(double IMUData[], double jointStateData[]);

	State<T> GetState();

private:
	struct 
	{
		int numDoF{ 18 }, numActuatedDoF{ 12 }, numUnActuatedDoF{ 6 };

	} robotParameters;


	State<T> state;
	StateDot<T> stateDot;
};


#endif
