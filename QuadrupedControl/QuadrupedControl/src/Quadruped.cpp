#include "Quadruped.h"


template<typename T>
Quadruped<T>::Quadruped()
{

}

template<typename T>
Quadruped<T>::~Quadruped()
{

}


template<typename T>
void Quadruped<T>::SetFloatingBaseStateFromIMU(double IMUData[])
{

}


template<typename T>
void Quadruped<T>::SetJointsStateFromSensors(double jointStateData[])
{

}

template<typename T>
void Quadruped<T>::SetState(State<T>& newState)
{
	state = newState;
}

template<typename T>
void Quadruped<T>::SetState(double* bodyState, double* jointState)
{
	state.bodyPosition[0] = bodyState[0];
	state.bodyPosition[1] = bodyState[1];
	state.bodyPosition[2] = bodyState[2];

	state.bodyOrientation[0] = bodyState[3];
	state.bodyOrientation[1] = bodyState[4];
	state.bodyOrientation[2] = bodyState[5];
	state.bodyOrientation[3] = bodyState[6];

	for (int i = 0; i < robotParameters.numActuatedDoF; i++)
	{
		state.q[i] = jointState[i];
	}

	for (int i = 0; i < robotParameters.numActuatedDoF; i++)
	{
		state.qDot[i] = jointState[robotParameters.numActuatedDoF + i];
	}
}

template class Quadruped<double>;
template class Quadruped<float>;