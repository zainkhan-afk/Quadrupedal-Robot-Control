#include "Quadruped.h"

template<typename T>
Quadruped<T>::Quadruped()
{
	 //legController = LegController<T>(robotParameters.abdLinkLength, robotParameters.hipLinkLength, robotParameters.kneeLinkLength, robotParameters.kneeLinkYOffset);
}

template<typename T>
Quadruped<T>::~Quadruped()
{
}

template<typename T>
void Quadruped<T>::Initialize()
{
	//legController = LegController<T>(robotParameters.abdLinkLength, robotParameters.hipLinkLength, robotParameters.kneeLinkLength, robotParameters.kneeLinkYOffset);
}

template<typename T>
void Quadruped<T>::Stand()
{

}


template<typename T>
void Quadruped<T>::SetFloatingBaseStateFromIMU(double IMUData[])
{

	state.bodyPosition[0] = IMUData[0];
	state.bodyPosition[1] = IMUData[1];
	state.bodyPosition[2] = IMUData[2];

	state.bodyOrientation[0] = IMUData[3];
	state.bodyOrientation[1] = IMUData[4];
	state.bodyOrientation[2] = IMUData[5];
	state.bodyOrientation[3] = IMUData[6];
}


template<typename T>
void Quadruped<T>::SetJointsStateFromSensors(double jointStateData[])
{
	for (int i = 0; i < robotParameters.numActuatedDoF; i++)
	{
		state.q[i] = jointStateData[i];
	}

	for (int i = 0; i < robotParameters.numActuatedDoF; i++)
	{
		state.qDot[i] = jointStateData[robotParameters.numActuatedDoF + i];
	}
}

template<typename T>
void Quadruped<T>::SetState(State<T>& newState)
{
	state = newState;
}

template<typename T>
void Quadruped<T>::SetState(double IMUData[], double jointStateData[])
{
	SetFloatingBaseStateFromIMU(IMUData);
	SetJointsStateFromSensors(jointStateData);

	Vec3<T> q = Vec3<T>(state.q[0], state.q[1], state.q[2]);

	Vec3<T> p1 = legController.ForwardKinematics(q, 0);
}

template<typename T>
State<T> Quadruped<T>::GetState()
{
	return state;
}

template class Quadruped<double>;
template class Quadruped<float>;