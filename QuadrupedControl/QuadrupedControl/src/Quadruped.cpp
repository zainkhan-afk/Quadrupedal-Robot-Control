#include "Quadruped.h"
#include "Utilities.h"

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
	legController = LegController<T>(robotParameters.abdLinkLength, robotParameters.hipLinkLength, robotParameters.kneeLinkLength, robotParameters.kneeLinkYOffset);


	Mat3<T> rotorRotationalInertiaZ;
	rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
	rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

	Mat3<T> RY = GetRotationMatrix(PI / 2, 1);
	Mat3<T> RX = GetRotationMatrix(PI / 2, 0);

	Mat3<T> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
	Mat3<T> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

	bodyInertiaParams
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

	for (int i = 0; i < 4; i++)
	{
		Vec3<T> q = Vec3<T>(state.q[i * 3 + 0], state.q[i * 3 + 1], state.q[i * 3 + 2]);
		Vec3<T> p = legController.ForwardKinematics(q, i);
	}
}

template<typename T>
State<T> Quadruped<T>::GetState()
{
	return state;
}

template<typename T>
Vec12<T> Quadruped<T>::LegPositionForState()
{
	Vec12<T> q = Vec12<T>::Zero();
	Vec3<T> p = Vec3<T>::Zero();

	p[0] = 0;
	p[1] = robotParameters.abdLinkLength + robotParameters.kneeLinkYOffset;
	p[2] = -0.3f;

	for (int i = 0; i < 4; i++)
	{
		Vec3<T> motorPos = legController.InverseKinematics(p, i);

		q[i * 3 + 0] = motorPos[0];
		q[i * 3 + 1] = motorPos[1];
		q[i * 3 + 2] = motorPos[2];
	}

	return q;
}


template class Quadruped<double>;
template class Quadruped<float>;