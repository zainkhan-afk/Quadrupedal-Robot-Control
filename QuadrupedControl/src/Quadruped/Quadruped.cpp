#include "pch.h"
#include "Quadruped/Quadruped.h"
#include "Quadruped/Utilities.h"
#include "Spatial.h"

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
	std::cout << "Initializing Quadruped." << std::endl;
	legController = LegController<T>(robotParameters.abdLinkLength, robotParameters.hipLinkLength, robotParameters.kneeLinkLength, robotParameters.kneeLinkYOffset);

	
	Mat3<T> rotorRotationalInertiaZ;
	rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
	rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

	Mat3<T> RY = GetRotationMatrix<T>(PI / 2.0f, 1);
	Mat3<T> RX = GetRotationMatrix<T>(PI / 2.0f, 0);

	Mat3<T> rotorRotationalInertiaX = RY * rotorRotationalInertiaZ * RY.transpose();
	Mat3<T> rotorRotationalInertiaY = RX * rotorRotationalInertiaZ * RX.transpose();

	// spatial inertias
	Mat3<T> abadRotationalInertia;
	abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
	abadRotationalInertia = abadRotationalInertia * 1e-6;
	Vec3<T> abadCOM(0, 0.036, 0); 
	bodyInertiaParams.abdInertia = SpatialInertia<T>(0.54f, abadCOM, abadRotationalInertia);

	Mat3<T> hipRotationalInertia;
	hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
	hipRotationalInertia = hipRotationalInertia * 1e-6;
	Vec3<T> hipCOM(0, 0.016, -0.02);
	bodyInertiaParams.hipInertia = SpatialInertia<T>(0.634f, hipCOM, hipRotationalInertia);

	Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
	kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
	kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
	kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
	Vec3<T> kneeCOM(0, 0, -0.061);
	bodyInertiaParams.kneeInertia = SpatialInertia<T>(0.064f, kneeCOM, kneeRotationalInertia);

	Vec3<T> rotorCOM(0, 0, 0);
	SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
	SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

	Mat3<T> bodyRotationalInertia;
	bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
	bodyRotationalInertia = bodyRotationalInertia * 1e-6;
	Vec3<T> bodyCOM(0, 0, 0);
	bodyInertiaParams.floatingBodyInertia = SpatialInertia<T>(robotParameters.bodyMass, bodyCOM, bodyRotationalInertia);


	// locations
	bodyInertiaParams.abdRotorLocation = Vec3<T>(0.125, 0.049, 0);
	bodyInertiaParams.abdLocation = Vec3<T>(robotParameters.bodyLength, robotParameters.bodyWidth, 0) * 0.5;
	bodyInertiaParams.hipLocation = Vec3<T>(0, robotParameters.abdLinkLength, 0);
	bodyInertiaParams.hipRotorLocation = Vec3<T>(0, 0.04, 0);
	bodyInertiaParams.kneeLocation = Vec3<T>(0, 0, -robotParameters.hipLinkLength);
	bodyInertiaParams.kneeRotorLocation = Vec3<T>(0, 0, 0);


	Mat6<T> abdPosition = 
	dynamics.AddBody()
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


template<typename T>
Vec18<T> Quadruped<T>::GetGravityVector()
{
	Vec18<T> G = Vec18<T>::Zero();
	return G;
}

template<typename T>
Mat18<T> Quadruped<T>::GetCoriolisMatrix()
{
	Mat18<T> C = Mat18<T>::Zero();
	return C;
}

template<typename T>
Mat18<T> Quadruped<T>::GetMassMatrix()
{
	Mat18<T> M = Mat18<T>::Zero();
	return M;
}


template class __declspec(dllexport) Quadruped<double>;
template class __declspec(dllexport) Quadruped<float>;