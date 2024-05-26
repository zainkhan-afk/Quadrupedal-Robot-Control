#include "pch.h"
#include "Quadruped/SpatialTransform.h"
#include "Quadruped/Quadruped.h"
#include "Quadruped/Utilities.h"
#include "Quadruped/Spatial.h"
#include <math.h>



Quadruped::Quadruped()
{
	 //legController = LegController(robotParameters.abdLinkLength, robotParameters.hipLinkLength, robotParameters.kneeLinkLength, robotParameters.kneeLinkYOffset);
}


Quadruped::~Quadruped()
{
}


void Quadruped::Initialize()
{
	std::cout << "Initializing Quadruped." << std::endl;
	legController = LegController(robotParameters.abdLinkLength, robotParameters.hipLinkLength, robotParameters.kneeLinkLength, robotParameters.kneeLinkYOffset);

	
	MathTypes::Mat3 rotorRotationalInertiaZ;

	MathTypes::Mat3 RY = GetRotationMatrix(M_PI / 2, COORD_AXIS::Y);
	MathTypes::Mat3 RX = GetRotationMatrix(M_PI / 2, COORD_AXIS::X);

	// spatial inertias
	MathTypes::Mat3 abadRotationalInertia;
	abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
	abadRotationalInertia = abadRotationalInertia * 1e-6;
	MathTypes::Vec3 abadCOM(0, 0.036, 0);
	bodyInertiaParams.abdInertia = SpatialInertia(0.54f, abadCOM, abadRotationalInertia);

	MathTypes::Mat3 hipRotationalInertia;
	hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
	hipRotationalInertia = hipRotationalInertia * 1e-6;
	MathTypes::Vec3 hipCOM(0, 0.016, -0.02);
	bodyInertiaParams.hipInertia = SpatialInertia(0.634f, hipCOM, hipRotationalInertia);

	MathTypes::Mat3 kneeRotationalInertia, kneeRotationalInertiaRotated;
	kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
	kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
	kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
	MathTypes::Vec3 kneeCOM(0, 0, -0.061);
	bodyInertiaParams.kneeInertia = SpatialInertia(0.064f, kneeCOM, kneeRotationalInertia);

	MathTypes::Mat3 bodyRotationalInertia;
	bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
	bodyRotationalInertia = bodyRotationalInertia * 1e-6;
	MathTypes::Vec3 bodyCOM(0, 0, 0);
	bodyInertiaParams.floatingBodyInertia = SpatialInertia(robotParameters.bodyMass, bodyCOM, bodyRotationalInertia);


	// locations
	bodyInertiaParams.abdLocation = MathTypes::Vec3(robotParameters.bodyLength, robotParameters.bodyWidth, 0) * 0.5;
	bodyInertiaParams.hipLocation = MathTypes::Vec3(0, -robotParameters.abdLinkLength, 0);
	bodyInertiaParams.kneeLocation = MathTypes::Vec3(0, 0, -robotParameters.hipLinkLength);

	int baseID = 0;
	int bodyID = 0;
	int parentID = -1;

	SpatialTransform X;

	// Floating Base
	dynamics.AddBody(bodyInertiaParams.floatingBodyInertia, X, COORD_AXIS::X, parentID);
	bodyID++;
	parentID++;

	transformationChain.push_back(MathTypes::Mat4::Identity());
	
	MathTypes::Mat3 RIdent = MathTypes::Mat3::Identity();
	// Legs
	int side = -1;
	for (int leg = 0; leg < 4; leg++)
	{
		transformationChain.push_back(MathTypes::Mat4::Identity());
		transformationChain.push_back(MathTypes::Mat4::Identity());
		transformationChain.push_back(MathTypes::Mat4::Identity());

		X = SpatialTransform(RIdent, GetLegSignedVector(bodyInertiaParams.abdLocation, leg));
		// Abd
		if (side < 0) {
			dynamics.AddBody(bodyInertiaParams.abdInertia.FlipAlongAxis(1), X, COORD_AXIS::X, baseID);
		}
		else {
			dynamics.AddBody(bodyInertiaParams.abdInertia, X, COORD_AXIS::X, baseID);
		}
		bodyID++;
		parentID++;

		// Hip
		X = SpatialTransform(GetRotationMatrix(M_PI, COORD_AXIS::Z), GetLegSignedVector(bodyInertiaParams.hipLocation, leg));
		if (side < 0) {
			dynamics.AddBody(bodyInertiaParams.hipInertia.FlipAlongAxis(1), X, COORD_AXIS::Y, parentID);
		}
		else {
			dynamics.AddBody(bodyInertiaParams.hipInertia, X, COORD_AXIS::Y, parentID);
		}
		bodyID++;
		parentID++;

		// Knee
		X = SpatialTransform(RIdent, bodyInertiaParams.kneeLocation);
		if (side < 0) {
			dynamics.AddBody(bodyInertiaParams.kneeInertia.FlipAlongAxis(1), X, COORD_AXIS::Y, parentID);
		}
		else {
			dynamics.AddBody(bodyInertiaParams.kneeInertia, X, COORD_AXIS::Y, parentID);
		}
		bodyID++;
		parentID++;

		side *= -1;
	}
}



void Quadruped::SetFloatingBaseStateFromIMU(float IMUData[])
{

	state.bodyPosition[0] = IMUData[0];
	state.bodyPosition[1] = IMUData[1];
	state.bodyPosition[2] = IMUData[2];

	state.bodyOrientation[0] = IMUData[3];
	state.bodyOrientation[1] = IMUData[4];
	state.bodyOrientation[2] = IMUData[5];
}



void Quadruped::SetJointsStateFromSensors(float jointStateData[])
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


void Quadruped::SetState(const State& newState)
{
	state = newState;
}


void Quadruped::SetState(float IMUData[], float jointStateData[])
{
	SetFloatingBaseStateFromIMU(IMUData);
	SetJointsStateFromSensors(jointStateData);

	for (int i = 0; i < 4; i++)
	{
		MathTypes::Vec3 q = MathTypes::Vec3(state.q[i * 3 + 0], state.q[i * 3 + 1], state.q[i * 3 + 2]);
		MathTypes::Vec3 p = legController.ForwardKinematics(q, i);
	}
}

void Quadruped::SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces)
{
	dynamics.SetExternalForces(externalForces);
}

void Quadruped::SetExternalForceAt(int i, const MathTypes::Vec6& externalForce)
{
	dynamics.SetExternalForceAt(i, externalForce);
}


State Quadruped::GetState()
{
	return state;
}

void Quadruped::Integrate(State& state,const StateDot& dstate)
{
	state.bodyVelocity += dstate.bodyVelocityDDot * deltaT;
	state.bodyPosition += state.bodyVelocity * deltaT;

	//state.bodyPosition += state.bodyVelocity.template block<3, 1>(3, 0) * deltaT;

	for (int i = 0; i < 12; i++)
	{
		float prevQ = state.q[i];
		state.qDot[i] += dstate.qDDot[i] * deltaT;
		state.q[i] += state.qDot[i] * deltaT;

		dynamics.torques[i] = (prevQ - state.q[i]) * 0.5;
	}
}

void Quadruped::GetVisualTransformations(const State& state)
{
	MathTypes::Mat3  R = GetRotationMatrix(state.bodyOrientation[0], COORD_AXIS::X) * GetRotationMatrix(state.bodyOrientation[1], COORD_AXIS::Y) * GetRotationMatrix(state.bodyOrientation[2], COORD_AXIS::Z);
	MathTypes::Vec3 position = state.bodyPosition;

	transformationChain[0].template topLeftCorner<3, 3>() = R;
	transformationChain[0](0, 3) = position[0];
	transformationChain[0](1, 3) = position[1];
	transformationChain[0](2, 3) = position[2];

	for (int i = 1; i < dynamics.numLinks; i++)
	{
		MathTypes::Mat4 T = MathTypes::Mat4::Identity();

		MathTypes::Mat4 TpjTrans = MathTypes::Mat4::Identity();
		MathTypes::Mat4 TpjRot = MathTypes::Mat4::Identity();
		MathTypes::Mat4 Tj = MathTypes::Mat4::Identity();

		MathTypes::Vec3 t = dynamics.Xl[i].GetTranslation();

		TpjRot.template topLeftCorner<3, 3>() = dynamics.Xl[i].GetRotation();
		TpjTrans(0, 3) = t[0];
		TpjTrans(1, 3) = t[1];
		TpjTrans(2, 3) = t[2];


		Tj.template topLeftCorner<3, 3>() = GetRotationMatrix(state.q[i - 1], dynamics.axis[i]);

		transformationChain[i] = transformationChain[dynamics.parents[i]] * TpjRot * TpjTrans * Tj;
	}
}


State Quadruped::StepDynamicsModel(State& state)
{
	StateDot dState = dynamics.Step(state);

	Integrate(state, dState);

	return state;
}