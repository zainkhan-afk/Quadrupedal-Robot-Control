#include "pch.h"
#include "Quadruped/CubeChain.h"
#include "Quadruped/Spatial.h"
#include "Quadruped/SpatialInertia.h"


CubeChain::CubeChain()
{

}

CubeChain::~CubeChain()
{

}

void CubeChain::Initialize()
{
	MathTypes::Mat3 linkRotationalInertia;
	linkRotationalInertia <<	0.006042f, 0.0f, 0.0f,
								0.0f, 0.006042f, 0.0f,
								0.0f, 0.0f, 0.0016667f;

	SpatialInertia linkSpatialInertial1(linkMass, MathTypes::Vec3::Zero(), linkRotationalInertia);
	SpatialInertia linkSpatialInertial2(linkMass, MathTypes::Vec3::Zero(), linkRotationalInertia);
	SpatialInertia linkSpatialInertial3(linkMass, MathTypes::Vec3::Zero(), linkRotationalInertia);

	int parentID = -1;
	
	dynamics.AddBody(linkSpatialInertial1, MathTypes::Mat6::Identity(), COORD_AXIS::Y, parentID);
	parentID++;

	for (int i = 1; i < numLinks; i++) {
		MathTypes::Vec3 constantLinkOffset;
		constantLinkOffset << 0.0f, 0.0f, -linkHieght;
		MathTypes::Mat6 X = CreateSpatialForm(MathTypes::Mat3::Identity(), constantLinkOffset);

		dynamics.AddBody(linkSpatialInertial1, X, COORD_AXIS::Y, parentID);
		parentID++;
	}
}

void CubeChain::SetState(const State& newState)
{
	state = newState;
}

void CubeChain::SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces)
{
	dynamics.SetExternalForces(externalForces);
}

void CubeChain::SetExternalForceAt(int i, const MathTypes::Vec6& externalForce)
{
	dynamics.SetExternalForceAt(i, externalForce);
}


State CubeChain::GetState()
{
	return state;
}

void CubeChain::Integrate(State& state, const StateDot& dstate)
{
	state.bodyVelocity += dstate.bodyVelocityDDot * deltaT;
	state.bodyPosition += dstate.bodyPositionDot * deltaT;

	//state.bodyPosition += state.bodyVelocity.template block<3, 1>(3, 0) * deltaT;

	for (int i = 0; i < 12; i++)
	{
		state.qDot[i] += dstate.qDDot[i] * deltaT;
		state.q[i] += state.qDot[i] * deltaT;
	}
}


State CubeChain::StepDynamicsModel(State& state)
{
	StateDot dState = dynamics.Step(state);

	Integrate(state, dState);

	return state;
}