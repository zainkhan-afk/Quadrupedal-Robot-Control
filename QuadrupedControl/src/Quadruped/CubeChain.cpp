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

void CubeChain::Initialize(int _numLinks)
{
	numLinks = _numLinks;

	dynamics.Initialize(numLinks);
	
	MathTypes::Mat3 linkRotationalInertia;
	linkRotationalInertia <<	0.00016667f, 0.0f, 0.0f,
								0.0f, 0.00016667f, 0.0f,
								0.0f, 0.0f, 0.00016667f;

	SpatialInertia linkSpatialInertial(linkMass, MathTypes::Vec3(0.0f, 0.0f, linkHieght/2.0f), linkRotationalInertia);

	int parentID = -1;
	
	dynamics.AddBody(linkSpatialInertial, MathTypes::Mat6::Identity(), COORD_AXIS::Y, parentID);
	parentID++;

	for (int i = 1; i < numLinks; i++) {
		MathTypes::Vec3 constantLinkOffset;
		constantLinkOffset << 0.0f, 0.0f, -linkHieght;
		MathTypes::Mat6 X = CreateSpatialForm(MathTypes::Mat3::Identity(), constantLinkOffset);

		if (i % 2 == 0) {
			dynamics.AddBody(linkSpatialInertial, X, COORD_AXIS::Y, parentID);
		}
		else {
			dynamics.AddBody(linkSpatialInertial, X, COORD_AXIS::X, parentID);
		}
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
	//state.bodyVelocity += dstate.bodyVelocityDDot * deltaT;
	//state.bodyPosition += dstate.bodyPositionDot * deltaT;

	//state.bodyPosition += state.bodyVelocity.template block<3, 1>(3, 0) * deltaT;

	for (int i = 1; i < numLinks; i++)
	{
		state.qDot[i - 1] += dstate.qDDot[i - 1] * deltaT;
		state.q[i - 1] += state.qDot[i - 1] * deltaT;
	}
}


State CubeChain::StepDynamicsModel(State& state)
{
	StateDot dState = dynamics.Step(state);

	Integrate(state, dState);

	return state;
}