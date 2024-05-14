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
	/*linkRotationalInertia << 0.00016667f, 0.0f, 0.0f,
		0.0f, 0.00016667f, 0.0f,
		0.0f, 0.0f, 0.00016667f;*/

	linkRotationalInertia << 0.005, 0.0, 0.0,
		0.0, 0.005, 0.0,
		0.0, 0.0, 0.005;

	SpatialInertia linkSpatialInertial(linkMass, MathTypes::Vec3(0, 0, linkHieght), linkRotationalInertia);

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
		
		//dynamics.torques[i - 1] = (state.q[i - 1] - prevState.q[i - 1]) * 0.1;
		//dynamics.torques[i - 1] += (state.qDot[i - 1] - prevState.qDot[i - 1]) * 0.1;
	}
	prevState = state;
	prevDState = dstate;
}

void CubeChain::VerletIntegrate(State& state, const StateDot& dstate)
{
	// TODO: Does not work. need to fix it.
	State state_i = state;
	for (int i = 1; i < numLinks; i++)
	{
		double prevVal = state.q[i - 1];
		state.q[i - 1] = 2 * state_i.q[i - 1] - prevState.q[i - 1] + (dstate.qDDot[i - 1] * deltaT * deltaT);
		
		state.qDot[i - 1] = (state.q[i - 1] - state_i.q[i - 1]);
		//state.qDot[i - 1] += dstate.qDDot[i - 1] * deltaT;
	}
	prevState = state_i;
}


State CubeChain::StepDynamicsModel(State& state)
{
	StateDot dState = dynamics.Step(state);

	Integrate(state, dState);

	return state;
}