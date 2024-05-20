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

	SpatialInertia linkSpatialInertial(linkMass, MathTypes::Vec3(0, 0, linkHieght/2), linkRotationalInertia);

	int parentID = -1;
	
	SpatialTransform XIdent;

	dynamics.AddBody(linkSpatialInertial, XIdent, COORD_AXIS::Y, parentID);
	parentID++;

	MathTypes::Mat3 RIdent = MathTypes::Mat3::Identity();

	transformationChain.push_back(MathTypes::Mat4::Identity());

	for (int i = 1; i < numLinks; i++) {
		MathTypes::Vec3 constantLinkOffset;
		constantLinkOffset << 0.0f, 0.0f, -linkHieght;
		SpatialTransform X(RIdent, constantLinkOffset);

		if (i % 2 == 0) {
			dynamics.AddBody(linkSpatialInertial, X, COORD_AXIS::Y, parentID);
		}
		else {
			dynamics.AddBody(linkSpatialInertial, X, COORD_AXIS::X, parentID);
		}
		transformationChain.push_back(MathTypes::Mat4::Identity());
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

void CubeChain::GetVisualTransformations(const State& state)
{
	//transformationChain[0] = MathTypes::Mat4::Identity();

	for (int i = 1; i < numLinks; i++)
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

		transformationChain[i] = transformationChain[dynamics.parents[i]] * Tj * TpjTrans * TpjRot;
	}
}

State CubeChain::StepDynamicsModel(State& state)
{
	StateDot dState = dynamics.Step(state);

	Integrate(state, dState);

	return state;
}