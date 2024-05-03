#include "pch.h"

#include "Quadruped/RobotDynamics.h"
#include "Quadruped/Spatial.h"

RobotDynamics::RobotDynamics()
{
	MathTypes::Mat6 eyeMat6 = MathTypes::Mat6::Identity();
	MathTypes::Vec6 zeroVec6 = MathTypes::Vec6::Zero();
	MathTypes::Mat6 zeroMat6 = MathTypes::Mat6::Zero();

	SpatialInertia zeroI;

	for (int i = 0; i < numLinks; i++) {
		this->linkInertias.push_back(zeroI);
		this->articulatedInertias.push_back(zeroI);

		this->v.push_back(zeroVec6);
		this->S.push_back(zeroVec6);
		
		this->Xl.push_back(eyeMat6);
		this->Xp.push_back(eyeMat6);
		this->Xb.push_back(eyeMat6);

		this->axis.push_back(COORD_AXIS::X);
		this->parents.push_back(-1);
	}
}


RobotDynamics::~RobotDynamics()
{

}


void RobotDynamics::AddBody(SpatialInertia I, MathTypes::Mat6 pos, COORD_AXIS axis, int parent)
{
	this->linkInertias[currentIndex] = I;
	this->Xl[currentIndex] = pos;
	this->parents[currentIndex] = parent;
	
	if (parent >= 0) {
		this->axis[currentIndex] = axis;
		this->S[currentIndex] = JointMotionSubspace(JOINT_TYPE::REVOLUTE, axis);
	}
	currentIndex++;
}

StateDot RobotDynamics::Step(const State& state)
{
	StateDot dState;

	dState = RunArticulatedBodyAlgorithm(state);

	return dState;
}

StateDot RobotDynamics::RunArticulatedBodyAlgorithm(const State& state)
{
	StateDot dState;
	// Pass 1 down the tree
	this->Xp[0] = MathTypes::Mat6::Identity(); // Change this to the actual robot base position wrt to the world.
	for (int i = 1; i < numLinks; i++)
	{	
		MathTypes::Mat6 Xj = JointRotationMatrix(state.q[i - 1], this->axis[i]);
		Xp[i] = Xj * Xl[i];


		MathTypes::Vec6 vJoint = S[i] * state.qDot[i - 1];
		
		v[i] = Xp[i] * v[parents[i]] + vJoint;
	}

	// This second loop can be removed and incorporated into the first one.
	for (int i = 0; i < numLinks; i++)
	{
		if (this->parents[i] != -1) {
			this->Xb[i] = this->Xp[i] * this->Xl[i];
		}
		else {
			this->Xb[i] = this->Xp[i] * this->Xb[this->parents[i]];
		}
	}

	// Pass 2 up the tree
	for (int i = numLinks; i > 0; i++)
	{

	}

	
	// Pass 3 down the tree
	for (int i = 0; i < numLinks; i++)
	{

	}

	return dState;
}