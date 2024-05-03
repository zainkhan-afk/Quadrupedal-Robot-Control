#include "pch.h"

#include "Quadruped/RobotDynamics.h"
#include "Quadruped/Spatial.h"

RobotDynamics::RobotDynamics()
{
	dtypes::Mat6 eyeMat6 = dtypes::Mat6::Identity();
	dtypes::Vec6 zeroVec6 = dtypes::Vec6::Zero();
	dtypes::Mat6 zeroMat6 = dtypes::Mat6::Zero();

	SpatialInertia zeroI;

	for (int i = 0; i < numLinks; i++) {
		this->linkInertias.push_back(zeroI);
		this->articulatedInertias.push_back(zeroI);
		
		this->Xl.push_back(eyeMat6);
		this->Xp.push_back(eyeMat6);
		this->Xb.push_back(eyeMat6);


	}
}


RobotDynamics::~RobotDynamics()
{

}


void RobotDynamics::AddBody(SpatialInertia I, dtypes::Mat6 pos, int axis, int parent)
{
	this->linkInertias[currentIndex] = I;
	this->Xl[currentIndex] = pos;
	this->parents[currentIndex] = parent;
	this->axis[currentIndex] = axis;
	
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
	this->Xp[0] = dtypes::Mat6::Identity(); // Change this to the actual robot base position wrt to the world.
	for (int i = 1; i < numLinks; i++)
	{
		dtypes::Mat6 Xj = JointRotationMatrix(state.q[i - 1], this->axis[i]);
		this->Xp[i] = Xj * Xl[i];
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