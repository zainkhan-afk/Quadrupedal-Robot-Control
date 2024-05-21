#include "pch.h"

#include "Quadruped/RobotDynamics.h"
#include "Quadruped/Spatial.h"

RobotDynamics::RobotDynamics()
{
	MathTypes::Mat6 eyeMat6 = MathTypes::Mat6::Identity();
	MathTypes::Vec6 zeroVec6 = MathTypes::Vec6::Zero();
	MathTypes::Mat6 zeroMat6 = MathTypes::Mat6::Zero();

	SpatialTransform xIdent;
	SpatialInertia zeroI;

	for (int i = 0; i < numLinks; i++) {
		this->linkInertias.push_back(zeroI);
		this->articulatedInertias.push_back(zeroI);

		this->v.push_back(zeroVec6);
		this->S.push_back(zeroVec6);
		this->c.push_back(zeroVec6);
		this->pa.push_back(zeroVec6);
		
		this->U.push_back(zeroVec6);
		this->a.push_back(zeroVec6);
		this->f.push_back(zeroVec6);
		
		this->Xl.push_back(xIdent);
		this->Xp.push_back(xIdent);
		this->Xb.push_back(xIdent);

		this->axis.push_back(COORD_AXIS::X);
		this->D.push_back(0.0f);
		this->u.push_back(0.0f);
		this->torques.push_back(0.0f);
		this->parents.push_back(-1);
	}

	//G[5] = -9.8f;
}


RobotDynamics::~RobotDynamics()
{

}


void RobotDynamics::AddBody(SpatialInertia I, SpatialTransform X, COORD_AXIS axis, int parent)
{
	this->linkInertias[currentIndex] = I;
	this->Xl[currentIndex] = X;
	this->parents[currentIndex] = parent;
	
	if (parent >= 0) {
		this->axis[currentIndex] = axis;
		this->S[currentIndex] = JointMotionSubspace(JOINT_TYPE::REVOLUTE, axis);
	}
	currentIndex++;
}

void RobotDynamics::SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces)
{
	for (int i = 0; i<externalForces.size(); i++)
	{
		f[i] = externalForces[i];
	}
}

void RobotDynamics::SetExternalForceAt(int i, const MathTypes::Vec6& externalForce)
{
	f[i] = externalForce;
}

StateDot RobotDynamics::Step(const State& state)
{
	StateDot dState;

	dState = RunArticulatedBodyAlgorithm(state);

	return dState;
}

StateDot RobotDynamics::RunArticulatedBodyAlgorithm(const State& state)
{
	//TODO: Calcuate everything in base coordinates. Remove global context, and calculate forces and everything else in robot base frame 
	StateDot dState;

	SpatialTransform Xref_base(QuatToRotationMatrix(state.bodyOrientation), state.bodyPosition);

	// Pass 1 down the tree
	//Xp[0] = SpatialTransform(QuatToRotationMatrix(state.bodyOrientation), state.bodyPosition);
	Xb[0] = Xp[0];
	v[0] = state.bodyVelocity;
	articulatedInertias[0] = linkInertias[0];

	pa[0] = CrossProductForce(v[0], linkInertias[0].GetInertia() * v[0]) - Xb[0].GetSpatialFormForce() * f[0];
	
	for (int i = 1; i < numLinks; i++)
	{	
		SpatialTransform Xj(GetRotationMatrix(state.q[i - 1], this->axis[i]), MathTypes::Vec3::Zero());
		Xp[i] = Xj * Xl[i];
		Xb[i] = Xb[parents[i]] * Xp[i];


		MathTypes::Vec6 vJoint = S[i] * state.qDot[i - 1];
		
		v[i] = Xp[i].GetSpatialForm() * v[parents[i]] + vJoint;
		c[i] = CrossProductMotion(v[i], vJoint);
		articulatedInertias[i] = linkInertias[i];
		
		pa[i] = CrossProductForce(v[i], linkInertias[i].GetInertia() * v[i]) - Xb[i].GetSpatialFormForce() * f[i];
	}

	// Pass 2 up the tree
	for (int i = numLinks - 1; i > 0; i--)
	{
		U[i] = articulatedInertias[i].GetInertia() * S[i];
		D[i] = S[i].transpose() * U[i];

		u[i] = torques[i - 1] - S[i].transpose() * pa[i];
		
		MathTypes::Mat6 Ia = articulatedInertias[i].GetInertia() - U[i] * (U[i] / D[i]).transpose();
		MathTypes::Vec6 _pa = pa[i] + Ia * c[i] + U[i] * u[i] / D[i];

		articulatedInertias[parents[i]].AddInertia(Xp[i].GetSpatialFormTranspose() * Ia * Xp[i].GetSpatialForm());
		pa[parents[i]] += Xp[i].GetSpatialFormTranspose() * _pa;
	}

	
	a[0]  = -articulatedInertias[0].GetInertia().inverse()* pa[0];
	// Pass 3 down the tree
	for (int i = 1; i < numLinks; i++)
	{
		a[i] = Xp[i].GetSpatialForm() * a[parents[i]] + c[i];
		dState.qDDot[i - 1] = D[i] * (u[i] - U[i].transpose() * a[i]);
		a[i] += S[i] * dState.qDDot[i - 1];
	}
	a[0] += Xb[0].GetSpatialFormForce() * G;
	dState.bodyVelocityDDot = a[0];
	dState.bodyPositionDot = state.bodyVelocity.template block<3, 1>(3, 0);
	
	return dState;
}