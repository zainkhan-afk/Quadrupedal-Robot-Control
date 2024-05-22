#include "pch.h"
#include "Quadruped/ChainDynamics.h"
#include "Quadruped/Spatial.h"

ChainDynamics::ChainDynamics()
{
	G[5] = -10;
}


ChainDynamics::~ChainDynamics()
{

}

void ChainDynamics::Initialize(int _numLinks)
{
	numLinks = _numLinks;

	MathTypes::Mat6 eyeMat6 = MathTypes::Mat6::Identity();
	MathTypes::Vec6 zeroVec6 = MathTypes::Vec6::Zero();
	MathTypes::Mat6 zeroMat6 = MathTypes::Mat6::Zero();

	SpatialTransform Xident;

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

		this->Xl.push_back(Xident);
		this->Xp.push_back(Xident);
		this->Xb.push_back(Xident);

		this->axis.push_back(COORD_AXIS::X);
		this->D.push_back(0.0f);
		this->u.push_back(0.0f);
		this->torques.push_back(0.0f);
		this->parents.push_back(-1);
	}
}


void ChainDynamics::AddBody(SpatialInertia I, SpatialTransform X, COORD_AXIS axis, int parent)
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

void ChainDynamics::SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces)
{
	for (int i = 0; i < externalForces.size(); i++)
	{
		f[i] = externalForces[i];
	}
}

void ChainDynamics::SetExternalForceAt(int i, const MathTypes::Vec6& externalForce)
{
	f[i] = externalForce;
}

StateDot ChainDynamics::Step(const State& state)
{
	StateDot dState;

	dState = RunArticulatedBodyAlgorithm(state);

	return dState;
}

StateDot ChainDynamics::RunArticulatedBodyAlgorithm(const State& state)
{
	StateDot dState;
	// Pass 1 down the tree
	Xp[0] = SpatialTransform(QuatToRotationMatrix(state.bodyOrientation), state.bodyPosition);
	Xb[0] = Xp[0];
	v[0] = state.bodyVelocity;
	v[0] = MathTypes::Vec6::Zero();
	articulatedInertias[0] = linkInertias[0];

	//MathTypes::Mat3 R = SpatialToRotMat(Xb[0]);
	//MathTypes::Vec3 p = SpatialToTranslation(Xb[0]);
	//MathTypes::Mat6 iX0 = CreateSpatialForm(R.transpose(), R * p);

	//pa[0] = CrossProductForce(v[0], linkInertias[0].GetInertia() * v[0]) /*- iX0 * f[0]*/;
	pa[0] = MathTypes::Vec6::Zero();

	for (int i = 1; i < numLinks; i++)
	{
		MathTypes::Mat3 R = GetRotationMatrix(state.q[i - 1], this->axis[i]);
		SpatialTransform Xj(R, MathTypes::Vec3::Zero());
		
		//Xp[i] = Xl[i] * Xj;
		//Xb[i] = Xb[parents[i]] * Xp[i];


		Xp[i] = Xj * Xl[i];
		Xb[i] = Xp[i] * Xb[parents[i]];


		MathTypes::Vec6 vJoint = S[i] * state.qDot[i - 1];

		v[i] = Xp[i].GetSpatialForm() * v[parents[i]] + vJoint;
		c[i] = CrossProductMotion(v[i], vJoint);
		articulatedInertias[i].SetInertia(linkInertias[i].GetInertia());

		pa[i] = CrossProductForce(v[i], linkInertias[i].GetInertia() * v[i]) /*- Xb[i].GetSpatialFormForce() * f[i]*/;
	}

	// Pass 2 up the tree
	for (int i = numLinks - 1; i > 0; i--)
	{
		U[i] = articulatedInertias[i].GetInertia() * S[i];
		D[i] = (S[i].transpose() * U[i]);
		
		u[i] = torques[i-1] - S[i].transpose() * pa[i];
		MathTypes::Mat6 Ia = articulatedInertias[i].GetInertia() - U[i] * (U[i] / D[i]).transpose();
		MathTypes::Vec6 _pa = pa[i] + Ia * c[i] + U[i] * u[i] / D[i];

		articulatedInertias[parents[i]].AddInertia(Xp[i].GetSpatialFormTranspose() * Ia * Xp[i].GetSpatialForm());
		pa[parents[i]] += Xp[i].GetSpatialFormTranspose() * _pa;
	}


	a[0] = -G;
	// Pass 3 down the tree
	for (int i = 1; i < numLinks; i++)
	{
		MathTypes::Vec6 a_ = Xp[i].GetSpatialForm() * a[parents[i]] + c[i];
		dState.qDDot[i - 1] = (1 / D[i]) * (u[i] - U[i].transpose() * a_);
		a[i] = a_ + S[i] * dState.qDDot[i - 1];
	}
	//a[0] += Xp[0] * G;
	//dState.bodyVelocityDDot = a[0];
	// dState.bodyPositionDot = state.bodyVelocity.template block<3, 1>(3, 0);

	return dState;
}