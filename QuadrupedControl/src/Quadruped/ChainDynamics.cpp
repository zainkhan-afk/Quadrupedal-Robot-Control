#include "pch.h"
#include "Quadruped/ChainDynamics.h"
#include "Quadruped/Spatial.h"

ChainDynamics::ChainDynamics()
{
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

	for (int i = 0; i < numLinks + 1; i++) {
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
	currentIndex += 1;
}


void ChainDynamics::AddBody(SpatialInertia I, SpatialTransform X, COORD_AXIS axis, int parent)
{	
	this->linkInertias[currentIndex] = I;
	this->Xl[currentIndex] = X;
	this->parents[currentIndex] = parent;
	this->axis[currentIndex] = axis;
	this->S[currentIndex] = JointMotionSubspace(JOINT_TYPE::REVOLUTE, axis);

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
	MathTypes::Mat3 RIdent = MathTypes::Mat3::Identity();

	// Pass 1 down the tree
	MathTypes::Vec3 position(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]);
	MathTypes::Vec3 rotation(state.bodyPose[0], state.bodyPose[1], state.bodyPose[2]);

	SpatialTransform Xref_base_tranlation(RIdent, MathTypes::Vec3(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]));
	SpatialTransform Xref_base_rot(MathTypes::Vec3(state.bodyPose[0], state.bodyPose[1], state.bodyPose[2]), MathTypes::Vec3::Zero());

	//Xp[0] = SpatialTransform(rotation, position);
	
	v[0] = MathTypes::Vec6::Zero();
	articulatedInertias[0] = linkInertias[0];

	pa[0] = MathTypes::Vec6::Zero();

	for (int i = 1; i < this->Xl.size(); i++)
	{
		MathTypes::Mat3 R = GetRotationMatrix(state.q[i - 1], this->axis[i]);
		SpatialTransform Xj(R, MathTypes::Vec3::Zero());


		Xp[i] = Xj * Xl[i];
		if (parents[i] != 0) {
			Xb[i] = Xp[i] * Xb[parents[i]];
		}
		else {
			Xb[i] = Xp[i];
		}


		MathTypes::Vec6 vJoint = S[i] * state.qDot[i - 1];

		v[i] = Xp[i].Apply(v[parents[i]]) + vJoint;
		c[i] = CrossProductMotion(v[i], vJoint);
		articulatedInertias[i].SetInertia(linkInertias[i].GetInertia());

		pa[i] = CrossProductForce(v[i], linkInertias[i].GetInertia() * v[i]) - Xb[i].GetSpatialFormForce() * f[i];
	}

	// Pass 2 up the tree
	for (int i = this->Xl.size() - 1; i > 0; i--)
	{
		U[i] = articulatedInertias[i].GetInertia() * S[i];
		D[i] = S[i].dot(U[i]);
		
		u[i] = torques[i-1] - S[i].dot(pa[i]);
		
		if (parents[i] != 0) {
			MathTypes::Mat6 Ia = articulatedInertias[i].GetInertia() - U[i] * (U[i] / D[i]).transpose();
			MathTypes::Vec6 _pa = pa[i] + Ia * c[i] + U[i] * u[i] / D[i];

			articulatedInertias[parents[i]].AddInertia(Xp[i].GetSpatialFormTranspose() * Ia * Xp[i].GetSpatialForm());
			pa[parents[i]] += Xp[i].ApplyTranspose(_pa);
		}
	}

	a[0] = G;
	// Pass 3 down the tree
	for (int i = 1; i < this->Xl.size(); i++)
	{
		MathTypes::Vec6 a_ = Xp[i].Apply(a[parents[i]]) + c[i];
		dState.qDDot[i - 1] = (1 / D[i]) * (u[i] - U[i].dot(a_));
		a[i] = a_ + S[i] * dState.qDDot[i - 1];
	}

	return dState;
}