#include "pch.h"

#include "Quadruped/RobotDynamics.h"
#include "Quadruped/Spatial.h"
#include <math.h>


RobotDynamics::RobotDynamics()
{
	MathTypes::Mat6 eyeMat6 = MathTypes::Mat6::Identity();
	MathTypes::Vec6 zeroVec6 = MathTypes::Vec6::Zero();
	MathTypes::Mat6 zeroMat6 = MathTypes::Mat6::Zero();

	SpatialTransform xIdent;
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
		
		this->Xl.push_back(xIdent);
		this->Xp.push_back(xIdent);
		this->Xb.push_back(xIdent);

		this->axis.push_back(COORD_AXIS::X);
		this->D.push_back(0.0f);
		this->u.push_back(0.0f);
		this->parents.push_back(0);

		if (i < 13) {
			this->f.push_back(zeroVec6);
		}
		if (i < 12) {
			this->torques.push_back(0.0f);
		}
	}

	currentIndex += 1;
}


RobotDynamics::~RobotDynamics()
{

}


void RobotDynamics::AddBody(SpatialInertia I, SpatialTransform X, COORD_AXIS axis, int parent)
{
	this->linkInertias[currentIndex] = I;
	this->Xl[currentIndex] = X;
	this->parents[currentIndex] = parent;
	this->axis[currentIndex] = axis;
	this->S[currentIndex] = JointMotionSubspace(JOINT_TYPE::REVOLUTE, axis);

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
	//dState = RunArticulatedBodyAlgorithmMiT(state);

	return dState;
}

StateDot RobotDynamics::RunArticulatedBodyAlgorithm(const State& state)
{
	//TODO: Calcuate everything in base coordinates. Remove global context, and calculate forces and everything else in robot base frame 
	StateDot dState;
	MathTypes::Mat3 RIdent = MathTypes::Mat3::Identity();

	SpatialTransform Xref_base_tranlation(RIdent, MathTypes::Vec3(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]));

	MathTypes::Mat3 Rx = GetRotationMatrix(state.bodyPose[0], COORD_AXIS::X);
	MathTypes::Mat3 Ry = GetRotationMatrix(state.bodyPose[1], COORD_AXIS::Y);
	MathTypes::Mat3 Rz = GetRotationMatrix(state.bodyPose[2], COORD_AXIS::Z);

	MathTypes::Mat3 Rref_base = Rx * Ry * Rz;
	MathTypes::Mat3 Rref_base_t = (Rx * Ry * Rz).transpose();
	SpatialTransform Xref_base_rot(Rref_base, MathTypes::Vec3::Zero());
	SpatialTransform Xbase_ref_rot(Rref_base_t, MathTypes::Vec3::Zero());

	Xp[1] = SpatialTransform(Rref_base, MathTypes::Vec3::Zero());;
	//Xp[1] = SpatialTransform();;
 	//Xp[1] = Xref_base_rot;

	/*Xp[1] = SpatialTransform(MathTypes::Vec3(state.bodyPose[0], state.bodyPose[1], state.bodyPose[2]),
							 MathTypes::Vec3(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]));*/

	// Pass 1 down the tree
	//Xb[0] = Xp[0];
	v[1] = state.bodyVelocity;
	//v[1] = MathTypes::Vec6::Zero();
	
	for (int i = 2; i < this->Xl.size(); i++)
	{	
		SpatialTransform Xj(GetRotationMatrix(state.q[i - 2], this->axis[i]), MathTypes::Vec3::Zero());
		Xp[i] = Xj * Xl[i];

		MathTypes::Vec6 vJoint = S[i] * state.qDot[i - 2];
		
		v[i] = Xp[i].GetSpatialForm() * v[parents[i]] + vJoint;
		c[i] = CrossProductMotion(v[i], vJoint);
	}


	//pa[1] = CrossProductForce(v[1], linkInertias[1].GetInertia() * v[1]) - Xb[1].GetSpatialFormForce() * f[0];
	for (int i = 1; i < this->Xl.size(); i++){

		if (parents[i] != 0) {
			Xb[i] = Xp[i] * Xb[parents[i]];
		}
		else {
			Xb[i] = Xp[i];
		}

		articulatedInertias[i].SetInertia(linkInertias[i].GetInertia());
		pa[i] = CrossProductForce(v[i], linkInertias[i].GetInertia() * v[i]) - Xb[i].GetSpatialFormForce() /** Xref_base_rot.GetSpatialFormForce()*/ * f[i - 1];
	}

	// Pass 2 up the tree
	for (int i = this->Xl.size() - 1; i >= 2; i--)
	{
		U[i] = articulatedInertias[i].GetInertia() * S[i];
		D[i] = S[i].transpose() * U[i];

		u[i] = torques[i - 2] - S[i].transpose() * pa[i];
		
		//if (parents[i] != 0) {
			MathTypes::Mat6 Ia = articulatedInertias[i].GetInertia() - (U[i] * (U[i] / D[i]).transpose());
			MathTypes::Vec6 _pa = pa[i] + Ia * c[i] + ((U[i] * u[i]) / D[i]);

			articulatedInertias[parents[i]].AddInertia(Xp[i].GetSpatialFormTranspose() * Ia * Xp[i].GetSpatialForm());
			pa[parents[i]].noalias() += Xp[i].GetSpatialFormTranspose() * _pa;
		//}
	}
	invIA0.compute(articulatedInertias[0].GetInertia());


	// Pass 3 down the tree
	a[1]  = -articulatedInertias[1].GetInertia().inverse() * pa[1];
	//a[1] = G;

	for (int i = 2; i < this->Xl.size(); i++)
	{
		// BOOK
		MathTypes::Vec6 a_ = Xp[i].GetSpatialForm() * a[parents[i]] + c[i];
		dState.qDDot[i - 2] = (u[i] - U[i].transpose() * a_) / D[i];
		a[i] = a_ + S[i] * dState.qDDot[i - 2];
	}
	a[1] +=  G;
	dState.bodyVelocityDDot = Xbase_ref_rot.GetSpatialForm() * a[1];

	return dState;
}

StateDot RobotDynamics::RunArticulatedBodyAlgorithmMiT(const State& state)
{
	//TODO: Calcuate everything in base coordinates. Remove global context, and calculate forces and everything else in robot base frame 
	StateDot dState;
	MathTypes::Mat3 RIdent = MathTypes::Mat3::Identity();

	Xp[0] = SpatialTransform(MathTypes::Vec3(state.bodyPose[0], state.bodyPose[1], state.bodyPose[2]),
		MathTypes::Vec3(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]));
	
	// Pass 1 down the tree
	Xb[0] = Xp[0];
	v[0] = state.bodyVelocity;
	articulatedInertias[0] = linkInertias[0];
	pa[0] = CrossProductForce(v[0], linkInertias[0].GetInertia() * v[0]) - Xb[0].GetSpatialFormForce() * f[0];

	for (int i = 1; i < numLinks; i++)
	{
		SpatialTransform Xj(GetRotationMatrix(state.q[i - 1], this->axis[i]), MathTypes::Vec3::Zero());
		Xp[i] = Xj * Xl[i];
		Xb[i] = Xp[i] * Xb[parents[i]];


		MathTypes::Vec6 vJoint = S[i] * state.qDot[i - 1];

		v[i] = Xp[i].GetSpatialForm() * v[parents[i]] + vJoint;
		c[i] = CrossProductMotion(v[i], vJoint);
		articulatedInertias[i].SetInertia(linkInertias[i].GetInertia());

		pa[i] = CrossProductForce(v[i], linkInertias[i].GetInertia() * v[i]) - Xb[i].GetSpatialFormForce() * f[i];
	}

	// Pass 2 up the tree
	for (int i = numLinks - 1; i > 0; i--)
	{
		U[i] = articulatedInertias[i].GetInertia() * S[i];
		D[i] = S[i].transpose() * U[i];

		u[i] = torques[i - 1] - S[i].transpose() * pa[i];

		MathTypes::Mat6 Ia = Xp[i].GetSpatialFormTranspose() * articulatedInertias[i].GetInertia() * 
							Xp[i].GetSpatialForm() - U[i] * U[i].transpose() / D[i];
		
		MathTypes::Vec6 _pa = Xp[i].GetSpatialFormTranspose() * (pa[i] + articulatedInertias[i].GetInertia() * c[i]) + 
							(U[i] * u[i] / D[i]);

		articulatedInertias[parents[i]].AddInertia(Ia);
		pa[parents[i]].noalias() += _pa;
	}
	invIA0.compute(articulatedInertias[0].GetInertia());

	// Pass 3 down the tree
	MathTypes::Vec6 a0 = -G;
	MathTypes::Vec6 ub = -pa[0];
	a[0] = Xp[0].GetSpatialForm() * a0;
	MathTypes::Vec6  afb = invIA0.solve(ub - articulatedInertias[0].GetInertia().transpose() * a[0]);
	a[0] += afb;

	for (int i = 1; i < numLinks; i++)
	{
		dState.qDDot[i - 1] = (u[i] - U[i].transpose() * a[parents[i]]) / D[i];
		a[i] = Xp[i].GetSpatialForm() * a[parents[i]] + S[i] * dState.qDDot[i - 1] + c[i];
	}
	dState.bodyVelocityDDot = afb;
	MathTypes::Mat3 R = Xp[0].GetRotation();
	dState.bodyPositionDot = R.transpose() * state.bodyVelocity.template block<3, 1>(3, 0);

	return dState;
}