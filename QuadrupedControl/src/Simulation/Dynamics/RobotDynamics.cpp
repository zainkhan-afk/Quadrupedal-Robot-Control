#include "Simulation/Dynamics/RobotDynamics.h"
#include "Simulation/Dynamics/Spatial.h"
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

void RobotDynamics::AddContactPoint(MathTypes::Vec3 contactPoint, int parent)
{
	MathTypes::Mat3 Rc = MathTypes::Mat3::Identity();
	Xc.push_back(SpatialTransform(Rc, contactPoint));
	Xcb.push_back(SpatialTransform());
	contactPoints.push_back(contactPoint);
	contactPointsParents.push_back(parent);
	fc.push_back(MathTypes::Vec3::Zero());
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

StateDot RobotDynamics::RunArticulatedBodyAlgorithm(const State& state)//
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

	//Xp[1] = SpatialTransform(Rref_base, MathTypes::Vec3::Zero());;
	Xp[1] = SpatialTransform();
 	//Xp[1] = Xref_base_rot;

	//Xp[1] = SpatialTransform(MathTypes::Vec3(state.bodyPose[0], state.bodyPose[1], state.bodyPose[2]), MathTypes::Vec3(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]));


	///////////////////////////// PASS 1 DOWN THE TREE START ///////////////////////////// 
	v[1] = state.bodyVelocity;
	
	for (size_t i = 2; i < this->Xl.size(); i++)
	{	
		SpatialTransform Xj(GetRotationMatrix(state.q[i - 2], this->axis[i]), MathTypes::Vec3::Zero());
		Xp[i] = Xj * Xl[i]; // Book
		//Xp[i] = Xl[i] * Xj; // How homog transforms work

		MathTypes::Vec6 vJoint = S[i] * state.qDot[i - 2];
		
		v[i] = Xp[i].GetSpatialForm() * v[parents[i]] + vJoint;
		c[i] = CrossProductMotion(v[i], vJoint);
	}

	for (size_t i = 0; i < contactPoints.size(); i++)
	{
		int contactParent = contactPointsParents[i];
		SpatialTransform Xpc = Xc[i] * Xp[contactParent];
		Xcb[i] = Xc[i] * Xb[contactParent];

		MathTypes::Vec3 p = Xc[i].GetTranslation();
		MathTypes::Vec3 r = Xb[contactParent].GetInverse().GetTranslation();

		MathTypes::Vec3 foot_pos = Xb[contactParent].GetInverse().GetRotation()*(p - r);

		MathTypes::Vec6 f_sp = MathTypes::Vec6::Zero();

		f_sp.topLeftCorner<3, 1>() = foot_pos.cross(fc[i]);
		f_sp.bottomLeftCorner<3, 1>() = fc[i];

		f[contactParent - 1] += f_sp;
		//f[contactParent - 1] += Xcb[i].GetSpatialFormForce() * fc[i];
		//f[contactParent - 1] += Xpc.GetSpatialFormForce() * fc[i];
	}

	for (size_t i = 1; i < this->Xl.size(); i++){
		if (parents[i] != 0) {
			Xb[i] = Xp[i] * Xb[parents[i]]; // Book
			//Xb[i] = Xb[parents[i]] * Xp[i]; // How homog transforms work
		}
		else {
			Xb[i] = Xp[i];
		}

		articulatedInertias[i].SetInertia(linkInertias[i].GetInertia());
		pa[i] = CrossProductForce(v[i], linkInertias[i].GetInertia() * v[i]) - 
			/*Xbase_ref_rot.GetSpatialFormForce() * */Xb[i].GetSpatialFormForce() * f[i - 1];
	}
	///////////////////////////// PASS 1 DOWN THE TREE END ///////////////////////////// 


	///////////////////////////// PASS 2 DOWN THE TREE Start ///////////////////////////// 
	for (size_t i = this->Xl.size() - 1; i >= 2; i--)
	{
		U[i] = articulatedInertias[i].GetInertia() * S[i];
		D[i] = S[i].dot(U[i]);

		u[i] = torques[i - 2] - S[i].dot(pa[i]);
		
		MathTypes::Mat6 Ia = articulatedInertias[i].GetInertia() - (U[i] * (U[i] / D[i]).transpose());
		MathTypes::Vec6 _pa = pa[i] + Ia * c[i] + ((U[i] * u[i]) / D[i]);

		articulatedInertias[parents[i]].AddInertia(Xp[i].GetSpatialFormTranspose() * Ia * Xp[i].GetSpatialForm());
		pa[parents[i]] += Xp[i].GetSpatialFormTranspose() * _pa;
	}
	///////////////////////////// PASS 2 DOWN THE TREE END ///////////////////////////// 

	///////////////////////////// PASS 3 DOWN THE TREE Start ///////////////////////////// 
	a[1]  = -articulatedInertias[1].GetInertia().inverse() * pa[1];
	for (size_t i = 2; i < this->Xl.size(); i++)
	{
		// BOOK
		MathTypes::Vec6 a_ = Xp[i].GetSpatialForm() * a[parents[i]] + c[i];
		dState.qDDot[i - 2] = (u[i] - U[i].dot(a_)) / D[i];
		a[i] = a_ + S[i] * dState.qDDot[i - 2];
	}
	a[1] +=  G;
	///////////////////////////// PASS 3 DOWN THE TREE END ///////////////////////////// 


	//dState.bodyVelocityDDot = a[1];

	dState.bodyVelocityDDot.block<3, 1>(0, 0) = MathTypes::Vec3::Zero();

	return dState;
}