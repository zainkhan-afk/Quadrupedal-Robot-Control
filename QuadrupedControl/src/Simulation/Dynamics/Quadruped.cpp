#include "Simulation/Dynamics/SpatialTransform.h"
#include "Simulation/Dynamics/Quadruped.h"
#include "Simulation/Dynamics/Utilities.h"
#include "Simulation/Dynamics/Spatial.h"
#include <math.h>
#include "cinder/Log.h"


Quadruped::Quadruped()
{
}


Quadruped::~Quadruped()
{
}


void Quadruped::Initialize()
{
	std::cout << "Initializing Quadruped." << std::endl;
	legController = LegController(robotParameters.abdLinkLength, robotParameters.hipLinkLength, robotParameters.kneeLinkLength, robotParameters.kneeLinkYOffset);
	//legController = LegController(robotParameters.abdLinkLength, robotParameters.hipLinkLength, robotParameters.kneeLinkLength, robotParameters.kneeLinkYOffset);

	
	MathTypes::Mat3 rotorRotationalInertiaZ;

	MathTypes::Mat3 RY = GetRotationMatrix(M_PI / 2, COORD_AXIS::Y);

	// spatial inertias
	MathTypes::Mat3 abadRotationalInertia;
	abadRotationalInertia << 0.000381,	 0.000058,	 0.00000045, 
							 0.000058,	 0.000560,	 0.00000095,
							 0.00000045, 0.00000095, 0.000444;
	//abadRotationalInertia << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	MathTypes::Vec3 abadCOM(0, 0.036, 0);
	//MathTypes::Vec3 abadCOM(0, 0, 0);
	bodyInertiaParams.abdInertia = SpatialInertia(0.54f, abadCOM, abadRotationalInertia);
	//bodyInertiaParams.abdInertia = SpatialInertia(1, abadCOM, abadRotationalInertia);

	MathTypes::Mat3 hipRotationalInertia;
	hipRotationalInertia << 0.001983, 0.000245, 0.000013, 
							0.000245, 0.002103, 0.0000015,
							0.000013, 0.0000015, 0.000408;
	//hipRotationalInertia << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	MathTypes::Vec3 hipCOM(0, 0.016, -0.02);
	//MathTypes::Vec3 hipCOM(0, 0, 0);
	bodyInertiaParams.hipInertia = SpatialInertia(0.634f, hipCOM, hipRotationalInertia);
	//bodyInertiaParams.hipInertia = SpatialInertia(1, hipCOM, hipRotationalInertia);

	MathTypes::Mat3 kneeRotationalInertia, kneeRotationalInertiaRotated;
	kneeRotationalInertia << 0.000245,  0,		  0, 
							 0,			0.000248, 0, 
							 0,			0,		  0.000006;
	//kneeRotationalInertia << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	//MathTypes::Vec3 kneeCOM(0, 0, -0.061);
	MathTypes::Vec3 kneeCOM(0, 0, -0.209);
	//MathTypes::Vec3 kneeCOM(0, 0, 0);
	bodyInertiaParams.kneeInertia = SpatialInertia(0.064f, kneeCOM, kneeRotationalInertia);
	//bodyInertiaParams.kneeInertia = SpatialInertia(1, kneeCOM, kneeRotationalInertia);

	MathTypes::Mat3 bodyRotationalInertia;
	bodyRotationalInertia << 0.011253, 0, 0, 0, 0.036203, 0, 0, 0, 0.042673;
	//bodyRotationalInertia << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	MathTypes::Vec3 bodyCOM(0, 0, 0);
	bodyInertiaParams.floatingBodyInertia = SpatialInertia(robotParameters.bodyMass, bodyCOM, bodyRotationalInertia);
	//bodyInertiaParams.floatingBodyInertia = SpatialInertia(1, bodyCOM, bodyRotationalInertia);


	// locations
	bodyInertiaParams.abdLocation = MathTypes::Vec3(robotParameters.bodyLength, robotParameters.bodyWidth, 0) * 0.5;
	bodyInertiaParams.hipLocation = MathTypes::Vec3(0, robotParameters.abdLinkLength, 0);
	bodyInertiaParams.kneeLocation = MathTypes::Vec3(0, 0, -robotParameters.hipLinkLength);

	int baseID = 1;
	int bodyID = 1;
	int parentID = 0;

	SpatialTransform X;

	// Floating Base
	dynamics.AddBody(bodyInertiaParams.floatingBodyInertia, X, COORD_AXIS::X, parentID);
	bodyID++;
	parentID++;

	transformationChain.push_back(MathTypes::Mat4::Identity());
	
	MathTypes::Mat3 RIdent = MathTypes::Mat3::Identity();
	// Legs
	int side = -1;
	for (int leg = 0; leg < 4; leg++)
	{
		transformationChain.push_back(MathTypes::Mat4::Identity());
		transformationChain.push_back(MathTypes::Mat4::Identity());
		transformationChain.push_back(MathTypes::Mat4::Identity());
		footPos.push_back(MathTypes::Vec3::Zero());
		
		X = SpatialTransform(RIdent, GetLegSignedVector(bodyInertiaParams.abdLocation, leg));
		//SpatialInertia abdIn(0.54f, GetLegSignedVector(abadCOM, leg), abadRotationalInertia);
		//dynamics.AddBody(abdIn, X, COORD_AXIS::X, baseID);

		// Abd
		if (side < 0) {
			dynamics.AddBody(bodyInertiaParams.abdInertia.FlipAlongAxis(1), X, COORD_AXIS::X, baseID);
		}
		else {
			dynamics.AddBody(bodyInertiaParams.abdInertia, X, COORD_AXIS::X, baseID);
		}
		bodyID++;
		parentID++;

		// Hip
		//X = SpatialTransform(GetRotationMatrix(M_PI, COORD_AXIS::Z), GetLegSignedVector(-bodyInertiaParams.hipLocation, leg));
		X = SpatialTransform(RIdent, GetLegSignedVector(bodyInertiaParams.hipLocation, leg));
		//SpatialInertia hipIn(0.634f, GetLegSignedVector(hipCOM, leg), hipRotationalInertia);
		//dynamics.AddBody(hipIn, X, COORD_AXIS::Y, parentID);

		if (side < 0) {
			dynamics.AddBody(bodyInertiaParams.hipInertia.FlipAlongAxis(1), X, COORD_AXIS::Y, parentID);
		}
		else {
			dynamics.AddBody(bodyInertiaParams.hipInertia, X, COORD_AXIS::Y, parentID);
		}
		bodyID++;
		parentID++;

		// Knee
		X = SpatialTransform(RIdent, bodyInertiaParams.kneeLocation);
		//SpatialInertia kneeIn(0.064f, GetLegSignedVector(kneeCOM, leg), kneeRotationalInertia);
		//dynamics.AddBody(kneeIn, X, COORD_AXIS::Y, parentID);
		
		if (side < 0) {
			dynamics.AddBody(bodyInertiaParams.kneeInertia.FlipAlongAxis(1), X, COORD_AXIS::Y, parentID);
		}
		else {
			dynamics.AddBody(bodyInertiaParams.kneeInertia, X, COORD_AXIS::Y, parentID);
		}
		bodyID++;
		parentID++;


		dynamics.AddContactPoint(MathTypes::Vec3(0, 0, -robotParameters.kneeLinkLength), parentID);
		//dynamics.AddContactPoint(MathTypes::Vec3(0, 0, 0), parentID);

		side *= -1;
	}
}

void Quadruped::SetState(const State& newState)
{
	state = newState;
}

void Quadruped::SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces)
{
	dynamics.SetExternalForces(externalForces);
}

void Quadruped::SetExternalForceAt(int i, const MathTypes::Vec6& externalForce)
{
	dynamics.SetExternalForceAt(i, externalForce);
}


State Quadruped::GetState()
{
	return state;
}

void Quadruped::Integrate(const StateDot& dstate, double dt)
{
	state.bodyVelocity += dstate.bodyVelocityDDot * dt;

	for (int i = 0; i < 3; i++)
	{
		if (state.bodyVelocity[i] > 10) { state.bodyVelocity[i] = 10; }
		if (state.bodyVelocity[i] < -10) { state.bodyVelocity[i] = -10; }

		if (state.bodyVelocity[i + 3] > 10) { state.bodyVelocity[i + 3] = 10; }
		if (state.bodyVelocity[i + 3] < -10) { state.bodyVelocity[i + 3] = -10; }
	}

	state.bodyPose += state.bodyVelocity * dt;

	//state.bodyPose.template block<3, 1>(3, 0) += dstate.bodyPositionDot * deltaT;

	for (int i = 0; i < 12; i++)
	{
		double prevQ = state.q[i];
		state.qDot[i] += dstate.qDDot[i] * dt;

		if (state.qDot[i] > 20.0) { state.qDot[i] = 10.0; }
		if (state.qDot[i] < -20.0) { state.qDot[i] = -10.0; }

		state.q[i] += state.qDot[i] * dt;

		dynamics.torques[i] = (prevQ - state.q[i]) * 0.8;
	}
}

void Quadruped::UpdateKinematics()
{
	CalculateVisualTransformations();
	dynamics.UpdateKinematics(state);
}

void Quadruped::CalculateVisualTransformations()
{
	MathTypes::Mat3  R = GetRotationMatrix(state.bodyPose[0], COORD_AXIS::X) * GetRotationMatrix(state.bodyPose[1], COORD_AXIS::Y) * GetRotationMatrix(state.bodyPose[2], COORD_AXIS::Z);
	MathTypes::Vec3 position(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]);

	transformationChain[0].template topLeftCorner<3, 3>() = R;
	transformationChain[0](0, 3) = position[0];
	transformationChain[0](1, 3) = position[1];
	transformationChain[0](2, 3) = position[2];

	MathTypes::Mat4 Hbase_ref = MathTypes::Mat4::Identity();
	Hbase_ref.template topLeftCorner<3, 3>() = R.transpose();
	Hbase_ref.template topRightCorner<3, 1>() = -R * position;

	int footIdx = 0;

	for (int i = 1; i < transformationChain.size(); i++)
	{
		MathTypes::Mat4 T = MathTypes::Mat4::Identity();

		MathTypes::Mat4 TpjTrans = MathTypes::Mat4::Identity();
		MathTypes::Mat4 TpjRot = MathTypes::Mat4::Identity();
		MathTypes::Mat4 Tj = MathTypes::Mat4::Identity();

		MathTypes::Vec3 t = dynamics.Xl[i+1].GetTranslation();

		TpjRot.template topLeftCorner<3, 3>() = dynamics.Xl[i+1].GetRotation();
		TpjTrans(0, 3) = t[0];
		TpjTrans(1, 3) = t[1];
		TpjTrans(2, 3) = t[2];


		Tj.template topLeftCorner<3, 3>() = GetRotationMatrix(state.q[i - 1], dynamics.axis[i + 1]);

		transformationChain[i] = transformationChain[dynamics.parents[i + 1] - 1] * TpjRot * TpjTrans * Tj;


		if (i % 3 == 0)
		{
			MathTypes::Mat4 TFoot = MathTypes::Mat4::Identity();

			TFoot(0, 3) = dynamics.contactPoints[footIdx][0];
			TFoot(1, 3) = dynamics.contactPoints[footIdx][1];
			TFoot(2, 3) = dynamics.contactPoints[footIdx][2];

			TFoot = transformationChain[i] * TFoot;

			footPos[footIdx] = TFoot.template topRightCorner<3, 1>();
			dynamics.contactBodyGlobalPositions[footIdx] = TFoot.template topRightCorner<3, 1>();

			dynamics.contactParentGlobalPositions[footIdx] = transformationChain[i].template topRightCorner<3, 1>();
			dynamics.contactPointPositions[footIdx] = dynamics.contactBodyGlobalPositions[footIdx];
			dynamics.contactPointPositions[footIdx][2] = 0.0;
			CI_LOG_D("Foot " << footIdx << " Global Pos: " << footPos[footIdx].transpose());

			footIdx++;
		}
	}
}

StateDot Quadruped::RunArticulatedBodyAlgorithm()
{
	StateDot dState = dynamics.RunArticulatedBodyAlgorithm(state);
	return dState;
}

State Quadruped::StepDynamicsModel()
{
	//state = gait.step(legController, state, deltaT);
	//StateDot dState = dynamics.Step(state);

	//Integrate(dState);

	return state;
}