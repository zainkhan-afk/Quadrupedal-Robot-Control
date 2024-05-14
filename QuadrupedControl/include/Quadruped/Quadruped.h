#ifndef QUARDUPED_H
#define QUARDUPED_H

#include "Quadruped/State.h"
#include "Quadruped/LegController.h"
#include "Quadruped/SpatialInertia.h"
#include "Quadruped/RobotDynamics.h"
#include "Quadruped/QuadrupedCommon.h"
/*
* The quadruped has 4 legs, the legs are arranged in the follwing way when viewed from top.
* 
*						Front
* 
* 
* 
* 
*						Head
*				Leg 2			Leg 1
* 
* 
*Left											Right
* 
* 
*				Leg 3			Leg 4
* 
* 
* 
* 
* 
* 
*						Back
*/

#define _USE_MATH_DEFINES


class QUADRUPED_API Quadruped
{
public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Quadruped();
	~Quadruped();

	void Initialize();

	void SetFloatingBaseStateFromIMU(float IMUData[]);
	void SetJointsStateFromSensors(float jointStateData[]);
	void SetState(const State& newState);
	void SetState(float IMUData[], float jointStateData[]);
	void SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces);
	void SetExternalForceAt(int i, const MathTypes::Vec6& externalForce);

	State StepDynamicsModel(State& state);

	State GetState();

	void Integrate(State& state, const StateDot& dstate);

private:
	struct 
	{
		std::vector<SpatialInertia> I;
		int numDoF{ 18 }, numActuatedDoF{ 12 }, numUnActuatedDoF{ 6 };

		double bodyLength{ 0.38 }, bodyWidth{ 0.098 }, bodyHeight{ 0.1 };
	
		double abdLinkLength{ 0.062 }, hipLinkLength{ 0.209 }, kneeLinkLength{ 0.195 }, kneeLinkYOffset{ 0.004 }, maxLegLength{ 0.409 };

		double bodyMass{ 3.3 };

	} robotParameters;

	struct
	{
		SpatialInertia floatingBodyInertia, abdInertia, hipInertia, kneeInertia;
		MathTypes::Vec3 abdLocation, hipLocation, kneeLocation;
	} bodyInertiaParams;

	double deltaT = 0.001;
	State state;
	StateDot stateDot;
	LegController legController;

public:
	RobotDynamics dynamics;
};


#endif
