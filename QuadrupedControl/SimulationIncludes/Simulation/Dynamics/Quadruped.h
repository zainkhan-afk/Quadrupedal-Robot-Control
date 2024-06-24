#ifndef QUARDUPED_H
#define QUARDUPED_H

#include "Simulation/Dynamics/State.h"
#include "Simulation/Dynamics/LegController.h"
#include "Simulation/Dynamics/SpatialInertia.h"
#include "Simulation/Dynamics/RobotDynamics.h"
#include "Simulation/Dynamics/Gait.h"

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


class Quadruped
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

	void UpdateKinematics();
	void CalculateVisualTransformations();

	StateDot RunArticulatedBodyAlgorithm();

	State StepDynamicsModel();

	State GetState();
	void Integrate(const StateDot& dstate, double dt);

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

	State state;
	StateDot stateDot;
	LegController legController;
	Gait gait;


public:
	RobotDynamics dynamics;
	std::vector<MathTypes::Mat4> transformationChain;
	std::vector<MathTypes::Vec3> footPos;
};


#endif
