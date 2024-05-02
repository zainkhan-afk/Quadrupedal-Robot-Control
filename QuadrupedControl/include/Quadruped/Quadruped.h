#ifndef QUARDUPED_H
#define QUARDUPED_H

#include "Quadruped/State.h"
#include "Quadruped/LegController.h"
#include "Quadruped/SpatialInertia.h"
#include "Quadruped/RobotDynamics.h"

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


#ifdef QUADRUPED_LIB
#    define QUADRUPED_API __declspec(dllexport)
#else
#    define QUADRUPED_API __declspec(dllimport)
#endif


class QUADRUPED_API Quadruped
{
public:
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Quadruped();
	~Quadruped();

	void Initialize();

	void SetFloatingBaseStateFromIMU(float IMUData[]);
	void SetJointsStateFromSensors(float jointStateData[]);
	void SetState(State& newState);
	void SetState(float IMUData[], float jointStateData[]);

	State GetState();
	dtypes::Vec12 LegPositionForState();

private:
	struct 
	{
		std::vector<SpatialInertia> I;
		int numDoF{ 18 }, numActuatedDoF{ 12 }, numUnActuatedDoF{ 6 };

		float bodyLength{ 0.38 }, bodyWidth{ 0.098 }, bodyHeight{ 0.1 };
		float abdGearRatio{ 6 }, hipGearRatio{ 6 }, kneeGearRatio{ 9.33 };
		
		float abdLinkLength{ 0.062 }, hipLinkLength{ 0.209 }, kneeLinkLength{ 0.195 }, kneeLinkYOffset{ 0.004 }, maxLegLength{ 0.409 };

		float bodyMass{ 3.3 };

	} robotParameters;

	struct
	{
		SpatialInertia floatingBodyInertia, abdInertia, hipInertia, kneeInertia;
		dtypes::Vec3 abdLocation, hipLocation, kneeLocation;
	} bodyInertiaParams;

	State state;
	StateDot stateDot;
	LegController legController;
	RobotDynamics dynamics;
};


#endif
