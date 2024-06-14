#ifndef ROBOTDYNAMICS_H 
#define ROBOTDYNAMICS_H

#include "Simulation/Dynamics/Types.h"
#include "Simulation/Dynamics/State.h"
#include "Simulation/Dynamics/SpatialInertia.h"
#include "Simulation/Dynamics/SpatialTransform.h"
#include "Simulation/Dynamics/LegController.h"
#include <eigen3/Eigen/StdVector>


#define _USE_MATH_DEFINES


/*
* Robot Chain Example
* 
* 
*				_________ Floating Base _________ ID - 0
*				O								O
*				|								|
*				| Thigh	ID - 1					| Thigh	ID - 3
*				|								|
*				O								O
*				|								|
*				| Knee	ID - 2					| Knee	ID - 4
*				|								|
*				|								|
* 
*
*	Link with ID - 0 is the parent and ID 1 and ID 3 are its children which have their own children.
* This hierarchy will be used to update the positions of the robot links. NOTE: The above is just an example
* configuration, not the actual robot.
*
*/


class RobotDynamics
{
public:
	RobotDynamics();
	~RobotDynamics();

	void AddBody(SpatialInertia I, SpatialTransform X, COORD_AXIS axis, int parent);
	void AddContactPoint(MathTypes::Vec3 contactPoint, int parent);
	void SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces);
	void SetExternalForceAt(int i, const MathTypes::Vec6& externalForce);

	StateDot Step(const State& state);


private:
	StateDot RunArticulatedBodyAlgorithm(const State& state);

public:
	std::vector<SpatialTransform> Xb;
	int numLinks = 13;
	std::vector<int> parents;
	std::vector<SpatialTransform> Xl;
	std::vector<SpatialTransform> Xc;
	std::vector<SpatialTransform> Xcb;
	std::vector<COORD_AXIS> axis;
	std::vector<float> torques;
	std::vector<MathTypes::Vec6> a;

	std::vector<MathTypes::Vec3> contactPoints;
	std::vector<int> contactPointsParents;
	std::vector<MathTypes::Vec3> fc;


	MathTypes::Vec6 G = MathTypes::Vec6::Zero();




private:
	int currentIndex = 0;

	std::vector<SpatialInertia> linkInertias;
	std::vector<SpatialInertia> articulatedInertias;


	std::vector<MathTypes::Vec6> v;
	std::vector<MathTypes::Vec6> S;
	std::vector<MathTypes::Vec6> c;
	std::vector<MathTypes::Vec6> pa;

	std::vector<MathTypes::Vec6> U;

	std::vector<MathTypes::Vec6> f;

	std::vector<SpatialTransform> Xp;

	std::vector<float> D;
	std::vector<float> u;

	Eigen::ColPivHouseholderQR<MathTypes::Mat6> invIA0;

};


#endif