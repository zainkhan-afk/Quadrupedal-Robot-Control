#ifndef ROBOTDYNAMICS_H 
#define ROBOTDYNAMICS_H

#include "Quadruped/Types.h"
#include "Quadruped/State.h"
#include "Quadruped/SpatialInertia.h"

#include <eigen3/Eigen/StdVector>


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

	void AddBody(SpatialInertia I, MathTypes::Mat6 pos, COORD_AXIS axis, int parent);

	StateDot Step(const State& state);


private:
	StateDot RunArticulatedBodyAlgorithm(const State& state);

private:
	int numLinks = 13;
	int currentIndex = 0;
	std::vector<SpatialInertia> linkInertias;
	std::vector<SpatialInertia> articulatedInertias;


	std::vector<MathTypes::Vec6> v;
	std::vector<MathTypes::Vec6> S;

	std::vector<MathTypes::Mat6> Xl;
	std::vector<MathTypes::Mat6> Xp;
	std::vector<MathTypes::Mat6> Xb;

	std::vector<int> parents;
	std::vector<COORD_AXIS> axis;

};


#endif