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

	void AddBody(SpatialInertia I, dtypes::Mat6 pos, int axis, int parent);


private:
	void RunArticulatedBodyAlgorithm(const State& state, StateDot& dState);

private:
	int numLinks = 13;
	int currentIndex = 0;
	std::vector<SpatialInertia> linkInertias;
	std::vector<SpatialInertia> articulatedInertias;

	std::vector<int> axis;
	
	std::vector<dtypes::Mat6> Xl;
	std::vector<dtypes::Mat6> Xp;
	std::vector<dtypes::Mat6> Xb;

	std::vector<int> parents;
};


#endif