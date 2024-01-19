#ifndef ROBOTDYNAMICS_H 
#define ROBOTDYNAMICS_H

#include "Types.h"
#include "SpatialInertia.h"

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

template<typename T>
class RobotDynamics
{
public:
	RobotDynamics();
	~RobotDynamics();

	void AddBody(SpatialInertia<T> I, Mat6<T> pos, int parent);


private:
	void CalculateCompositeInertia();
	void CalculateLinkTreePositions();

private:
	std::vector<SpatialInertia<T>> robotInertias;
	std::vector<Mat6<T>> linkPositions;
	std::vector<int> parents;
	
	std::vector<SpatialInertia<T>> compositeInertia;
	std::vector<Mat6<T>> linkTreePositions;
};


#endif