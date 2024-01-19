#ifndef ROBOTDYNAMICS_H 
#define ROBOTDYNAMICS_H

#include "Types.h"
#include "SpatialInertia.h"

template<typename T>
class RobotDynamics
{
public:
	RobotDynamics();
	~RobotDynamics();

	void AddBody(MatSp<T> I, Mat6<T> pos);

private:
	struct
	{
		std::vector<SpatialInertia<T>> robotInertias;
		std::vector<Mat6<T>> linkPositions;
	} parameters;
};


#endif // !