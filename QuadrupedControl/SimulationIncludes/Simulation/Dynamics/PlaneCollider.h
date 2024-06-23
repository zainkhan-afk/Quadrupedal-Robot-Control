#ifndef PLANECOLLIDER_H
#define PLANECOLLIDER_H

#include "Simulation/Dynamics/Types.h"
#include "Simulation/Dynamics/Collider.h"

class PlaneCollider : public Collider
{
public:
	PlaneCollider();

	bool IsColliding(const MathTypes::Vec3& contactPt, const double floorHeight, double& penetration);
};

#endif // !PLANECOLLIDER_H
