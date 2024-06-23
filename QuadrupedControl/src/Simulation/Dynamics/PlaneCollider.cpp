#include "Simulation/Dynamics/PlaneCollider.h"


PlaneCollider::PlaneCollider(){}

bool PlaneCollider::IsColliding(const MathTypes::Vec3& contactPt, const double floorHeight, double& penetration)
{
	if (contactPt[2] <= floorHeight)
	{
		penetration = floorHeight - contactPt[2];
		return true;
	}
	else
	{
		penetration = 0.0;
		return false;
	}
}