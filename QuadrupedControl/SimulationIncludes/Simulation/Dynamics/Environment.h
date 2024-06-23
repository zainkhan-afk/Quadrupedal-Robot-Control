#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "Simulation/Dynamics/Quadruped.h"
#include "Simulation/Dynamics/Collider.h"
#include "Simulation/Dynamics/PlaneCollider.h"
#include "Simulation/Dynamics/ContactSpringDamper.h"
#include "Simulation/Dynamics/State.h"

class Environment
{
public:
	Environment();
	~Environment();

	void AddRobot(Quadruped* _robot);
	void AddPlane(PlaneCollider* _plane);
	void AddCollisionObject(Collider* collisionObject);

	void Step(double dt);

	State GetRobotState() { return robotState; };

private:
	void CheckCollisions();
	void UpdateCollisionForces();

private:
	State robotState;
	Quadruped* robot;
	PlaneCollider* plane;

	std::vector<Collider*> collisionObjects;
	ContactSpringDamper contactSolver;
};

#endif // !ENVIRONMENT_H
