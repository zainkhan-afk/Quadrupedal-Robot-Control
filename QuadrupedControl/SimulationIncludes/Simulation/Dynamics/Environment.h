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
	std::vector<MathTypes::Mat4> GetRobotTransformationChain();

	State GetRobotState() { return robotState; };

private:
	void CheckCollisions();
	void UpdateCollisionForces();

public:
	Quadruped* robot;

private:
	State robotState;
	PlaneCollider* plane;

	std::vector<Collider*> collisionObjects;
	ContactSpringDamper contactSolver;
};

#endif // !ENVIRONMENT_H
