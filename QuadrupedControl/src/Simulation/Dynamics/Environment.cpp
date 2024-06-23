#include "Simulation/Dynamics/Environment.h"

Environment::Environment() {
	robot = nullptr;
	plane = nullptr;
}

Environment::~Environment() {
	if (robot != nullptr) { delete robot; }
	if (plane != nullptr) { delete plane; }
	for (int i = 0; i < collisionObjects.size(); i++)
	{
		delete collisionObjects[i];
	}

	collisionObjects.clear();
}

void Environment::AddRobot(Quadruped* _robot) {
	robot = _robot;
}

void Environment::AddPlane(PlaneCollider* _plane) {
	plane = _plane;
}


void Environment::AddCollisionObject(Collider* collisionObject) {
	collisionObjects.push_back(collisionObject);
}

void Environment::Step(double dt) {
	CheckCollisions();
}

void Environment::CheckCollisions() {
	for (int i = 0; i < robot->dynamics.numContacts; i++)
	{
		double penetration;
		bool isCollision = plane->IsColliding(robot->dynamics.contactBodyGlobalPositions[i], 0, penetration);
		if (isCollision)
		{
			ContactInfo c;
			c.contactIdx = i;
			c.contactParent = robot->dynamics.contactPointsParents[i];
			c.penetration = penetration;

			contactSolver.AddContact(c);
		}
	}

	contactSolver.SolveContacts(robot);
}