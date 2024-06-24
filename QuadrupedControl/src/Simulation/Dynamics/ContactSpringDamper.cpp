#include "Simulation/Dynamics/ContactSpringDamper.h"


ContactSpringDamper::ContactSpringDamper(){}
ContactSpringDamper::~ContactSpringDamper() {}

void ContactSpringDamper::AddContact(const ContactInfo& contact)
{
	contactsToSolve.push_back(contact);
}

void ContactSpringDamper::SolveContacts(Quadruped* robot)
{
	CalculateContactForces(robot);
	PropagateContactForces(robot);
	contactsToSolve.clear();
}

void ContactSpringDamper::CalculateContactForces(Quadruped* robot)
{
	for (int i = 0; i < contactsToSolve.size(); i++)
	{
		ContactInfo c = contactsToSolve[i];

		double penetSqrt = std::sqrt(std::max(0.0, -c.penetration));

		double normalForceZ = penetSqrt * (-K * c.penetration - D * robot->dynamics.contactVelocity[c.contactIdx][2]);

		if (normalForceZ > 0) {
			robot->dynamics.fc[c.contactIdx] << 0, 0, normalForceZ;
		}
		else
		{
			robot->dynamics.fc[c.contactIdx] << 0, 0, 0;
		}
	}
}

void ContactSpringDamper::PropagateContactForces(Quadruped* robot)
{
	for (int i = 0; i < contactsToSolve.size(); i++)
	{
		ContactInfo c = contactsToSolve[i];
		int contactParent = robot->dynamics.contactPointsParents[c.contactIdx];
		MathTypes::Vec6 f_sp = MathTypes::Vec6::Zero();

		f_sp.topLeftCorner<3, 1>() = robot->dynamics.contactPointPositions[c.contactIdx].cross(robot->dynamics.fc[c.contactIdx]);
		f_sp.bottomLeftCorner<3, 1>() = robot->dynamics.fc[c.contactIdx];

		robot->dynamics.f[contactParent - 1] += f_sp;
	}
}
