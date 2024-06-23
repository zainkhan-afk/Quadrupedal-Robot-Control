#include "Simulation/Dynamics/ContactSpringDamper.h"


ContactSpringDamper::ContactSpringDamper(){}
ContactSpringDamper::~ContactSpringDamper() {}

void ContactSpringDamper::AddContact(const ContactInfo& contact)
{
	contactsToSolve.push_back(contact);
}

void ContactSpringDamper::SolveContacts(const Quadruped* robot)
{
	for (int i = 0; i < contactsToSolve.size(); i++)
	{
		
	}
	contactsToSolve.clear();
}
