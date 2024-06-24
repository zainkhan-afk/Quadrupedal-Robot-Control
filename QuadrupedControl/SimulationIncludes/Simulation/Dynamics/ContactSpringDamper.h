#ifndef CONTACTSPRINGDAMPER_H
#define CONTACTSPRINGDAMPER_H

#include "Simulation/Dynamics/Quadruped.h"

struct ContactInfo {
	int contactIdx;
	int contactParent;
	double penetration;
};

class ContactSpringDamper
{
public:
	ContactSpringDamper();
	~ContactSpringDamper();

	void AddContact(const ContactInfo& contact);

	void SolveContacts(Quadruped* robot);
	void CalculateContactForces(Quadruped* robot);
	void PropagateContactForces(Quadruped* robot);


private:
	double K = 1000;
	double D = 500;
	std::vector<ContactInfo> contactsToSolve;
};

#endif // !CONTACTSPRINGDAMPER_H
