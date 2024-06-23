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

	void SolveContacts(const Quadruped* robot);

private:
	std::vector<ContactInfo> contactsToSolve;
};

#endif // !CONTACTSPRINGDAMPER_H
