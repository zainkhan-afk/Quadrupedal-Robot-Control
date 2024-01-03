#ifndef QUARDUPED_H
#define QUARDUPED_H

#include "State.h"

template <typename T>
class Quadruped
{
public:
	Quadruped() {}
	~Quadruped() {}



private:
	State state;
	StateDot stateDot;
};


#endif