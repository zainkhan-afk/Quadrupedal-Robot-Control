#ifndef CUBECHAIN_H
#define CUBECHAIN_H

#include "Quadruped/State.h"
#include "Quadruped/ChainDynamics.h"
class QUADRUPED_API CubeChain
{
public:
	CubeChain();
	~CubeChain();

	void Initialize(int _numLinks = 4);

	void SetState(const State& newState);
	
	void SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces);
	void SetExternalForceAt(int i, const MathTypes::Vec6& externalForce);

	State StepDynamicsModel(State& state);

	State GetState();

	void Integrate(State& state, const StateDot& dstate);

private:
	State state;
	int numLinks;
	float deltaT = 0.01f;
	float linkHieght{ 0.1f };
	float linkWidth{ 0.1f };
	float linkLength{ 0.1f };
	float linkMass{ 0.1f };

public:
	ChainDynamics dynamics;
};

#endif // CHAIN_H