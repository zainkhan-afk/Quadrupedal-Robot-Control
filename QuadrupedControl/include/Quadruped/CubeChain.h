#ifndef CUBECHAIN_H
#define CUBECHAIN_H

#include "Quadruped/State.h"
#include "Quadruped/ChainDynamics.h"
class QUADRUPED_API CubeChain
{
public:
	CubeChain();
	~CubeChain();

	void Initialize();

	void SetState(const State& newState);
	
	void SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces);
	void SetExternalForceAt(int i, const MathTypes::Vec6& externalForce);

	State StepDynamicsModel(State& state);

	State GetState();

	void Integrate(State& state, const StateDot& dstate);

private:
	State state;
	int numLinks = 4;
	float deltaT = 0.001f;
	float linkHieght{ 0.25f };
	float linkWidth{ 0.1f };
	float linkLength{ 0.1f };
	float linkMass{ 1.0f };

public:
	ChainDynamics dynamics;
};

#endif // CHAIN_H