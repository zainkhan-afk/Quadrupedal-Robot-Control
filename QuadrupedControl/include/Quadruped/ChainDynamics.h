#ifndef CHAINDYNAMICS_H
#define CHAINDYNAMICS_H

#include "Quadruped/Types.h"
#include "Quadruped/State.h"
#include "Quadruped/SpatialInertia.h"
#include "Quadruped/SpatialTransform.h"

class ChainDynamics
{

public:
	ChainDynamics();
	~ChainDynamics();

	void Initialize(int _numLinks = 4);

	void AddBody(SpatialInertia I, SpatialTransform X, COORD_AXIS axis, int parent);
	void SetExternalForces(const std::vector<MathTypes::Vec6>& externalForces);
	void SetExternalForceAt(int i, const MathTypes::Vec6& externalForce);
	
	StateDot Step(const State& state);


private:
	StateDot RunArticulatedBodyAlgorithm(const State& state);


public:
	std::vector<SpatialTransform> Xb;
	std::vector<float> torques;
	int numLinks;
	std::vector<COORD_AXIS> axis;
	std::vector<SpatialTransform> Xl;
	std::vector<int> parents;
	MathTypes::Vec6 G = MathTypes::Vec6::Zero();

private:
	int currentIndex = 0;

	std::vector<SpatialInertia> linkInertias;
	std::vector<SpatialInertia> articulatedInertias;


	std::vector<MathTypes::Vec6> v;
	std::vector<MathTypes::Vec6> S;
	std::vector<MathTypes::Vec6> c;
	std::vector<MathTypes::Vec6> pa;

	std::vector<MathTypes::Vec6> U;
	std::vector<MathTypes::Vec6> a;

	std::vector<MathTypes::Vec6> f;

	std::vector<SpatialTransform> Xp;
	

	std::vector<float> D;
	std::vector<float> u;
};

#endif // CHAINDYNAMICS_H
