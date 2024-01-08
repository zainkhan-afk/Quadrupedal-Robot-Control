#ifndef QUARDUPED_H
#define QUARDUPED_H

#include "State.h"
#include "LegController.h"

template <typename T>
class Quadruped
{
public:
	Quadruped();
	~Quadruped();

	void SetFloatingBaseStateFromIMU(double IMUData[]);
	void SetJointsStateFromSensors(double jointStateData[]);
	void SetState(State<T>& newState);
	void SetState(double IMUData[], double jointStateData[]);

	State<T> GetState();

private:
	struct 
	{
		int numDoF{ 18 }, numActuatedDoF{ 12 }, numUnActuatedDoF{ 6 };

		T bodyLength, bodyWidth, bodyHeight;
		T abdGearRatio, hipGearRatio, kneeGearRatio;
		
		T abdLinkLength, hipLinkLength, kneeLinkLength, kneeLinkYOffset, maxLegLength;

		T bodyMass;

	} robotParameters;

	/*SpatialInertia<T> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia,
		_hipRotorInertia, _kneeRotorInertia, _bodyInertia;*/
	Vec3<T> _abadLocation, _abadRotorLocation, _hipLocation, _hipRotorLocation,
		_kneeLocation, _kneeRotorLocation;


	State<T> state;
	StateDot<T> stateDot;
	LegController<T> legController;
};


#endif
