#ifndef SPATIAL_TRANSFORM_H
#define SPATIAL_TRANSFORM_H

#include "Simulation/Dynamics/QuadrupedCommon.h"
#include "Simulation/Dynamics/Types.h"


class SpatialTransform
{
public:
	SpatialTransform();
	SpatialTransform(const MathTypes::Mat3& _R, const MathTypes::Vec3& _p);
	SpatialTransform(const MathTypes::Vec3& _R, const MathTypes::Vec3& _p);
	~SpatialTransform();

	MathTypes::Mat6 GetSpatialForm() const;
	MathTypes::Mat6 GetSpatialFormTranspose() const;
	MathTypes::Mat6 GetSpatialFormForce() const;
	SpatialTransform GetInverse() const;

	MathTypes::Mat3 GetRotation() const;
	MathTypes::Vec3 GetTranslation() const;

	//MathTypes::Vec6 Apply(MathTypes::Vec6);
	//MathTypes::Vec6 ApplyTranspose(MathTypes::Vec6);


	SpatialTransform operator*(const SpatialTransform& rhs) const;


private:
	MathTypes::Mat3 R;
	MathTypes::Vec3 p;
};

#endif // !SPATIAL_TRANSFORM_H