#ifndef SPATIAL_TRANSFORM_H
#define SPATIAL_TRANSFORM_H

#include "Quadruped/Types.h"

class SpatialTransform
{
public:
	SpatialTransform();
	SpatialTransform(const MathTypes::Mat3& _R, const MathTypes::Vec3& _p);
	SpatialTransform(const MathTypes::Vec3& _R, const MathTypes::Vec3& _p);
	~SpatialTransform();


	SpatialTransform operator*(const SpatialTransform& rhs) const;


private:
	MathTypes::Mat3 R;
	MathTypes::Vec3 p;
};

#endif // !SPATIAL_TRANSFORM_H