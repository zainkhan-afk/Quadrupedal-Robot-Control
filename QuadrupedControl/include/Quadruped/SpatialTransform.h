#ifndef SPATIAL_TRANSFORM_H
#define SPATIAL_TRANSFORM_H

#include "Quadruped/QuadrupedCommon.h"
#include "Quadruped/Types.h"


class QUADRUPED_API SpatialTransform
{
public:
	SpatialTransform();
	SpatialTransform(const MathTypes::Mat3& _R, const MathTypes::Vec3& _p);
	SpatialTransform(const MathTypes::Vec3& _R, const MathTypes::Vec3& _p);
	~SpatialTransform();

	MathTypes::Mat6 GetSpatialForm();
	MathTypes::Mat6 GetSpatialFormTranspose();
	MathTypes::Mat6 GetSpatialFormForce();

	MathTypes::Mat3 GetRotation();
	MathTypes::Vec3 GetTranslation();

	MathTypes::Vec6 Apply(MathTypes::Vec6);


	SpatialTransform operator*(const SpatialTransform& rhs) const;


private:
	MathTypes::Mat3 R;
	MathTypes::Vec3 p;
};

#endif // !SPATIAL_TRANSFORM_H