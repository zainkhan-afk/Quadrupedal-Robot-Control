
#include "QuadrupedVisualizer/Sphere.h"

namespace myprimitives {
	Sphere::Sphere(const ci::gl::GlslProgRef& mGlsl, const ci::vec3& _pos,
		const ci::vec3& _rot, const ci::vec3& _posOffset,
		const ci::vec3& _rotOffset, const float _radius,
		const ci::Color& _color) :
			Entity(mGlsl, &ci::geom::Sphere().radius(_radius), _pos, _rot, _posOffset, _rotOffset, _color),
			radius(_radius)
	{
	}

	Sphere::~Sphere()
	{
	}
}