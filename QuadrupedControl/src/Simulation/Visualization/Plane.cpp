#include "Simulation/Visualization/Plane.h"

namespace myprimitives {
	Plane::Plane(const ci::gl::GlslProgRef& mGlsl, const ci::vec3& _pos, 
				const ci::vec3& _rot, const ci::vec3& _posOffset,
				const ci::vec3& _rotOffset, const ci::vec2& _size, const ci::Color& _color,
				const ci::vec3& _normal) : 
				Entity(mGlsl, &ci::geom::Plane().size(_size).normal(_normal), _pos, _rot, _posOffset, _rotOffset, _color),
				size(_size) {}

	Plane::~Plane() {}
}