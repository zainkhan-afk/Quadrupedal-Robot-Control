#pragma once

#include "QuadrupedVisualizer/Entity.h"

namespace myprimitives {
	class Sphere : public Entity
	{
	public:
		Sphere(const ci::gl::GlslProgRef& mGlsl, const ci::vec3& _pos = ci::vec3(0),
			const ci::vec3& _rot = ci::vec3(0), const ci::vec3& _posOffset = ci::vec3(0),
			const ci::vec3& _rotOffset = ci::vec3(0),
			const float _radius = 1.0f,
			const ci::Color& _color = ci::Color(0, 0, 1));

		~Sphere();

	private:
		float radius;
	};
}