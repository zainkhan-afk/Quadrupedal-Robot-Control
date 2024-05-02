#pragma once

#include "QuadrupedVisualizer/Entity.h"


namespace myprimitives {
	class Plane : public Entity
	{
	public:
		Plane(const ci::gl::GlslProgRef& mGlsl, const ci::vec3& _pos = ci::vec3(0), 
			  const ci::vec3& _rot = ci::vec3(0), const ci::vec2& _size = ci::vec2(10), 
			  const ci::Color& _color = ci::Color(0.5, 0.5, 0), 
			  const ci::vec3& _normal = ci::vec3(0, 0, 1));

		~Plane();

	private:
		ci::vec2 size;
	};
}

