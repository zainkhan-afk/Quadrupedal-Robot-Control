#pragma once

#include "Simulation/Visualization/Entity.h"

namespace myprimitives {
	class Cube : public Entity
	{
	public:
		Cube(const ci::gl::GlslProgRef& mGlsl, const ci::vec3& _pos = ci::vec3(0), 
			 const ci::vec3& _rot = ci::vec3(0), const ci::vec3& _posOffset = ci::vec3(0), 
			 const ci::vec3& _rotOffset = ci::vec3(0),
			 const ci::vec3& _size = ci::vec3(1),
			 const ci::Color& _color = ci::Color(0, 0, 1));

		~Cube();

	private:
		ci::vec3 size;
	};
}