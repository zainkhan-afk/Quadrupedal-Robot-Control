#pragma once

#include "cinder/gl/gl.h"

namespace myprimitives {
	class Entity
	{
	public:
		Entity(const ci::gl::GlslProgRef& mGlsl, const ci::geom::Source* shape, 
			   const ci::vec3& _pos = ci::vec3(0), const ci::vec3& _rot = ci::vec3(0),
			   const ci::Color& _color = ci::Color(1, 1, 1));

		~Entity();

		void Draw();

	private:
		ci::gl::BatchRef objectRef;

	public:
		ci::vec3 position;
		ci::vec3 rotation;
		ci::Color color;
	};
}