#include "QuadrupedVisualizer/Entity.h"

namespace myprimitives {
	Entity::Entity(const ci::gl::GlslProgRef& mGlsl, const ci::geom::Source* shape, 
				   const ci::vec3& _pos, const ci::vec3& _rot, const ci::Color& _color) :
				   position(_pos), rotation(_rot), color(_color)
	{
		objectRef = ci::gl::Batch::create(*shape, mGlsl);
	}

	Entity::~Entity()
	{
	}

	void Entity::Draw()
	{
		ci::gl::pushModelMatrix();
		ci::gl::translate(position);
		ci::gl::rotate(rotation);
		
		ci::gl::color(color);
		objectRef->draw();
		ci::gl::popModelMatrix();
	}
}