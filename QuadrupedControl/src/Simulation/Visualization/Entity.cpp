#include "Simulation/Visualization/Entity.h"

namespace myprimitives {
	Entity::Entity(const ci::gl::GlslProgRef& mGlsl, const ci::geom::Source* shape, 
				   const ci::vec3& _pos, const ci::vec3& _rot, const ci::vec3& _posOffset, 
				   const ci::vec3& _rotOffset, const ci::Color& _color) :
				   position(_pos), rotation(_rot), color(_color), positionOffset(_posOffset), rotationOffset(_rotOffset)
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
		ci::gl::translate(positionOffset);
		
		ci::gl::color(color);
		objectRef->draw();
		ci::gl::popModelMatrix();
	}
}