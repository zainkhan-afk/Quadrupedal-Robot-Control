#include "QuadrupedVisualizer/Cube.h"

namespace myprimitives {
	Cube::Cube(const ci::gl::GlslProgRef& mGlsl, const ci::vec3& _pos, 
			   const ci::vec3& _rot, const ci::vec3& _posOffset,
			   const ci::vec3& _rotOffset, const ci::vec3& _size,
			   const ci::Color& _color) :
			   Entity(mGlsl, &ci::geom::Cube().size(_size), _pos, _rot, _posOffset, _rotOffset, _color),
			   size(_size) 
	{
	}

	Cube::~Cube()
	{
	}
}