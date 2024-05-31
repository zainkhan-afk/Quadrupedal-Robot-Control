#pragma once

#include "QuadrupedVisualizer/Cube.h"
#include "Quadruped/Types.h"

class Axes
{
	public:
		Axes(const ci::gl::GlslProgRef& Glsl);
		~Axes();

		void Draw();

	private:
		myprimitives::Cube* axesParts[3];
		float dimWid = 0.01f;
		float dimLen = 1.0f;
};

