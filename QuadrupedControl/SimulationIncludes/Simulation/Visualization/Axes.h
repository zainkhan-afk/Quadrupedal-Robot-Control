#pragma once

#include "Simulation/Visualization/Cube.h"
#include "Simulation/Dynamics/Types.h"

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

