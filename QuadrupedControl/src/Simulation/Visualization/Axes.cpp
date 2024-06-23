#include "Simulation/Visualization/Axes.h"

Axes::Axes(const ci::gl::GlslProgRef& Glsl) {
	for (int i = 0; i < 3; i++)
	{
		if (i == 0) {
			axesParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0), ci::vec3(0), ci::vec3(0, 0, dimLen/2), 
								ci::vec3(0), ci::vec3(dimWid, dimWid, dimLen), ci::vec3(0, 0, 1));
		}
		else if (i == 1) {
			axesParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0), ci::vec3(0), ci::vec3(0, dimLen / 2, 0), 
								ci::vec3(0), ci::vec3(dimWid, dimLen, dimWid), ci::vec3(0, 1, 0));
		}
		else if (i == 2) {
			axesParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0), ci::vec3(0), ci::vec3(dimLen / 2, 0, 0), 
								ci::vec3(0), ci::vec3(dimLen, dimWid, dimWid), ci::vec3(1, 0, 0));
		}
	}
}

Axes::~Axes() {
	for (int i = 0; i < 3; i++)
	{
		delete axesParts[i];
	}
}

void Axes::Draw() {
	for (int i = 0; i < 3; i++)
	{
		axesParts[i]->Draw();
	}
}