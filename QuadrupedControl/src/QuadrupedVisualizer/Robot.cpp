#include "QuadrupedVisualizer/Robot.h"

Robot::Robot(const ci::gl::GlslProgRef& Glsl) {
	for (int i = 0; i < 13; i++)
	{
		if (i == 0) {
			bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0, 0, 1), ci::vec3(0,0,0), 
						ci::vec3(robotParameters.bodyLength, robotParameters.bodyWidth, robotParameters.bodyHeight));
		}
		else {
			int index = i - 1;
			if (index % 3 == 0) {
				bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0, i * 0.1, 1), ci::vec3(0),
					ci::vec3(robotParameters.abdLinkLength, robotParameters.bodyHeight, robotParameters.bodyHeight));
			}
			else if (index % 3 == 1) {
				bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0, i * 0.1, 1), ci::vec3(0),
					ci::vec3(robotParameters.hipLinkLength, robotParameters.bodyHeight, robotParameters.bodyHeight));
			}
			else if (index % 3 == 2) {
				bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0, i * 0.1, 1), ci::vec3(0),
					ci::vec3(robotParameters.kneeLinkLength, robotParameters.bodyHeight, robotParameters.bodyHeight));
			}
		}
	}
}
Robot::~Robot() {
	for (int i = 0; i < 13; i++)
	{
		delete bodyParts[i];
	}
}

void Robot::Draw() {
	for (int i = 0; i < 13; i++)
	{
		bodyParts[i]->Draw();
	}
}