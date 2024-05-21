#include "QuadrupedVisualizer/Robot.h"

Robot::Robot(const ci::gl::GlslProgRef& Glsl) {
	int sign = -1;
	for (int i = 0; i < 13; i++)
	{
		if (i == 0) {
			bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0), ci::vec3(0), ci::vec3(0), ci::vec3(0),
						ci::vec3(robotParameters.bodyLength, robotParameters.bodyWidth, robotParameters.bodyHeight));
		}
		else {
			int index = i - 1;
			if (index % 3 == 0) {
				bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0), ci::vec3(0), ci::vec3(0, sign*robotParameters.abdLinkLength/2.0f, 0), ci::vec3(0, 0, 0),
					ci::vec3(robotParameters.bodyHeight, robotParameters.abdLinkLength, robotParameters.bodyHeight));
				sign *= -1;
			}
			else if (index % 3 == 1) {
				//-robotParameters.hipLinkLength / 2)
				bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0), ci::vec3(0), ci::vec3(0, 0, -robotParameters.hipLinkLength/2.0f), ci::vec3(0),
					ci::vec3(robotParameters.bodyHeight, robotParameters.bodyHeight, robotParameters.hipLinkLength));
			}
			else if (index % 3 == 2) {
				//-robotParameters.kneeLinkLength / 2
				bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0), ci::vec3(0), ci::vec3(0, 0, -robotParameters.kneeLinkLength / 2.0f), ci::vec3(0),
					ci::vec3(robotParameters.bodyHeight, robotParameters.bodyHeight, robotParameters.kneeLinkLength));
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

void Robot::SetRobotLinkPose(MathTypes::Vec3 position, MathTypes::Vec3 rotation, int linkIdx)
{
	bodyParts[linkIdx]->position = ci::vec3(position[0], position[1], position[2]);
	bodyParts[linkIdx]->rotation = ci::vec3(rotation[0], rotation[1], rotation[2]);
}

void Robot::Draw() {
	for (int i = 0; i < 13; i++)
	{
		bodyParts[i]->Draw();
	}
}