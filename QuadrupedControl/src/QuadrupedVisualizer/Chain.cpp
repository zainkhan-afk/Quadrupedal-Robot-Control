#include "QuadrupedVisualizer/Chain.h"


Chain::Chain(const ci::gl::GlslProgRef& Glsl) {
	for (int i = 0; i < chainParameters.numLinks; i++)
	{
		bodyParts[i] = new myprimitives::Cube(Glsl, ci::vec3(0), ci::vec3(0), ci::vec3(0, 0, chainParameters.cubeHeight / 2.0f), ci::vec3(0),
			ci::vec3(chainParameters.cubeWidth, chainParameters.cubeLength, chainParameters.cubeHeight));
	}
}

Chain::~Chain() {
	for (int i = 0; i < chainParameters.numLinks; i++)
	{
		delete bodyParts[i];
	}
}

void Chain::SetRobotLinkPose(MathTypes::Vec3 position, MathTypes::Vec3 rotation, int linkIdx)
{
	bodyParts[linkIdx]->position = ci::vec3(position[0], position[1], position[2]);
	bodyParts[linkIdx]->rotation = ci::vec3(rotation[0], rotation[1], rotation[2]);
}

void Chain::SetRobotLinkPose(MathTypes::Mat4 pose, int linkIdx)
{
	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			bodyParts[linkIdx]->transformMatrix[r][c] = pose(r, c);
		}
	}
	bodyParts[linkIdx]->transformMatrix[0][3] = pose(0, 3);
	bodyParts[linkIdx]->transformMatrix[1][3] = pose(1, 3);
	bodyParts[linkIdx]->transformMatrix[2][3] = pose(2, 3);
	bodyParts[linkIdx]->transformMatrix[3][3] = 1.0f;

}

void Chain::Draw() {
	for (int i = 0; i < chainParameters.numLinks; i++)
	{
		bodyParts[i]->Draw();
	}
}