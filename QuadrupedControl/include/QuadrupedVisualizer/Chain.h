#pragma once
#include "QuadrupedVisualizer/Cube.h"
#include "Quadruped/Types.h"


class Chain
{
public:
	Chain(const ci::gl::GlslProgRef& Glsl);
	~Chain();

	void SetRobotLinkPose(MathTypes::Vec3 position, MathTypes::Vec3 rotation, int linkIdx);
	void SetRobotLinkPose(MathTypes::Mat4 pose, int linkIdx);

	void Draw();

private:
	myprimitives::Cube* bodyParts[13];

	struct
	{
		float cubeLength{ 0.1f }, cubeWidth{ 0.1f }, cubeHeight{ 0.25f };
		int numLinks{ 4 };

	} chainParameters;
};


