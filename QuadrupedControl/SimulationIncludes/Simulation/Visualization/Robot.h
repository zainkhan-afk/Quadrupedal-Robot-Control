#pragma once

#include "Simulation/Visualization/Cube.h"
#include "Simulation/Visualization/Sphere.h"
#include "Simulation/Dynamics/Types.h"


class Robot
{
public:
	Robot(const ci::gl::GlslProgRef& Glsl);
	~Robot();

	void SetRobotLinkPose(MathTypes::Vec3 position, MathTypes::Vec3 rotation, int linkIdx);
	void SetRobotFootPosition(MathTypes::Vec3 position, int footIdx);

	void Draw();

private:
	myprimitives::Cube* bodyParts[13];
	myprimitives::Sphere* feet[4];

	struct
	{
		float bodyLength{ 0.38f }, bodyWidth{ 0.098f }, bodyHeight{ 0.025f };
		
		float abdLinkLength{ 0.062f }, hipLinkLength{ 0.209f }, kneeLinkLength{ 0.195f }, kneeLinkYOffset{ 0.004f }, maxLegLength{ 0.409f };

	} robotParameters;
};

