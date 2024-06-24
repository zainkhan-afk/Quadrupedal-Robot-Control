#pragma once

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

#include "Simulation/Visualization/Plane.h"
#include "Simulation/Visualization/Cube.h"
#include "Simulation/Visualization/Robot.h"
#include "Simulation/Visualization/Axes.h"

#include "Simulation/Dynamics/Quadruped.h"
#include "Simulation/Dynamics/State.h"
#include "Simulation/Dynamics/Environment.h"
#include "Simulation/Dynamics/PlaneCollider.h"

class QuadrupedVisualizer : public ci::app::App 
{
public:
    void                setup() override;
    void                resize() override;
    void                update() override;
    void                draw() override;

    void                cleanup() override;

private:
    ci::CameraPersp         mCam;
    ci::gl::GlslProgRef		mGlsl;
    myprimitives::Plane*    plane;

    int                     linkIdx;
    int                     footIdx;

    Environment             simEnviron;

    Axes*                   sceneAxes;
    Robot*                  myRobot;

    double                  ang = 0.0;
    double                  dt = 0.01;
};
