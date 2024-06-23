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

class QuadrupedVisualizer : public ci::app::App 
{
public:
    void                setup() override;
    void                resize() override;
    void                update() override;
    void                draw() override;
    void                InitializeRobot();

    void                cleanup() override;

private:
    ci::CameraPersp         mCam;
    ci::gl::GlslProgRef		mGlsl;
    myprimitives::Plane*    plane;

    int                     linkIdx;
    int                     footIdx;

    Quadruped               robotModel;
    State                   state;

    Axes*                   sceneAxes;
    Robot*                  myRobot;

    double                  ang = 0.0;
};
