#pragma once

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

#include "QuadrupedVisualizer/Plane.h"
#include "QuadrupedVisualizer/Cube.h"
#include "QuadrupedVisualizer/Robot.h"
#include "QuadrupedVisualizer/Chain.h"
#include "QuadrupedVisualizer/Axes.h"

#include "Quadruped/Quadruped.h"
#include "Quadruped/CubeChain.h"
#include "Quadruped/State.h"

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

    Quadruped               robotModel;
    CubeChain               chainModel;
    State                   state;

    Axes*                   sceneAxes;
    Robot*                  myRobot;
    Chain*                  myChain;

    double                  ang = 0.0;
};
