#include "QuadrupedVisualizer/QuadrupedVisualizer.h"
#include "QuadrupedVisualizer/Resources.h"
#include "cinder/Log.h"

#include "Quadruped/Spatial.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void QuadrupedVisualizer::setup() {
    gl::enableDepthWrite();
    gl::enableDepthRead();

    try {
        mGlsl = gl::GlslProg::create(loadResource(RES_VERT_SHADER), loadResource(RES_FRAG_SHADER));
        CI_LOG_I("Loaded shader");
    }
    catch (const std::exception& e) {
        CI_LOG_E("Shader Error: " << e.what());
    }

    //
    plane = new myprimitives::Plane(mGlsl);
    myRobot = new Robot(mGlsl);

    robotModel.Initialize();
}

void QuadrupedVisualizer::resize()
{
    mCam.setPerspective(45, getWindowAspectRatio(), 1, 1000);
    gl::setMatrices(mCam);
}

void QuadrupedVisualizer::update() {
    // Update logic if needed
    state.bodyPosition = MathTypes::Vec3(0, 0, 0.5f);
    
    int offset = 2;
    state.q[0 + offset] = 2 * ang;
    state.q[3 + offset] = 2 * ang;
    state.q[6 + offset] = 2 * ang;
    state.q[9 + offset] = 2 * ang;

    state = robotModel.StepDynamicsModel(state);

    for (int i = 0; i < 13; i++) {
        MathTypes::Vec3 R = RotationMatrixToEuler(SpatialToRotMat(robotModel.dynamics.Xb[i]));
        MathTypes::Vec3 P = SpatialToTranslation(robotModel.dynamics.Xb[i]);

        myRobot->SetRobotLinkPose(P, R, i);
    }
}

void QuadrupedVisualizer::draw() {
    //mCam.lookAt(vec3(2.0, 1.0 * cos(ang), 1.0), vec3(0, 0, 0), vec3(0, 0, 1));
    mCam.lookAt(vec3(2.0, -2.0, 1.0), vec3(0, 0, 0), vec3(0, 0, 1));

    gl::clear();
    gl::setMatrices(mCam);


    plane->Draw();
    myRobot->Draw();

    ang += 0.01;
}


void QuadrupedVisualizer::cleanup() {
    delete plane;
    delete myRobot;
}

CINDER_APP(QuadrupedVisualizer, RendererGl(RendererGl::Options().msaa(16)))