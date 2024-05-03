#include "QuadrupedVisualizer/QuadrupedVisualizer.h"
#include "QuadrupedVisualizer/Resources.h"
#include "cinder/Log.h"

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
    mCam.setPerspective(30, getWindowAspectRatio(), 1, 1000);
    gl::setMatrices(mCam);
}

void QuadrupedVisualizer::update() {
    // Update logic if needed
    
    robotModel.dynamics.Xb[0];

}

void QuadrupedVisualizer::draw() {
    mCam.lookAt(vec3(2.0 * sin(ang), 2.0 * cos(ang), 2.0), vec3(0, 0, 1), vec3(0, 0, 1));

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