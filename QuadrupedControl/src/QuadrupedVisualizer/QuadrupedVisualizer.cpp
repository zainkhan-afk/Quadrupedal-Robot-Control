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
    int numLinks = 13;
    
    plane = new myprimitives::Plane(mGlsl);
    //myRobot = new Robot(mGlsl);
    myChain = new Chain(mGlsl, numLinks);
   
    //robotModel.Initialize();
    //robotModel.SetExternalForceAt(2, f);

    chainModel.Initialize(numLinks);


    //state.bodyPosition = MathTypes::Vec3(0, 0, 1.5);
    state.q[0] = M_PI / 10.0f;
    state.q[1] = M_PI / 10.0f;
}

void QuadrupedVisualizer::resize()
{
    mCam.setPerspective(45, getWindowAspectRatio(), 1, 1000);
    gl::setMatrices(mCam);
}

void QuadrupedVisualizer::update() {
    /*state.q[0] = ang;
    state.q[1] = ang;*/


    //MathTypes::Vec6 f;
    //f << 0, 0, 0, ang, 0, 0;
    //chainModel.SetExternalForceAt(10, f);

    state = chainModel.StepDynamicsModel(state);
    chainModel.GetVisualTransformations(state);
    for (int i = 0; i < myChain->chainParameters.numLinks; i++) {
        MathTypes::Vec3 R = RotationMatrixToEuler(chainModel.dynamics.Xb[i].GetRotation());
        MathTypes::Vec3 P = chainModel.dynamics.Xb[i].GetTranslation();

        R = RotationMatrixToEuler(chainModel.transformationChain[i].template topLeftCorner<3, 3>());
        P[0] = chainModel.transformationChain[i](0, 3);
        P[1] = chainModel.transformationChain[i](1, 3);
        P[2] = chainModel.transformationChain[i](2, 3);

        myChain->SetRobotLinkPose(P, R, i);
    }

    /*state = robotModel.StepDynamicsModel(state);
    for (int i = 0; i < 13; i++) {
        MathTypes::Vec3 R = RotationMatrixToEuler(robotModel.dynamics.Xb[i].GetRotation());
        MathTypes::Vec3 P = robotModel.dynamics.Xb[i].GetTranslation();
        myRobot->SetRobotLinkPose(P, R, i);
    }*/
}

void QuadrupedVisualizer::draw() {
    //mCam.lookAt(vec3(4.0 * sin(ang), 4.0 * cos(ang), 4.0), vec3(0, 0, 0), vec3(0, 0, 1));
    mCam.lookAt(vec3(3.0f, -3.0, 1.0), vec3(0, 0, 0), vec3(0, 0, 1));
    
    gl::clear();
    gl::setMatrices(mCam);


    //plane->Draw();
    //myRobot->Draw();
    myChain->Draw();
    
    ang += 0.001;
}


void QuadrupedVisualizer::cleanup() {
    delete plane;
    //delete myRobot;
    delete myChain;
}

CINDER_APP(QuadrupedVisualizer, RendererGl(RendererGl::Options().msaa(16)))