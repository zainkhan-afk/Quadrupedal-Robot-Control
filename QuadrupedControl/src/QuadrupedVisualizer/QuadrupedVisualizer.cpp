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

    int numLinks = 8;
    //
    plane = new myprimitives::Plane(mGlsl);
    //myRobot = new Robot(mGlsl);
    myChain = new Chain(mGlsl, numLinks);

    //robotModel.Initialize();
    //robotModel.SetExternalForceAt(0, f);

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
    //state.q[0] = ang;
    //chainModel.dynamics.torques[0] = (state.q[0] - M_PI) * 3;
    //chainModel.dynamics.torques[2] = (state.q[2] - M_PI) * 3;
    //chainModel.dynamics.torques[11] = (state.q[11] - M_PI / 6.0) * 0.2;

    state = chainModel.StepDynamicsModel(state);
    chainModel.GetVisualTransformations(state);
    for (int i = 0; i < myChain->chainParameters.numLinks; i++) {
        //MathTypes::Vec3 R = RotationMatrixToEuler(chainModel.dynamics.Xb[i].GetRotation());
        //MathTypes::Vec3 P = chainModel.dynamics.Xb[i].GetTranslation();

        MathTypes::Vec3 R = RotationMatrixToEuler(chainModel.transformationChain[i].template topLeftCorner<3, 3>());
        //MathTypes::Vec3 R = MathTypes::Vec3::Zero();
        MathTypes::Vec3 P = MathTypes::Vec3::Zero();

        P[0] = chainModel.transformationChain[i](0, 3);
        P[1] = chainModel.transformationChain[i](1, 3);
        P[2] = chainModel.transformationChain[i](2, 3);


        myChain->SetRobotLinkPose(P, R, i);
    }

    /*if (ang < 0.1)
    {
        MathTypes::Vec6 f;
        f << 0, 0, 0, 10, 0, 0;
        chainModel.SetExternalForceAt(4, f);
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
    
    ang += 0.01;
}


void QuadrupedVisualizer::cleanup() {
    delete plane;
    //delete myRobot;
    delete myChain;
}

CINDER_APP(QuadrupedVisualizer, RendererGl(RendererGl::Options().msaa(16)))