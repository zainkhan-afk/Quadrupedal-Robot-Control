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
    //
    plane = new myprimitives::Plane(mGlsl);
    //myRobot = new Robot(mGlsl);
    myChain = new Chain(mGlsl, numLinks);

    MathTypes::Vec6 f;

    f << 0, 0, 0, 2, 2, 0;

    //robotModel.Initialize();
    //robotModel.SetExternalForceAt(0, f);

    chainModel.Initialize(numLinks);
    //chainModel.SetExternalForceAt(3, f);

    state.bodyPosition = MathTypes::Vec3(0, 0, 1.5f);
    state.q[7] = M_PI / 5.0f;
    //state.q[1] = M_PI / 5.0f;
}

void QuadrupedVisualizer::resize()
{
    mCam.setPerspective(45, getWindowAspectRatio(), 1, 1000);
    gl::setMatrices(mCam);
}

void QuadrupedVisualizer::update() {
    // Update logic if needed
    //state.bodyPosition = MathTypes::Vec3(0, 0, 0.5f);
    /*
    int offset = 2;
    state.q[0 + offset] = 2 * ang;
    state.q[3 + offset] = 2 * ang;
    state.q[6 + offset] = 2 * ang;
    state.q[9 + offset] = 2 * ang;*/

    //state = robotModel.StepDynamicsModel(state);
    /*for (int i = 0; i < 13; i++) {
        MathTypes::Vec3 R = RotationMatrixToEuler(SpatialToRotMat(robotModel.dynamics.Xb[i]));
        MathTypes::Vec3 P = SpatialToTranslation(robotModel.dynamics.Xb[i]);

        myRobot->SetRobotLinkPose(P, R, i);
    }*/
    
    /*state.q[0] = ang;
    state.q[1] = ang;
    state.q[2] = ang;*/
    //state.q[1] = ang;
    
    state = chainModel.StepDynamicsModel(state);
    for (int i = 0; i < myChain->chainParameters.numLinks; i++) {
        MathTypes::Vec3 R = RotationMatrixToEuler(SpatialToRotMat(chainModel.dynamics.Xb[i]));
        MathTypes::Vec3 P = SpatialToTranslation(chainModel.dynamics.Xb[i]);

        if (i == myChain->chainParameters.numLinks - 1)
        {
            int j = i + 1;
        }
    
        myChain->SetRobotLinkPose(P, R, i);

        //MathTypes::Mat4 T = SpatialToHomog(chainModel.dynamics.Xb[i]);
        //myChain->SetRobotLinkPose(T, i);
    }

    /*if (ang > 1) {
        MathTypes::Vec6 f;
        f << 0, 0, 0, 0, 0, 0;
        chainModel.SetExternalForceAt(3, f);
    }*/
}

void QuadrupedVisualizer::draw() {
    //mCam.lookAt(vec3(4.0 * sin(ang), 4.0 * cos(ang), 4.0), vec3(0, 0, 0), vec3(0, 0, 1));
    mCam.lookAt(vec3(3.0f, -3.0, 1.0), vec3(0, 0, 0), vec3(0, 0, 1));

    gl::clear();
    gl::setMatrices(mCam);


    plane->Draw();
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