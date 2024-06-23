#include "Simulation/Visualization/QuadrupedVisualizer.h"
#include "Simulation/Visualization/Resources.h"
#include "cinder/Log.h"
#include "Simulation/Dynamics/Spatial.h"

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

    plane = new myprimitives::Plane(mGlsl);
    myRobot = new Robot(mGlsl);
    sceneAxes = new Axes(mGlsl);

    linkIdx = 0;
    footIdx = 0;
    InitializeRobot();
    CI_LOG_D("THIS IS A DEBUG LOG");
    //MathTypes::Vec6 f;
    //f << 0, 0, 0, 0, 0, 5;
    //robotModel.SetExternalForceAt(linkIdx, f);
    //robotModel.SetExternalForceAt(linkIdx, f);
    //robotModel.dynamics.fc[footIdx] << 0, 0, 0, 0.1, 0, 0;
    robotModel.dynamics.G[5] = -0.1;

    //robotModel.dynamics.fc[footIdx] << 1, 0, 0;


    //state.q[0] = M_PI / 10.0f;
    //state.q[1] = M_PI / 10.0f;
}

void QuadrupedVisualizer::resize()
{
    mCam.setPerspective(45, getWindowAspectRatio(), 1, 1000);
    gl::setMatrices(mCam);
}

void QuadrupedVisualizer::update() {
    //state.q[2] = 0;


    MathTypes::Vec6 f;
    //f << 0, 0, 0, 0, 0, 10;

    //robotModel.dynamics.fc[footIdx] = (robotModel.footPos[footIdx] - MathTypes::Vec3(robotModel.footPos[footIdx][0], robotModel.footPos[footIdx][1], -0.1));

    MathTypes::Vec3 contactF; 
    contactF << 0, 0, 0.3;
    
    robotModel.dynamics.fc[0] = contactF;
    robotModel.dynamics.fc[1] = contactF;
    robotModel.dynamics.fc[2] = contactF;
    robotModel.dynamics.fc[3] = contactF;
    
    state = robotModel.StepDynamicsModel(state);
    robotModel.GetVisualTransformations(state);

    for (int i = 0; i < 13; i++) {

        //MathTypes::Vec3 R = RotationMatrixToEuler(robotModel.dynamics.Xb[i + 1].GetInverse().GetRotation());
        //MathTypes::Vec3 P = robotModel.dynamics.Xb[i + 1].GetInverse().GetTranslation();

        MathTypes::Vec3 R;
        MathTypes::Vec3 P;
        R = RotationMatrixToEuler(robotModel.transformationChain[i].template topLeftCorner<3, 3>());
        P[0] = robotModel.transformationChain[i](0, 3);
        P[1] = robotModel.transformationChain[i](1, 3);
        P[2] = robotModel.transformationChain[i](2, 3);

        myRobot->SetRobotLinkPose(P, R, i);
    }

    for (int i = 0; i < 4; i++) {
        //myRobot->SetRobotFootPosition(robotModel.dynamics.Xcb[i].GetInverse().GetTranslation(), i);
        myRobot->SetRobotFootPosition(robotModel.footPos[i], i);
    }


    f << 0, 0, 0, 0, 0, 0;
    for (int i = 0; i < 13; i++)
    {
        robotModel.SetExternalForceAt(i, f);
    }
}

void QuadrupedVisualizer::draw() {
    //mCam.lookAt(vec3(4.0 * sin(ang), 4.0 * cos(ang), 1.0), vec3(0, 0, 0), vec3(0, 0, 1));
    //mCam.lookAt(vec3(state.bodyPose[3] + 3.0f, state.bodyPose[4] - 3.0, state.bodyPose[5] + 1.0), vec3(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]), vec3(0, 0, 1));
    mCam.lookAt(vec3(3.0f, - 3.0, 1.0), vec3(0), vec3(0, 0, 1));

    gl::clear();
    gl::setMatrices(mCam);

    sceneAxes->Draw();
    //plane->Draw();
    myRobot->Draw();
    
    ang += 0.01;
}


void QuadrupedVisualizer::InitializeRobot()
{
    robotModel.Initialize();

    state.bodyPose << 0, 0, 0, 0, 0, 0.5;

    robotModel.SetState(state);
}

void QuadrupedVisualizer::cleanup() {
    delete plane;
    delete myRobot;
    delete sceneAxes;
}

CINDER_APP(QuadrupedVisualizer, RendererGl(RendererGl::Options().msaa(16)))