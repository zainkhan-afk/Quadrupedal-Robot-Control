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

    CI_LOG_D("Initializing robot.");

    State st;

    st.bodyPose << 0, 0, 0, 0, 0, 0.5;

    Quadruped* robot = new Quadruped();
    robot->Initialize();
    robot->SetState(st);
    robot->dynamics.G << 0, 0, 0, 0, 0, -5;

    simEnviron.AddRobot(robot);

    PlaneCollider* pc = new PlaneCollider();
    simEnviron.AddPlane(pc);
}

void QuadrupedVisualizer::resize()
{
    mCam.setPerspective(45, getWindowAspectRatio(), 1, 1000);
    gl::setMatrices(mCam);
}

void QuadrupedVisualizer::update() {
    simEnviron.Step(dt);
    std::vector<MathTypes::Mat4> transformationChain = simEnviron.GetRobotTransformationChain();
    for (int i = 0; i < 13; i++) {

        //MathTypes::Vec3 R = RotationMatrixToEuler(robotModel.dynamics.Xb[i + 1].GetInverse().GetRotation());
        //MathTypes::Vec3 P = robotModel.dynamics.Xb[i + 1].GetInverse().GetTranslation();

        MathTypes::Vec3 R;
        MathTypes::Vec3 P;
        R = RotationMatrixToEuler(simEnviron.robot->transformationChain[i].template topLeftCorner<3, 3>());
        P[0] = simEnviron.robot->transformationChain[i](0, 3);
        P[1] = simEnviron.robot->transformationChain[i](1, 3);
        P[2] = simEnviron.robot->transformationChain[i](2, 3);

        myRobot->SetRobotLinkPose(P, R, i);
    }

    for (int i = 0; i < 4; i++) {
        //myRobot->SetRobotFootPosition(robotModel.dynamics.Xcb[i].GetInverse().GetTranslation(), i);
        myRobot->SetRobotFootPosition(simEnviron.robot->footPos[i], i);
    }

    MathTypes::Vec6 f;
    f << 0, 0, 0, 0, 0, 0;
    for (int i = 0; i < 13; i++)
    {
        simEnviron.robot->SetExternalForceAt(i, f);
    }
}

void QuadrupedVisualizer::draw() {
    //mCam.lookAt(vec3(4.0 * sin(ang), 4.0 * cos(ang), 1.0), vec3(0, 0, 0), vec3(0, 0, 1));
    //mCam.lookAt(vec3(state.bodyPose[3] + 3.0f, state.bodyPose[4] - 3.0, state.bodyPose[5] + 1.0), vec3(state.bodyPose[3], state.bodyPose[4], state.bodyPose[5]), vec3(0, 0, 1));
    mCam.lookAt(vec3(3.0f, - 3.0, 1.0), vec3(0), vec3(0, 0, 1));

    gl::clear();
    gl::setMatrices(mCam);

    sceneAxes->Draw();
    plane->Draw();
    myRobot->Draw();
    
    ang += 0.01;
}

void QuadrupedVisualizer::cleanup() {
    delete plane;
    delete myRobot;
    delete sceneAxes;
}

CINDER_APP(QuadrupedVisualizer, RendererGl(RendererGl::Options().msaa(16)))