#pragma once

#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

class QuadrupedVisualizer : public ci::app::App 
{
public:
    void setup() override;
    void resize() override;
    void update() override;
    void draw() override;

private:
    ci::CameraPersp mCam;
    ci::gl::BatchRef mCubeBatch;

    double ang = 0.0;
};
