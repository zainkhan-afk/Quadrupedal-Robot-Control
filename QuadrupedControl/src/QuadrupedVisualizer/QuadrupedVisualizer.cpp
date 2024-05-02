#include "QuadrupedVisualizer/QuadrupedVisualizer.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void QuadrupedVisualizer::setup() {
    gl::enableDepthWrite();
    gl::enableDepthRead();

    // Create cube batch
    mCubeBatch = gl::Batch::create(geom::Cube().size(vec3(1, 1, 1)), gl::getStockShader(gl::ShaderDef().color()));
}

void QuadrupedVisualizer::resize()
{
    mCam.setPerspective(60, getWindowAspectRatio(), 1, 1000);
    mCam.lookAt(vec3(7, 7, 5), vec3(0), vec3(0, 0, 1));
    gl::setMatrices(mCam);
}

void QuadrupedVisualizer::update() {
    // Update logic if needed
}

void QuadrupedVisualizer::draw() {
    gl::clear();
    gl::setMatrices(mCam);

    //gl::clear(Color(1, 0, 0));
    gl::color(Color(1, 0, 0));
    gl::rotate(ang, vec3(0.5f, 1.0f, 0.0f));
    mCubeBatch->draw();

    ang += 0.01;
}

CINDER_APP(QuadrupedVisualizer, RendererGl(RendererGl::Options().msaa(16)))