#include "pch.h"

#include "Quadruped/SpatialInertia.h"
#include "Quadruped/Utilities.h"


SpatialInertia::SpatialInertia()
{
    this->inertia = MathTypes::Mat6::Zero();
}


SpatialInertia::SpatialInertia(float mass, const MathTypes::Vec3& com, const MathTypes::Mat3& inertiaMatrix)
{
    MathTypes::Mat3 cSkew = VectorToSkewMat(com);
    this->inertia.template topLeftCorner<3, 3>() = inertiaMatrix + mass * cSkew * cSkew.transpose();
    this->inertia.template topRightCorner<3, 3>() = mass * cSkew;
    this->inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    this->inertia.template bottomRightCorner<3, 3>() = mass * MathTypes::Mat3::Identity();
}


SpatialInertia::SpatialInertia(const MathTypes::Mat4& P) {
    MathTypes::Mat6 I;
    float m = P(3, 3);
    MathTypes::Vec3 h = P.template topRightCorner<3, 1>();
    MathTypes::Mat3 E = P.template topLeftCorner<3, 3>();
    
    MathTypes::Mat3 Ibar = E.trace() * MathTypes::Mat3::Identity() - E;
    
    I.template topLeftCorner<3, 3>() = Ibar;
    I.template topRightCorner<3, 3>() = VectorToSkewMat(h);
    I.template bottomLeftCorner<3, 3>() = VectorToSkewMat(h).transpose();
    I.template bottomRightCorner<3, 3>() = m * MathTypes::Mat3::Identity();
    
    this->inertia = I;
}



SpatialInertia::~SpatialInertia()
{

}


MathTypes::Mat4 SpatialInertia::GetPseudoInertia() {
    MathTypes::Vec3 h = MatToSkewVec(this->inertia.template topRightCorner<3, 3>());
    MathTypes::Mat3 Ibar = this->inertia.template topLeftCorner<3, 3>();
    float m = this->inertia(5, 5);
    
    MathTypes::Mat4 P;
    P.template topLeftCorner<3, 3>() = 0.5 * Ibar.trace() * MathTypes::Mat3::Identity() - Ibar;
    P.template topRightCorner<3, 1>() = h;
    P.template bottomLeftCorner<1, 3>() = h.transpose();
    
    P(3, 3) = m;
    
    return P;
}


SpatialInertia SpatialInertia::FlipAlongAxis(int axis)
{
    MathTypes::Mat4 P = GetPseudoInertia();
    MathTypes::Mat4 X = MathTypes::Mat4::Identity();
    if (axis == 0)
        X(0, 0) = -1;
    
    else if (axis == 1)
        X(1, 1) = -1;
    
    else if (axis == 2)
        X(2, 2) = -1;
    
    P = X * P * X;
    
    return SpatialInertia(P);
}




void SpatialInertia::AddInertia(MathTypes::Mat6 otherInertia)
{
    this->inertia += otherInertia;
}


void SpatialInertia::SetInertia(MathTypes::Mat6 otherInertia)
{
    this->inertia = otherInertia;
}


MathTypes::Mat6 SpatialInertia::GetInertia()
{
    return this->inertia;
}