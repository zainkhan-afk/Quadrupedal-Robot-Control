#include "pch.h"

#include "Quadruped/SpatialInertia.h"
#include "Quadruped/Utilities.h"


SpatialInertia::SpatialInertia()
{
    this->inertia = dtypes::MatSp::Zero();
}


SpatialInertia::SpatialInertia(float mass, const dtypes::Vec3& com, const dtypes::Mat3& inertiaMatrix)
{
    dtypes::Mat3 cSkew = VectorToSkewMat(com);
    this->inertia.template topLeftCorner<3, 3>() = inertiaMatrix + mass * cSkew * cSkew.transpose();
    this->inertia.template topRightCorner<3, 3>() = mass * cSkew;
    this->inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    this->inertia.template bottomRightCorner<3, 3>() = mass * dtypes::Mat3::Identity();
}


SpatialInertia::SpatialInertia(const dtypes::Mat4& P) {
    dtypes::Mat6 I;
    float m = P(3, 3);
    dtypes::Vec3 h = P.template topRightCorner<3, 1>();
    dtypes::Mat3 E = P.template topLeftCorner<3, 3>();
    
    dtypes::Mat3 Ibar = E.trace() * dtypes::Mat3::Identity() - E;
    
    I.template topLeftCorner<3, 3>() = Ibar;
    I.template topRightCorner<3, 3>() = VectorToSkewMat(h);
    I.template bottomLeftCorner<3, 3>() = VectorToSkewMat(h).transpose();
    I.template bottomRightCorner<3, 3>() = m * dtypes::Mat3::Identity();
    
    this->inertia = I;
}



SpatialInertia::~SpatialInertia()
{

}


dtypes::Mat4 SpatialInertia::GetPseudoInertia() {
    dtypes::Vec3 h = MatToSkewVec(this->inertia.template topRightCorner<3, 3>());
    dtypes::Mat3 Ibar = this->inertia.template topLeftCorner<3, 3>();
    float m = this->inertia(5, 5);
    
    dtypes::Mat4 P;
    P.template topLeftCorner<3, 3>() = 0.5 * Ibar.trace() * dtypes::Mat3::Identity() - Ibar;
    P.template topRightCorner<3, 1>() = h;
    P.template bottomLeftCorner<1, 3>() = h.transpose();
    
    P(3, 3) = m;
    
    return P;
}


SpatialInertia SpatialInertia::FlipAlongAxis(int axis)
{
    dtypes::Mat4 P = GetPseudoInertia();
    dtypes::Mat4 X = dtypes::Mat4::Identity();
    if (axis == 0)
        X(0, 0) = -1;
    
    else if (axis == 1)
        X(1, 1) = -1;
    
    else if (axis == 2)
        X(2, 2) = -1;
    
    P = X * P * X;
    
    return SpatialInertia(P);
}




void SpatialInertia::AddInertia(dtypes::MatSp otherInertia)
{
    this->inertia += otherInertia;
}


void SpatialInertia::SetInertia(dtypes::MatSp otherInertia)
{
    this->inertia = otherInertia;
}


dtypes::MatSp SpatialInertia::GetInertia()
{
    return this->inertia;
}