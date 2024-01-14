#include "SpatialInertia.h"


template<typename T>
SpatialInertia<T>::SpatialInertia()
{
    this->intertia = MatSp::Zero();
}

template<typename T>
SpatialInertia<T>::SpatialInertia(T mass, const Vec3<T>& com, const Mat3<T>& inertiaMatrix) 
{
    Mat3<T> cSkew = vectorToSkewMat(com);
    inertia.template topLeftCorner<3, 3>() = inertiaMatrix + mass * cSkew * cSkew.transpose();
    inertia.template topRightCorner<3, 3>() = mass * cSkew;
    inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    inertia.template bottomRightCorner<3, 3>() = mass * Mat3<T>::Identity();
}


template<typename T>
void SpatialInertia<T>::AddInertia(MatSp<T> otherInertia)
{
    this->intertia += otherInertia;
}

template<typename T>
void SpatialInertia<T>::SetInertia(MatSp<T> otherInertia)
{
    this->intertia = otherInertia;
}


template<typename T>
MatSp<T> SpatialInertia<T>::GetInertia()
{
    return this->intertia;
}