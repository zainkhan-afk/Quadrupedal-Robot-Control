#include "SpatialInertia.h"
#include "Utilities.h"

template<typename T>
SpatialInertia<T>::SpatialInertia()
{
    this->inertia = MatSp<T>::Zero();
}

template<typename T>
SpatialInertia<T>::SpatialInertia(T mass, const Vec3<T>& com, const Mat3<T>& inertiaMatrix) 
{
    /*Mat3<T> cSkew = vectorToSkewMat(com);
    this->inertia.template topLeftCorner<3, 3>() = inertiaMatrix + mass * cSkew * cSkew.transpose();
    this->inertia.template topRightCorner<3, 3>() = mass * cSkew;
    this->inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    this->inertia.template bottomRightCorner<3, 3>() = mass * Mat3<T>::Identity();*/
}


template<typename T>
void SpatialInertia<T>::AddInertia(MatSp<T> otherInertia)
{
    this->inertia += otherInertia;
}

template<typename T>
void SpatialInertia<T>::SetInertia(MatSp<T> otherInertia)
{
    this->inertia = otherInertia;
}


template<typename T>
MatSp<T> SpatialInertia<T>::GetInertia()
{
    return this->inertia;
}

template class SpatialInertia<double>;
template class SpatialInertia<float>;