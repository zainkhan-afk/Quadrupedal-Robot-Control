#include "LegController.h"
#include "Utilities.h"
#include<iostream>



template<typename T>
LegController<T>::LegController()
{

}

/*
* The constructor takes the link lengths of the robot and uses that to compute the IK, FK and Jacobian.
* 
* l1 -> abdLinkLength
* l2 -> hipLinkLength
* l3 -> kneeLinkLength
* l4 -> kneeLinkYOffset
*
*/
template<typename T>
LegController<T>::LegController(T abdLinkLength, T hipLinkLength, T kneeLinkLength, T kneeLinkYOffset)
{
	this->l1 = abdLinkLength;
	this->l2 = hipLinkLength;
	this->l3 = kneeLinkLength;
	this->l4 = kneeLinkYOffset;

    this->l14 = this->l1 + this->l4;
}

template<typename T>
Vec3<T> LegController<T>::ForwardKinematics(Vec3<T> q, int leg)
{
    // X : -1.0 * c2 * l3 * s3 - 1.0 * c3 * l3 * s2 - 1.0 * l2 * s2
    // Y : 1.0 * c1 * l1 + s1 * (c2 * c3 * l3 + c2 * l2 - 1.0 * l3 * s2 * s3)
    // Z : -1.0 * c1 * (c2 * c3 * l3 + c2 * l2 - 1.0 * l3 * s2 * s3) + 1.0 * l1 * s1

    Vec3<T> P = Vec3<T>::Zero();

    int sideSign = GetLegSign(leg);

    T s1 = std::sin(q[0]);
    T s2 = std::sin(q[1]);
    T s3 = std::sin(q[2]);

    T c1 = std::cos(q[0]);
    T c2 = std::cos(q[1]);
    T c3 = std::cos(q[2]);

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;

    P[0] = -this->l3 * c2 * s3 - this->l3 * s2 * c3 - this->l2 * s2;
    P[1]  = this->l14 * c1 + s1 * (this->l3 * c2 * c3 + this->l2 * c2 - this->l3 * s2 * s3);
    P[2] =  this->l14 * s1 - c1 * (this->l3 * c2 * c3 + this->l2 * c2 - this->l3 * s2 * s3);

    std::cout << "Leg: " << leg << " Position: " << P[0] << ", " << P[1] << ", " << P[2] << "\n";


    return P;
}

template<typename T>
Vec3<T> LegController<T>::InverseKinematics(Vec3<T> pos, int leg)
{
    int sideSign = GetLegSign(leg);
    Vec3<T> q = Vec3<T>::Zero();
    T R = sqrt(pos[2] * pos[2] + pos[1] * pos[1]);

    T beta = acos(pos[1] / R);
    T alpha = acos(this->l14 / R);

    T theta1 = alpha - beta;

    T x_ = -pos[2];
    T y_ = -pos[1] + this->l14;
    T z_ = -pos[0];


    T temp = (x_ * x_ + z_ * z_ - this->l2 * this->l2 - this->l3 * this->l3) / (2 * this->l2 * this->l3);

    if (temp > 1)
        temp = 1;
    else if (temp < -1)
        temp = -1;

    T theta3 = acos(temp);
    T theta2 = (atan2(z_, x_) - atan2(this->l3 * sin(theta3), (this->l2 + this->l3 * cos(theta3))));

    q[0] = theta1;
    q[1] = theta2;
    q[2] = theta3;

    return q;
}

//template<typename T>
//Vec3<T> LegController<T>::InverseKinematics(Vec3<T> pos, int leg)
//{
//    int sideSign = GetLegSign(leg);
//    pos[1] *= -1;
//    Vec3<T> q = Vec3<T>::Zero();
//    T l14 = l1 + l4;
//
//    T R = sqrt(pos[2] * pos[2] + pos[1] * pos[1]);
//
//    T alpha = acos(abs(pos[1]) / R);
//    T beta = acos(l14 / R);
//
//    if (pos[1] >= 0) { q[0] = alpha - beta; }
//    else { q[0] = PI - alpha - beta; }
//
//    T _x = pos[0];
//    T _z = -sqrt(pos[1] * pos[1] + pos[2] * pos[2] + l14 * l14);
//
//
//    T D = (_x*_x + _z*_z - l2 * l2 - l3 * l3) / (2 * l2 * l3);
//
//    if (D > 1)
//    {
//        D = 1;
//    }
//
//    else if (D < -1)
//    {
//        D = -1;
//    }
//
//    q[2] = acos(D);
//
//    q[1] = PI / 2 + (atan2(_z, _x) - atan2(l3 * sin(q[2]), l2 + l3 * cos(q[2])));
//
//    return q;
//}

template<typename T>
Mat3<T> LegController<T>::GetLegJacobian(Vec3<T> q, int leg)
{
    Mat3<T> J = Mat3<T>::Zero();
    int sideSign = GetLegSign(leg);

    T s1 = std::sin(q[0]);
    T s2 = std::sin(q[1]);
    T s3 = std::sin(q[1]);

    T c1 = std::cos(q[0]);
    T c2 = std::cos(q[1]);
    T c3 = std::cos(q[2]);

    T c23 = c2 * c3 - s2 * s3;
    T s23 = s2 * c3 + c2 * s3;
        
    J(0, 0) = 0;
    J(0, 1) = l3 * c23 + l2 * c2;
    J(0, 2) = l3 * c23;
    J(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
    J(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J(1, 2) = -l3 * s1 * s23;
    J(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
    J(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J(2, 2) = l3 * c1 * s23;

    return J;
}

template class LegController<double>;
template class LegController<float>;