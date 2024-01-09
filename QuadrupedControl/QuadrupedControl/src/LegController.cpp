#include "LegController.h"
#include "Utilities.h"



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
}

template<typename T>
Vec3<T> LegController<T>::ForwardKinematics(Vec3<T> q, int leg)
{
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

    P[0] = l3 * s23 + l2 * s2;
    P[1]  = (l1 + l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    P[3] = (l1 + l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;


    return P;
}

template<typename T>
Vec3<T> LegController<T>::InverseKinematics(Vec3<T> pos, int leg)
{
    T l14 = l1 + l4;

    int sideSign = GetLegSign(leg);

    T D = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2] - l14 * l14 - l2 * l2 - l3 * l3) / (2 * l2 * l3);

    if (D > 1)
    {
        D = 1;
    }

    else if (D < -1)
    {
        D = -1;
    }

    T t1 = atan2(-sqrt(1 - D * D), D);
    T t2 = -atan2(pos[2], pos[1]) - atan2(sqrt(pos[1] * pos[1] + pos[2] * pos[2] - l1 * l1), sideSign * l1);
    T t3 = atan2(-pos[0], sqrt(pos[1] * pos[1] + pos[2] * pos[2] - l14 * l14)) - atan2(l3 * sin(t1), l2 + l3 * cos(t1));

    Vec3<T> q = Vec3<T>::Zero();

    q[0] = -t1;
    q[1] = t2;
    q[2] = t3;

    return q;
}

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