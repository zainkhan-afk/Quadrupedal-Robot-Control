#include "LegController.h"

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
LegController<T>::LegController(T _l1, T _l2, T _l3, T _l4)
{
	this->l1 = _l1;
	this->l2 = _l2;
	this->l3 = _l3;
	this->l4 = _l4;
}

template<typename T>
Vec3<T> LegController<T>::ForwardKinematics(int leg)
{

}

template<typename T>
Vec3<T> LegController<T>::InverseKinematics(Vec3<T> pos, int leg)
{
    T l14 = l1 + l4;

    T sideSign = quad.getSideSign(leg);

    T D = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2] - l14 * l14 -
        l2 * l2 - l3 * l3) /
        (2 * l2 * l3);

    if (D > 1)
    {
        D = 1;
    }

    else if (D < -1)
    {
        D = -1;
    }

    T gamma = atan2(-sqrt(1 - D * D), D);
    T theta = -atan2(pos[2], pos[1]) - atan2(sqrt(pos[1] * pos[1] + pos[2] * pos[2] - l1 * l1), sideSign * l1);
    T alpha = atan2(-pos[0], sqrt(pos[1] * pos[1] + pos[2] * pos[2] - l14 * l14)) - atan2(l3 * sin(gamma), l2 + l3 * cos(gamma));

    Vec3<T> q;

    q[0] = -theta;
    q[1] = alpha;
    q[2] = gamma;

    return q;
}

template<typename T>
Mat3<T> LegController<T>::GetLegJacobian(int leg)
{

}