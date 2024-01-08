#include "LegController.h"
#include "Utilities.h"

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

    T sideSign = GetLegSign(leg);

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

    Vec3<T> q;

    q[0] = -t1;
    q[1] = t2;
    q[2] = t3;

    return q;
}

template<typename T>
Mat3<T> LegController<T>::GetLegJacobian(int leg)
{

}