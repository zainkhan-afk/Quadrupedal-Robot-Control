#include "pch.h"
#include<iostream>
#include "Quadruped/LegController.h"
#include "Quadruped/Utilities.h"


LegController::LegController()
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

LegController::LegController(float abdLinkLength, float hipLinkLength, float kneeLinkLength, float kneeLinkYOffset)
{
	this->l1 = abdLinkLength;
	this->l2 = hipLinkLength;
	this->l3 = kneeLinkLength;
	this->l4 = kneeLinkYOffset;

    this->l14 = this->l1 + this->l4;
}


dtypes::Vec3 LegController::ForwardKinematics(dtypes::Vec3 q, int leg)
{
    // X : -1.0 * c2 * l3 * s3 - 1.0 * c3 * l3 * s2 - 1.0 * l2 * s2
    // Y : 1.0 * c1 * l1 + s1 * (c2 * c3 * l3 + c2 * l2 - 1.0 * l3 * s2 * s3)
    // Z : -1.0 * c1 * (c2 * c3 * l3 + c2 * l2 - 1.0 * l3 * s2 * s3) + 1.0 * l1 * s1

    dtypes::Vec3 P = dtypes::Vec3::Zero();

    //int sideSign = GetLegSign(leg);

    float s1 = std::sin(q[0]);
    float s2 = std::sin(q[1]);
    float s3 = std::sin(q[2]);

    float c1 = std::cos(q[0]);
    float c2 = std::cos(q[1]);
    float c3 = std::cos(q[2]);

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    P[0] = -this->l3 * c2 * s3 - this->l3 * s2 * c3 - this->l2 * s2;
    P[1]  = this->l14 * c1 + s1 * (this->l3 * c2 * c3 + this->l2 * c2 - this->l3 * s2 * s3);
    P[2] =  this->l14 * s1 - c1 * (this->l3 * c2 * c3 + this->l2 * c2 - this->l3 * s2 * s3);

    std::cout << "Leg: " << leg << " Position: " << P[0] << ", " << P[1] << ", " << P[2] << "\n";


    return P;
}


dtypes::Vec3 LegController::InverseKinematics(dtypes::Vec3 pos, int leg)
{
    //int sideSign = GetLegSign(leg);
    dtypes::Vec3 q = dtypes::Vec3::Zero();
    float R = sqrt(pos[2] * pos[2] + pos[1] * pos[1]);

    float beta = acos(pos[1] / R);
    float alpha = acos(this->l14 / R);

    float theta1 = alpha - beta;

    float x_ = -pos[2];
    float y_ = -pos[1] + this->l14;
    float z_ = -pos[0];


    float temp = (x_ * x_ + z_ * z_ - this->l2 * this->l2 - this->l3 * this->l3) / (2 * this->l2 * this->l3);

    if (temp > 1)
        temp = 1;
    else if (temp < -1)
        temp = -1;

    float theta3 = acos(temp);
    float theta2 = (atan2(z_, x_) - atan2(this->l3 * sin(theta3), (this->l2 + this->l3 * cos(theta3))));

    q[0] = theta1;
    q[1] = theta2;
    q[2] = theta3;

    return q;
}

//
//Vec3 LegController::InverseKinematics(Vec3 pos, int leg)
//{
//    int sideSign = GetLegSign(leg);
//    pos[1] *= -1;
//    Vec3 q = Vec3::Zero();
//    float l14 = l1 + l4;
//
//    float R = sqrt(pos[2] * pos[2] + pos[1] * pos[1]);
//
//    float alpha = acos(abs(pos[1]) / R);
//    float beta = acos(l14 / R);
//
//    if (pos[1] >= 0) { q[0] = alpha - beta; }
//    else { q[0] = PI - alpha - beta; }
//
//    float _x = pos[0];
//    float _z = -sqrt(pos[1] * pos[1] + pos[2] * pos[2] + l14 * l14);
//
//
//    float D = (_x*_x + _z*_z - l2 * l2 - l3 * l3) / (2 * l2 * l3);
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


dtypes::Mat3 LegController::GetLegJacobian(dtypes::Vec3 q, int leg)
{
    dtypes::Mat3 J = dtypes::Mat3::Zero();
    /*int sideSign = GetLegSign(leg);

    float s1 = std::sin(q[0]);
    float s2 = std::sin(q[1]);
    float s3 = std::sin(q[1]);

    float c1 = std::cos(q[0]);
    float c2 = std::cos(q[1]);
    float c3 = std::cos(q[2]);

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;
        
    J(0, 0) = 0;
    J(0, 1) = l3 * c23 + l2 * c2;
    J(0, 2) = l3 * c23;
    J(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
    J(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J(1, 2) = -l3 * s1 * s23;
    J(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
    J(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J(2, 2) = l3 * c1 * s23;*/

    return J;
}