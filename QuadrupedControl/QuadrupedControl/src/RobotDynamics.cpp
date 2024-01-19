#include "RobotDynamics.h"
#include <iostream>

template<typename T>
RobotDynamics<T>::RobotDynamics()
{

}

template<typename T>
RobotDynamics<T>::~RobotDynamics()
{

}

template<typename T>
void RobotDynamics<T>::AddBody(MatSp<T> I, Mat6<T> pos)
{
	this->parameters.robotInertias.push_back(I);
	this->parameters.linkPositions.push_back(pos);
}