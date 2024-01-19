#include "RobotDynamics.h"

template<typename T>
RobotDynamics<T>::RobotDynamics()
{
}

template<typename T>
RobotDynamics<T>::~RobotDynamics()
{

}

template<typename T>
void RobotDynamics<T>::AddBody(SpatialInertia<T> I, Mat6<T> pos, int parent)
{
	this->robotInertias.push_back(I);
	this->linkPositions.push_back(pos);
	this->parents.push_back(parent);
}

template<typename T>
void RobotDynamics<T>::CalculateCompositeInertia()
{
}

template<typename T>
void RobotDynamics<T>::CalculateLinkTreePositions()
{
}

template class RobotDynamics<double>;
template class RobotDynamics<float>;