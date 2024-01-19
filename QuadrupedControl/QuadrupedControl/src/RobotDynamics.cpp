#include "RobotDynamics.h"

template<typename T>
RobotDynamics<T>::RobotDynamics()
{
	Mat6<T> eye6 = Mat6<T>::Identity();
	VecSP<T> zero6 = VecSP<T>::Zero();
	SpatialInertia<T> zeroInertia;
	for (int i = 0; i < 13; i++)
	{
		compositeInertia.push_back(zeroInertia)
	}
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