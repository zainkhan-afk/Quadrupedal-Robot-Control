#include "pch.h"
#include "Quadruped/TestClass.h"
#include <iostream>


TestClass::TestClass()
{
	std::cout << "Test Class Constructor" << std::endl;
}

TestClass::~TestClass()
{
	std::cout << "Test Class Destructor" << std::endl;
}