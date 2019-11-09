
#include "include/velocityField.h"
#include <iostream>

// Eigen::Vector2d velocityField::velocitySum(Eigen::Vector2d& position, std::function<Eigen::Vector2d(Eigen::Vector2d)> func)
// {
// 	return func(position); //can now return multiple things, fuck yeah
// }

int velocityField::_areaWidth;
int velocityField::_areaHeight;
double velocityField::_dFlowStrength = 200;
double velocityField::_dRadius = 100;
double velocityField::_dCylinderrotation = 120000;
double velocityField::_dRankinevortexrotation = 10;
std::out_of_range velocityField::_outOfRangeError = std::out_of_range("Field dimensions not set! Use velocityField::setFieldDimensions(int w, int h)");

// velocityField::velocityField(int w, int h)
// {
//     this->_areaWidth = w;
//     this->_areaHeight = h;
//     this->_dFlowStrength = 200;
//     this->_dRadius = 100;
//     this->_dRankinevortexrotation = -100000;
//     this->_dCylinderrotation = 10;
// }
// velocityField::velocityField(int w, int h, double flow)
// {
//     this->_areaWidth = w;
//     this->_areaHeight = h;
//     this->_dFlowStrength = flow;
//     this->_dRadius = 100;
//     this->_dRankinevortexrotation = -100000;
//     this->_dCylinderrotation = 10;
// }
// velocityField::velocityField(int w, int h, double flow, double radius, double rankineRotation, double cylinderRotation)
// {
//     this->_areaWidth = w;
//     this->_areaHeight = h;
//     this->_dFlowStrength = flow;
//     this->_dRadius = radius;
//     this->_dRankinevortexrotation = rankineRotation;
//     this->_dCylinderrotation = cylinderRotation;
// }
void velocityField::setFieldDimensions(int w, int h)
{
    _areaWidth = w;
    _areaHeight = h;
}

void velocityField::resetScalingParameters(){
    velocityField::_dFlowStrength = 200;
    velocityField::_dRadius = 100;
    velocityField::_dCylinderrotation = 120000;
    velocityField::_dRankinevortexrotation = 10;
     std::cout << "System Reset" << "\n";
}

void velocityField::setFlowStrength(double strength, int higherLower)
{
    if(higherLower == higher){
        _dFlowStrength += strength;
    }
    else{
        _dFlowStrength -= strength;
    }
    std::cout << "Flow Rate set: " << _dFlowStrength << "\n";
}
void velocityField::setRadius(double radius, int higherLower)
{
     if(higherLower == higher){
        _dRadius += radius;
    }
    else{
        _dRadius -= radius;
    }
    std::cout << "Central Radius set: " << _dRadius << "\n";
}
void velocityField::setCylinderRotation(double rotation, int higherLower)
{
     if(higherLower == higher){
        _dCylinderrotation += rotation;
    }
    else{
        _dCylinderrotation -= rotation;
    }

    std::cout << "Cylinder Rotation set: " << _dCylinderrotation << "\n";
}
void velocityField::setRankineVortexRotation(double rotation, int higherLower)
{
     if(higherLower == higher){
        _dRankinevortexrotation += rotation;
    }
    else{
        _dRankinevortexrotation -= rotation;
    }
    std::cout << "Rankine Vortex Rotation set: " << _dRankinevortexrotation << "\n";
}

Eigen::Vector2d velocityField::noFlow(const Eigen::Vector2d& position)
{    
	return Eigen::Vector2d(0, 0);
}

Eigen::Vector2d velocityField::uniformFlow(const Eigen::Vector2d& position)
{
	return(Eigen::Vector2d(_dFlowStrength, 0));
}

Eigen::Vector2d velocityField::shearFlow(const Eigen::Vector2d& position)
{
	return Eigen::Vector2d(-0.01 * _dFlowStrength * (position[1]-_areaHeight), 0);
}

Eigen::Vector2d velocityField::stagnationPointFlow(const Eigen::Vector2d& position)
{
	return Eigen::Vector2d(0.01 * _dFlowStrength * (position[0]), -0.01*_dFlowStrength*(position[1]));
}

Eigen::Vector2d velocityField::sphereFlow(const Eigen::Vector2d& position)
{
	return Eigen::Vector2d(0.5*_dFlowStrength*(((powf(_dRadius, 3)) / (powf((powf(position[0], 2) + powf(position[1], 2)), 1.5))) + 2) - (3 * powf(_dRadius, 3)*_dFlowStrength*powf(position[0], 2)) / (2 * powf((powf(position[0], 2) + powf(position[1], 2)), 2.5)), -(3 * powf(_dRadius, 3)*_dFlowStrength*(position[0])*(position[1])) / (2 * powf((powf(position[0], 2) + powf(position[1], 2)), 2.5)));
}

Eigen::Vector2d velocityField::vortex(const Eigen::Vector2d& position)
{
	float angle = atan2f(position[0], position[1]);
	return Eigen::Vector2d((-_dFlowStrength*(position[0])) / (sqrtf(powf(position[0], 2) + powf(position[1], 2))) - (_dFlowStrength*(position[1])) / (sqrtf(powf(position[0], 2) + powf(position[1], 2))), (_dFlowStrength*(position[0])) / (sqrtf(powf(position[0], 2) + powf(position[1], 2))) - (_dFlowStrength*(position[1])) / (sqrtf(powf(position[0], 2) + powf(position[1], 2))));
}

Eigen::Vector2d velocityField::bathplugVortex(const Eigen::Vector2d& position)
{
	if (sqrtf(powf(position[0], 2) + powf(position[1], 2)) <= _dRadius)
	{
		return Eigen::Vector2d((-_dFlowStrength*(position[0])) / (sqrtf(powf(position[0], 2) + powf(position[1], 2))) - (_dRankinevortexrotation*(position[1])) / 2, ((_dRankinevortexrotation*(position[0])) / 2) - (_dFlowStrength*(position[1])) / (sqrtf(powf(position[0], 2) + powf(position[1], 2))));
		//return Eigen::Vector2d(0, 0);
	}
	else
		return Eigen::Vector2d((-powf(_dRadius, 2) * _dRankinevortexrotation*(position[1])) / (2 * (powf(position[0], 2) + powf(position[1], 2))) - (_dFlowStrength*(position[0])) / (sqrtf(powf(position[0], 2) + powf(position[1], 2))), (powf(_dRadius, 2) * _dRankinevortexrotation*(position[0])) / (2 * (powf(position[0], 2) + powf(position[1], 2))) - (_dFlowStrength*(position[1])) / (sqrtf(powf(position[0], 2) + powf(position[1], 2))));
}

Eigen::Vector2d velocityField::liftonCylinder(const Eigen::Vector2d& position)
{
	float x = position[0];
	float y = position[1];
	float xysquare = powf(x, 2) + powf(y, 2);

	return Eigen::Vector2d((2 * 3.14*powf(_dRadius, 2)*_dFlowStrength*(y*y - x*x) + xysquare*(2 * 3.14*_dFlowStrength*xysquare - _dCylinderrotation*y)) / (2 * 3.14*powf(xysquare, 2)), (x*(_dCylinderrotation*xysquare - 4 * 3.14*powf(_dRadius, 2)*_dFlowStrength*y)) / (2 * 3.14*powf(xysquare, 2)));
}