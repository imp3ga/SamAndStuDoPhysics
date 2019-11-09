#include "include/basicParticle.h"
#include <iostream>



void basicParticle::setPosition(Eigen::Vector2d pos)
{
    this->_position = pos;
}

void basicParticle::setPosition(int x, int y)
{
    this->_position = Eigen::Vector2d(x, y);
}

void basicParticle::setVelocity(Eigen::Vector2d vel)
{
    this->_velocity = vel;
}

void basicParticle::move(double t)
{ 
    this->_position += this->_velocity * t;
}