#include "flowSim/include/fluidParticle.h"
#include <iostream>

fluidParticle::fluidParticle(int posX, int posY, int id)
{
    this->_position = Eigen::Vector2d (posX, posY);
    this->_id = id;
}
fluidParticle::fluidParticle(Eigen::Vector2d pos)
{
    this->_position = pos;
}

void fluidParticle::incrementLifeTime(double dt)
{
    this->_dLifeTime += dt;
}

///Updates particle position based on applied velocity field
void fluidParticle::move(double t, std::function<Eigen::Vector2d(Eigen::Vector2d)> velocityFieldFunction)
{
    this->incrementLifeTime(t);
    this-> _velocity = velocityFieldFunction(this->_position);
    
    fluidParticle::basicParticle::move(t);

}

//resets lifetime to random number between -5 and 0. (Particles die when lifetime == CONST_LIFETIME <-- define in flowContainer.h)
void fluidParticle::resetLifeTime()
{
    this->_dLifeTime = - 5 + std::rand() % 5;
}

