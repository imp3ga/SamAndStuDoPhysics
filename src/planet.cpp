#include "include/planet.h"
#include <iostream>

planet::planet(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
               double dRestCoef, double dMassDensity, std::vector<AstroObjectBase*> &refVecObjects)  : 
        AstroObjectBase(nId, dMass, position, velocity, dRestCoef, dMassDensity, refVecObjects)
{
    _dRadius = sqrt(dMass / dMassDensity);
}

double planet::getRadius()
{
    return _dRadius;
}

bool planet::calculateForceCollisions()
{
    std::cout << "Calculating PLANET collisions" << std::endl;
    return false;
}