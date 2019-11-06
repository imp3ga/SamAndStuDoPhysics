#include "include/planet.h"

planet::planet(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
               double dRestCoef, double dMassDensity, std::vector<AstroObjectBase> &refVecObjects,
               bool bInteracts)  : 
        AstroObjectBase(nId, dMass, position, velocity, dRestCoef, dMassDensity, refVecObjects, bInteracts)
{
    _dRadius = sqrt(dMass / dMassDensity);
}

double planet::getRadius()
{
    return _dRadius;
}