#include "include/planet.h"
#include <iostream>

planet::planet(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
               double dRestCoef, double dMassDensity)  : 
        AstroObjectBase(nId, dMass, position, velocity, dRestCoef, dMassDensity)
{
    _dRadius = sqrt(dMass / dMassDensity);
}

double planet::getRadius()
{
    return _dRadius;
}

bool planet::calculateForceCollisions(std::vector<AstroObjectBase>& _vecObjects)
{        
    int nSize = static_cast<int>(_vecObjects.size());
    for (int i = 0; i < nSize; ++i)
    {
        // Object to compare against
        AstroObjectBase *pObj = &_vecObjects[i];            

        if (_nId == pObj->getId() 
        || !collidesWith(pObj) 
        || hasAlreadyInteracted(pObj->getDontInteractIDs()))
        {
            // Don't interact with self;
            // Skip if no collision;
            // Skip if these two have already met.

            continue;
        }
        else
        {
           // Need to add Force exchange logic.
         
            addCollision(pObj);
        };       
    }
    return true;
}