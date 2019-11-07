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
    int nCollisions = _vecCollisions.size();
    for (int i = 0; i < nCollisions; ++i)
    {
        std::cout << "Collision!" << std::endl;
        planet *pPlanet1 = static_cast<planet*>(_vecCollisions[i]);
        if (nullptr == pPlanet1)
        {
            std::cout << "ERROR! pPlanet1 is nullptr!" << std::endl;
        }
        if (doesNotInteractWith(pPlanet1->getId()))
        {
            std::cout << "planet id " << getId() << " and " << pPlanet1->getId() << " do not interact." << std::endl;
        }

        std::cout << getId() << " <= " << pPlanet1->getId() << std::endl;
        std::cout << "force before: " << _force.transpose() << std::endl;
        
        Eigen::Vector2d displacementUnitVec = -(pPlanet1->getPosition() - getPosition()).normalized();
        Eigen::Vector2d momentumP1 = pPlanet1->getMass() * pPlanet1->getVelocity();
        std::cout << "p1: \n velocity: " << pPlanet1->getVelocity().transpose() << "\n mass: " << pPlanet1->getMass() << 
         "\nmomentum: " << momentumP1.transpose() << std::endl;
        Eigen::Vector2d impulse = momentumP1.normalized().dot(displacementUnitVec) * momentumP1.norm() * displacementUnitVec / 0.001;
        addForce(impulse);  // 1ms
        pPlanet1->addForce(impulse);
        std::cout << "force after: " << _force.transpose() << std::endl;
        // Remove collisions
        removeCollision(pPlanet1);
        // pPlanet1->removeCollision(this);
    }
    // std::cout << "Calculating PLANET collisions" << std::endl;
    return false;
}