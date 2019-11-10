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
    // Check if reached non-collision state
    for (int i = 0; i < _vecDontCollideIds.size(); ++i)
    {
        int nId = _vecDontCollideIds[i];
        planet *pPlanet1 = static_cast<planet*>(findById(nId));
        if (nullptr == pPlanet1)
        {
            std::cout << "ERROR! Could not find by id!" << std::endl;
        }
        double dDistance = getDistanceBetween(pPlanet1) ;
        if (dDistance > getRadius() + pPlanet1->getRadius())
        {
            removeDoesNotCollideWith(pPlanet1->getId());
        }
    }

    int nCollisions = _vecCollisions.size();
    for (int i = 0; i < nCollisions; ++i)
    {
        _force = Eigen::Vector2d(0., 0.);

        planet *pPlanet1 = static_cast<planet*>(_vecCollisions[i]);
        if (nullptr == pPlanet1)
        {
            std::cout << "ERROR! pPlanet1 is nullptr!" << std::endl;
        }
        if (doesNotCollideWith(pPlanet1->getId()))
        {
            // std::cout << "planet id " << getId() << " and " << pPlanet1->getId() << " do not collide." << std::endl;
            removeCollision(pPlanet1);
            continue;
        }
        
        Eigen::Vector2d p0Pos = getPosition();
        Eigen::Vector2d p1Pos = pPlanet1->getPosition();
        Eigen::Vector2d p0p1Displacement = (p0Pos - p1Pos).normalized();
        Eigen::Vector2d p1p0Displacement = - p0p1Displacement;
        Eigen::Vector2d p0NewVel = getVelocity() - (2.*pPlanet1->getMass()/(getMass()+pPlanet1->getMass())) *
                                  (getVelocity() - pPlanet1->getVelocity()).dot(p0p1Displacement) * p0p1Displacement;
        Eigen::Vector2d deltap0 = getMass() * (p0NewVel - getVelocity());
        addForce(deltap0 / 0.001);
        removeCollision(pPlanet1);
        addDoesNotCollideWith(pPlanet1->getId());      //!!! This is a lazy way of fixing this!
        _bNeedUpdate = true;

        --i; --nCollisions;
    }
    return true;
}