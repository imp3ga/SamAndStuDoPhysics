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
    // std::cout << "nCollisions: " << nCollisions << std::endl;
    for (int i = 0; i < nCollisions; ++i)
    {
        _force = Eigen::Vector2d(0., 0.);

        std::cout << getId() << " Collision! Velocity before is " << _velocity.transpose() << " position is " << _position.transpose() << std::endl;
        // _velocity += 0.001 *_force / getMass();
        // std::cout << "         new velocity calc: " << _velocity.transpose() << std::endl;
        planet *pPlanet1 = static_cast<planet*>(_vecCollisions[i]);
        if (nullptr == pPlanet1)
        {
            std::cout << "ERROR! pPlanet1 is nullptr!" << std::endl;
        }
        if (doesNotCollideWith(pPlanet1->getId()))
        {
            std::cout << "planet id " << getId() << " and " << pPlanet1->getId() << " do not collide." << std::endl;
            removeCollision(pPlanet1);
            continue;
        }
        
        Eigen::Vector2d displacementUnitVec = (pPlanet1->getPosition() - getPosition()).normalized();
        Eigen::Vector2d momentumP0 = getMass() * getVelocity();
        Eigen::Vector2d momentumP1 = pPlanet1->getMass() * pPlanet1->getVelocity();
        Eigen::Vector2d totalMomentum = momentumP0 + momentumP1;
        Eigen::Vector2d impulse;
        if (momentumP1 == Eigen::Vector2d(0., 0.))
        {
            impulse = Eigen::Vector2d(0., 0.);
        }
        else
        {
            // impulse = momentumP1.dot(displacementUnitVec) * displacementUnitVec / 0.001;
            // impulse = -(totalMomentum.normalized().dot(displacementUnitVec) * totalMomentum.norm() * displacementUnitVec) / 0.001;
        }


        // pPlanet1->doesNotInteractWith(this);
        // pPlanet1->removeCollision(this);
    Eigen::Vector2d p0Pos = getPosition();
    Eigen::Vector2d p1Pos = pPlanet1->getPosition();
    Eigen::Vector2d p0p1Displacement = (p0Pos - p1Pos).normalized();
    Eigen::Vector2d p1p0Displacement = - p0p1Displacement;

    Eigen::Vector2d p0NewVel, p1NewVel;
    p0NewVel = getVelocity() - (2.*pPlanet1->getMass()/(getMass()+pPlanet1->getMass())) *
    (getVelocity() - pPlanet1->getVelocity()).dot(p0p1Displacement) * p0p1Displacement;
    p1NewVel = pPlanet1->getVelocity() - (2.*getMass()/(getMass()+pPlanet1->getMass())) *
    (pPlanet1->getVelocity() - getVelocity()).dot(p1p0Displacement) * p1p0Displacement;

    Eigen::Vector2d deltap0 = getMass() * (p0NewVel - getVelocity());

    std::cout << "old method: " << p0NewVel.transpose() << "        " << p1NewVel.transpose() << std::endl;

        // addForce(2.*impulse);
        // pPlanet1->addForce(-impulse);
        addForce(deltap0 / 0.001);
        // pPlanet1->addForce(deltap1 / 0.001);
        // _velocity = p0NewVel;
        // pPlanet1->_velocity = p1NewVel;

        removeCollision(pPlanet1);
        addDoesNotCollideWith(pPlanet1->getId());      //!!! This is a lazy way of fixing this!
        _bNeedUpdate = true;
        
        // pPlanet1->removeCollision(this);
        // pPlanet1->addDoesNotCollideWith(this->getId()); 
        // pPlanet1->_bNeedUpdate = true;
        // std::cout << "newvel0 newvel1: " << p0NewVel.transpose() << ", " << p1NewVel.transpose() << std::endl;
        --i; --nCollisions;
    }
    return false;
}