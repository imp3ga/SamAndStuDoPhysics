#include <iostream>
#include "include/astroObjectBase.h"

AstroObjectBase::AstroObjectBase(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
                                 double dRestCoef, double dMassDensity, std::vector<AstroObjectBase> &refVecObjects,
                                 bool bInteracts)
{
    if(0 >= dMass)
    {
        std::cout << "ERROR: Cannot have 0 or negative mass!" << std::endl;
        exit(1);
    }
    _nId = nId;
    _dMass = dMass;
    _position = position;
    _velocity = velocity;
    _dRestCoef = dRestCoef;
    _dMassDensity = dMassDensity;
    _pVecObjects = &refVecObjects;
    _bInteracts = bInteracts;

    _force = Eigen::Vector2d(0., 0.);
    _momentum = Eigen::Vector2d(0., 0.);

    std::cout << (_pVecObjects)->size() << " located at " << _pVecObjects <<  std::endl;

}

double AstroObjectBase::getMass()
{
    return _dMass;
}

double AstroObjectBase::getKE()
{
    return 0.5 * _dMass * _velocity.squaredNorm();
}

Eigen::Vector2d AstroObjectBase::getPosition()
{
    return _position;
}

Eigen::Vector2d AstroObjectBase::getVelocity()
{
    return _velocity;
}

bool AstroObjectBase::addCollision(int nId)
{
    _vecCollisionIds.push_back(nId);
}

bool AstroObjectBase::update()
{
    _force = Eigen::Vector2d(0., 0.);
    if(!calculateForceGravity())
    {
        std::cout << "Error calculating gravity for planet ID " << _nId << std::endl;
        exit(1);
    }
    if(!calculateForceCollisions())
    {
        std::cout << "Error calculating collisions for planet ID " << _nId << std::endl;
        exit(1);
    }
}

bool AstroObjectBase::calculateForceGravity()
{
    // int nSize = static_cast<int>(_pVecObjects->size());
    // for (int i = 0; i < nSize; ++i)
    // {

    // }
    return false;
}

bool AstroObjectBase::calculateForceCollisions()
{
    return false;
}