#include <iostream>
#include "include/astroObjectBase.h"

AstroObjectBase::AstroObjectBase(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
                                 double dRestCoef, double dMassDensity, std::vector<AstroObjectBase*> &refVecObjects)
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


bool AstroObjectBase::setPosition(Eigen::Vector2d position)
{
    _position = position;
}

Eigen::Vector2d AstroObjectBase::getVelocity()
{
    return _velocity;
}

bool AstroObjectBase::addCollision(int nId)
{
    _vecCollisionIds.push_back(nId);
}

bool AstroObjectBase::update(Eigen::VectorXd &objectDistances)
{
    _pObjectDistances = &objectDistances;
    _force = Eigen::Vector2d(0., 0.);

    if (!calculateForceGravity())
    {
        std::cout << "Error calculating gravity for planet ID " << _nId << std::endl;
        return false;
    }

    if (!calculateForceCollisions())
    {
        std::cout << "Error calculating collisions for planet ID " << _nId << std::endl;
        return false;
    }

    if (!updatePositionVelocity())
    {
        std::cout << "Planet ID " << _nId << " updated." << std::endl;
        return true;
    }
    else
    {
        std::cout << "Planet ID " << _nId << "NOT updated." << std::endl;
        return true;
    }
    
}

int AstroObjectBase::getId()
{
    return _nId;
}

bool AstroObjectBase::updatePositionVelocity()
{
    if(!_bFixed && !_bNeedUpdate) // Not sure if _bNeedUpdate is needed anymore?
    {
        _velocity += 0.001 * (_force / _dMass);           // Physics update is 1ms
        _position += 0.001 * _velocity;                   // Physics update is 1ms
        return true;
    }
    else
    {
        return false;
    }
}

bool AstroObjectBase::calculateForceGravity()
{
    if (nullptr == _pVecObjects)
    {
        std::cout << "ERROR! _pVecObjects is nullptr!" << std::endl;
    }
    if (nullptr == _pObjectDistances)
    {
        std::cout << "ERROR! _pObjectDistances is nullptr!" << std::endl;
    }
    
    int nSize = static_cast<int>(_pVecObjects->size());
    for (int i = 0; i < nSize; ++i)
    {
        // Object to compare against
        AstroObjectBase *pObj = (*_pVecObjects)[i];            // This could be better
        std::cout << "object with id " << pObj->_nId << " has mass " << pObj->getMass() << std::endl;
        
        if (_nId == pObj->_nId)
        {
            // Don't interact with self
            continue;
        }
        auto it = find(pObj->_vecDontInteractIds.begin(),
                       pObj->_vecDontInteractIds.end(),
                       _nId);

        if (it != pObj->_vecDontInteractIds.end())
        {
            // Dont interact with other object
            continue;
        }

        double obj1Mass = pObj->getMass();
        Eigen::Vector2d obj1Pos = pObj->getPosition();    

        // Add gravitational force (G is arbitrary)
        double dForce = 1E3 * getMass() * obj1Mass / (obj1Pos - getPosition()).squaredNorm();
        _force += dForce * (obj1Pos - getPosition()).normalized();
    }
    return true;
}
