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

bool AstroObjectBase::collidesWith(AstroObjectBase *pObj1)
{
    int nId = pObj1->getId();
    auto it = std::find_if(_vecCollisions.begin(),
                           _vecCollisions.end(),
                            [nId] (AstroObjectBase* pObj) 
                            {
                                return nId == pObj->getId();
                            });
    return !(it == _vecCollisions.end());
}

bool AstroObjectBase::doesNotInteractWith(int nId)
{
    auto it = std::find(_vecDontInteractIds.begin(),
                        _vecDontInteractIds.end(),
                         nId);
    return !(it == _vecDontInteractIds.end());
}

Eigen::Vector2d AstroObjectBase::getVelocity()
{
    return _velocity;
}

bool AstroObjectBase::addCollision(AstroObjectBase *pObj1)
{
    AstroObjectBase *pNew = pObj1;
    pNew = pObj1;
    _vecCollisions.push_back(pNew);
}

bool AstroObjectBase::removeCollision(AstroObjectBase *pObj1)
{
    int nId = pObj1->getId();
    auto it = std::find_if(_vecCollisions.begin(),
                           _vecCollisions.end(),
                            [nId] (AstroObjectBase* pObj) 
                            {
                                return nId == pObj->getId();
                            });
    if (it != _vecCollisions.end())
    {
        _vecCollisions.erase(it);
        return true;
    }
    else
    {
        std::cout << "ERROR! Could not remove collision with ID " << nId << " from object ID " << _nId << std::endl;
    }
}

bool AstroObjectBase::updateMotion()
{
    // std::cout << "Updating pos and vel" << std::endl;
    if (!updatePositionVelocity())
    {
        // std::cout << "Planet ID " << _nId << " updated." << std::endl;
        return true;
    }
    else
    {
        // std::cout << "Planet ID " << _nId << " NOT updated." << std::endl;
        return true;
    }
    
}

bool AstroObjectBase::addForce(Eigen::Vector2d force)
{
    _force += force;
    std::cout << _nId << " gets force change of " << force.transpose() << ", total force now is " << _force.transpose() << std::endl;
}

bool AstroObjectBase::updateForces()
{
    _force = Eigen::Vector2d(0., 0.);
    // std::cout << "Calculating gravity" << std::endl;
    if (!calculateForceGravity())
    {
        // std::cout << "Error calculating gravity for planet ID " << _nId << std::endl;
        // return false;
    }
    // std::cout << "Calculating collisions" << std::endl;
    if (!calculateForceCollisions())
    {
        // std::cout << "Error calculating collisions for planet ID " << _nId << std::endl;
        // return false;
    }
}


int AstroObjectBase::getId()
{
    return _nId;
}

double AstroObjectBase::getDistanceBetween(AstroObjectBase *pObj1)
{
    Eigen::Vector2d displacement = pObj1->getPosition() - _position;
    return displacement.norm();
}


bool AstroObjectBase::updatePositionVelocity()
{
    // if(!_bFixed && _bNeedUpdate) // Not sure if _bNeedUpdate is needed anymore?
    // {
        // std::cout << " Updating pos and vel for id " << _nId << ", force is " << _force.transpose() << std::endl;
        _velocity += 0.001 * (_force / _dMass);           // Physics update is 1ms
        _position += 0.001 * _velocity;                   // Physics update is 1ms
        // std::cout << " vel is " << _velocity.transpose() << ", force is " << _velocity.transpose() << std::endl;

        return true;
    // }
    // else
    // {
        // return false;
    // }
}

bool AstroObjectBase::calculateForceGravity()
{
    if (nullptr == _pVecObjects)
    {
        std::cout << "ERROR! _pVecObjects is nullptr!" << std::endl;
    }
    
    int nSize = static_cast<int>(_pVecObjects->size());
    for (int i = 0; i < nSize; ++i)
    {
        // Object to compare against
        AstroObjectBase *pObj = (*_pVecObjects)[i];            // This could be better
        if (_nId == pObj->_nId)
        {
            // Don't interact with self
            continue;
        }

        if (doesNotInteractWith(pObj->getId()))
        {
            continue;
        }
        // auto it = find(pObj->_vecDontInteractIds.begin(),
        //                pObj->_vecDontInteractIds.end(),
        //                _nId);

        // if (it != pObj->_vecDontInteractIds.end())
        // {
        //     // Dont interact with other object
        //     continue;
        // }

        double obj1Mass = pObj->getMass();
        Eigen::Vector2d obj1Pos = pObj->getPosition(); 
        Eigen::Vector2d displacement = obj1Pos - getPosition();
        if (0 >= displacement.norm() - 1E-1)
        {
            std::cout << "Distance between " << _nId << " and " << pObj->getId() << 
            " too small, ignoring gravity force. " << std::endl;
            return false; 
        }

        // Add gravitational force (G is arbitrary)
        double dForce = _G * getMass() * obj1Mass / displacement.squaredNorm();
        _force += dForce * (obj1Pos - getPosition()).normalized();
    }
    // std::cout << "total force on id " << _nId << " is " << _force.transpose() << std::endl;
    _bNeedUpdate = true;
    return true;
}
