#include <iostream>
#include "include/astroObjectBase.h"

AstroObjectBase::AstroObjectBase(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
                                 double dRestCoef, double dMassDensity)
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

Eigen::Vector2d AstroObjectBase::getVelocity()
{
    return _velocity;
}

bool AstroObjectBase::addCollision(AstroObjectBase *pObj1)
{
    _vecCollisions.push_back(pObj1);
}

bool AstroObjectBase::update(std::vector<AstroObjectBase*> *refVecObjects)
{
    _force = Eigen::Vector2d(0., 0.);

    // std::cout << "Calculating gravity" << std::endl;
    if (!calculateForceGravity(refVecObjects))
    {
        std::cout << "Error calculating gravity for planet ID " << _nId << std::endl;
        // return false;
    }
    // std::cout << "Calculating collisions" << std::endl;
    if (!calculateForceCollisions(refVecObjects))
    {
        std::cout << "Error calculating collisions for planet ID " << _nId << std::endl;
        // return false;
    }
    // std::cout << "Updating pos and vel" << std::endl;
    if (!updatePositionVelocity())
    {
        std::cout << "Planet ID " << _nId << " updated." << std::endl;
        return true;
    }
    else
    {
        std::cout << "Planet ID " << _nId << " NOT updated." << std::endl;
        return true;
    }
    
}

bool AstroObjectBase::hasAlreadyInteracted(std::vector<int> _otherDontInteractIds){

    int nSize = static_cast<int>(_otherDontInteractIds.size());

    for(int i = 0 ; i< nSize; i++){

        if(find(_otherDontInteractIds.begin(), _otherDontInteractIds.end(), _nId) != _otherDontInteractIds.end())
        {
            return true;
        }
    }

    return false;
}

std::vector<int> AstroObjectBase::getDontInteractIDs()
{
    return _vecDontInteractIds;
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

void AstroObjectBase::setAlreadyInteracted(bool value){
    _alreadyInteracted = value;
}

bool AstroObjectBase::calculateForceGravity(std::vector<AstroObjectBase*> *_pVecObjects)
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
