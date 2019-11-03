#include "include/astroObjectBase.h"


void AstroObjectBase::setMass(const double m)
{
    _dMass = m;
}

void AstroObjectBase::setPosition(const Eigen::Vector2d s)
{
    _positionPrev = _position;
    _position = s;
}

void AstroObjectBase::setVelocity(const Eigen::Vector2d v)
{
    _velocityPrev = _velocity;
    _velocity = v * _dRestCoef;
}


void AstroObjectBase::setFixed(bool bFixed)
{
    _bFixed = bFixed;
}

void AstroObjectBase::setNeedsUpdate(bool bUpdate)
{
    _bNeedUpdate = bUpdate;
}

void AstroObjectBase::revertPosition()
{
    // _velocity = _velocityPrev;
    _position = _positionPrev;
}


