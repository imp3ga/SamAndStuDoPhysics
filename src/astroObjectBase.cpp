#include "include/astroObjectBase.h"


void AstroObjectBase::setMass(const double m)
{
    _dMass = m;
}

void AstroObjectBase::setPosition(const Eigen::Vector2d s)
{
    _position = s;
}

void AstroObjectBase::setVelocity(const Eigen::Vector2d v)
{
    _velocity = v;
}

void AstroObjectBase::setFixed(bool bFixed)
{
    _bFixed = bFixed;
}

void AstroObjectBase::setNeedsUpdate(bool bUpdate)
{
    _bNeedUpdate = bUpdate;
}


