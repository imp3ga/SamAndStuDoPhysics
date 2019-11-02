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

std::pair<Eigen::Vector2d,Eigen::Vector2d> AstroObjectBase::collisionResult(AstroObjectBase obj){
    
    Eigen::Vector2d p0p1Displacement = (getPosition() - obj.getPosition()).normalized();
    Eigen::Vector2d p1p0Displacement = - p0p1Displacement;

    Eigen::Vector2d p0NewVel = getVelocity() - (2.* obj.getMass()/(getMass()+obj.getMass())) *
    (getVelocity() - obj.getVelocity()).dot(p0p1Displacement) * p0p1Displacement;
    Eigen::Vector2d p1NewVel = obj.getVelocity() - (2.*getMass()/(getMass()+obj.getMass())) *
    (obj.getVelocity() - getVelocity()).dot(p1p0Displacement) * p1p0Displacement;

    return std::make_pair(p0NewVel, p1NewVel);
}
