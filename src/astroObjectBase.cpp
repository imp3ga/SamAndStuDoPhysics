#include "include/astroObjectBase.h"


void AstroObjectBase::setMass(const double m)
{
    dMass = m;
}

void AstroObjectBase::setPosition(const Eigen::Vector2d s)
{
    position = s;
}

void AstroObjectBase::setVelocity(const Eigen::Vector2d v)
{
    velocity = v;
}


