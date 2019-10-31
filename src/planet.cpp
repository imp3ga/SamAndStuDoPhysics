#include "include/planet.h"

planet::planet(const double r, const double rho, bool fixed)
{
    setRadius(r);
    setMass(r * r * rho);
    _bFixed = fixed;
}

void planet::setRadius(const double r)
{
    dRadius = r;
}

void planet::setMass(const double m)
{
    dMass = m;
}

void planet::setPosition(const Eigen::Vector2d s)
{
    position = s;
}

void planet::setVelocity(const Eigen::Vector2d v)
{
    velocity = v;
}


