#include "include/planet.h"

planet::planet(const double r, const double rho, bool fixed)
{
    setRadius(r);
    setMass(r * r * rho);
    setFixed(fixed);
}

void planet::setRadius(const double r)
{
    dRadius = r;
}