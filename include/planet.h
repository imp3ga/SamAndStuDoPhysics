#ifndef PLANET_H_
#define PLANET_H_

#include <Eigen/Core>
#include "include/astroObjectBase.h"

#define CONST_PI 3.14159265358979

class planet : public AstroObjectBase
{
    public:
        planet(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
               double dRestCoef, double dMassDensity);
        double getRadius();
        bool calculateForceCollisions(std::vector<AstroObjectBase> &refVecObjects);

    private:
        double _dRadius;
        bool _bToBeRemoved = false;
};

#endif