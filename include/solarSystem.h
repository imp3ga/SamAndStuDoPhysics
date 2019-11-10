#ifndef SOLARSYSTEM_H_
#define SOLARSYSTEM_H_

#include "include/astroObjectBase.h"
#include <Eigen/Core>
// #include <Eigen/Dense>
#include <GL/freeglut.h>

#include <vector>
class solarSystem
{
    public:
        // solarSystem();
        bool init(double dInitPlanetMass, double dMassDensity);
        bool addObject(double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, double dMassDensity);
        bool reset();
        bool centre();
        bool removeObject(const int nId);
        bool update();
        bool getGlVertices(std::vector<std::vector<Eigen::Vector2d>> &vecVertices);
        bool updateMotion();

    private:
        bool checkCollisions();
        bool resolveCollisions();
        bool removeObjects();
        bool applyGravity();

        int _nPlanetIdx = 0;
        double _dRestCoef = 1.0, _dInitPlanetMass, _dInitMassDensity;
        std::vector<AstroObjectBase*> _vecObjects;
        std::vector<std::vector<AstroObjectBase*>::iterator> _vecToBeRemoved;       
};

#endif