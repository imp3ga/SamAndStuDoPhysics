#include "include/solarSystem.h"
#include <iostream>

solarSystem::solarSystem(double rho)
{
    _dMassDensity = rho;
}

void solarSystem::init(double initPlanetRad)
{
    // Add the initial large planet
    addPlanet(Eigen::Vector2d(0.,0.), Eigen::Vector2d(0.,0.), initPlanetRad);
    // planet initPlanet(initPlanetRad, _dMassDensity);
    // initPlanet.setPosition(Eigen::Vector2d(0., 0.));
    // initPlanet.setVelocity(Eigen::Vector2d(0.,0.));
    // _vecPlanets.push_back(initPlanet);
    std::cout << "Initialized!\n";
}

void solarSystem::getInfo(std::vector<Eigen::Vector2d> &allCentres,
                                std::vector<double> &allRadii)
{
    for (int i = 0; i < _nPlanets; ++i)
    {
        planet &planet = _vecPlanets[i];
        allCentres.push_back(planet.getPosition());
        allRadii.push_back(planet.getRadius());
    }
}

void solarSystem::addPlanet(Eigen::Vector2d pos,
                            Eigen::Vector2d vel,
                            double r)
{
    planet newPlanet(r, _dMassDensity);     // Change this to just include all variables
    newPlanet.setPosition(pos);
    newPlanet.setVelocity(vel);

    _vecPlanets.push_back(newPlanet);
    ++_nPlanets;
}

void solarSystem::update()
{
    Eigen::Vector2d aForces[_nPlanets - 1];
    std::vector<planet> newPlanets;
    for (int i = 0; i < _nPlanets; ++i)
    {
        planet &p0 = _vecPlanets[i];
        Eigen::Vector2d p0Pos = p0.getPosition();
        double p0Mass = p0.getMass();
        if(p0Mass < 0)
        {
            continue;
        }

        Eigen::Vector2d force(0., 0.);
        for (int j = 0; j < _nPlanets; ++j)
        {
            if(i == j)
            {
                continue;
            }
            planet &p1 = _vecPlanets[j];
            Eigen::Vector2d p1Pos = p1.getPosition();
            double p1Mass = p1.getMass();
            if(p1Mass < 0.)
            { 
                continue;
            }
            // Check for collisions
            double dist = (p1Pos - p0Pos).norm();
            if (dist < p0.getRadius() + p1.getRadius())
            {
                double newMass = p0Mass + p1Mass;
                double newRadius = sqrt(newMass / _dMassDensity);       // Preferably remove this sqrt somehow
                planet pNew(newRadius, _dMassDensity);
                pNew.setPosition(p0Mass > p1Mass ? p0Pos : p1Pos);
                Eigen::Vector2d p0Vel = p0.getVelocity();
                Eigen::Vector2d p1Vel = p1.getVelocity();
                Eigen::Vector2d newVel = (p0Mass * p0Vel + p1Mass * p1Vel) / newMass;
                pNew.setVelocity(newVel);
                newPlanets.push_back(pNew);
                _vecPlanets[i].setMass(-1.0); _vecPlanets[j].setMass(-1.0);
                continue;
            }
            double dForce = p0Mass * p1Mass / (p1Pos - p0Pos).squaredNorm();
            force += dForce * (p1Pos - p0Pos);
        }
        aForces[i] = force;
    }
        // Remove negative mass planets (ie collided objects), add new ones
        for (int i = 0; i < _nPlanets; ++i)
        {
            planet &planet = _vecPlanets[i];
            double m = planet.getMass();
            if(m < 0.0)
            {
                _vecPlanets.erase(_vecPlanets.begin() + i);
                ++i;
            }
        }
        _vecPlanets.insert(_vecPlanets.end(), newPlanets.begin(), newPlanets.end());
        _nPlanets = _vecPlanets.size();

    for (int i = 0; i < _vecPlanets.size(); ++i)
    {
        planet &planet = _vecPlanets[i];
        // Update velocity
        double mass = planet.getMass();
        Eigen::Vector2d acc = aForces[i] / mass;
        Eigen::Vector2d newVel = planet.getVelocity() + (0.001 * acc);      // Physics update is 1ms
        planet.setVelocity(newVel);
        // Update position
        Eigen::Vector2d newPos = planet.getPosition() + (0.001 * newVel);   // Physics update is 1ms
        planet.setPosition(newPos);
    }
}