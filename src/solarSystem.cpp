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
    _initPlanetRad = initPlanetRad;
}

void solarSystem::reset()
{
    _vecPlanets.clear();
    init(_initPlanetRad);
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
    Eigen::Vector2d totalMag(0.0, 0.0);
    for (int i = 0; i < _nPlanets; ++i)
    {
        planet &p0 = _vecPlanets[i];
        if (p0._bNeedUpdate)
        {
            continue;
        }
        Eigen::Vector2d p0Pos = p0.getPosition();
        double p0Mass = p0.getMass();
        // if(p0Mass < 0)
        // {
        //     continue;
        // }
        totalMag += p0.getVelocity() * p0Mass;

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
            if (p1._bNeedUpdate)
            {
                continue;
            }
            // if(p1Mass < 0.)
            // { 
            //     continue;
            // }
            // Check for collisions
            double dist = (p1Pos - p0Pos).norm();
            if (dist < p0.getRadius() + p1.getRadius())
            {
                // Check for bounce or absorb
                Eigen::Vector2d p0Vel = p0.getVelocity();
                Eigen::Vector2d p1Vel = p1.getVelocity();
                double dDot = p0Vel.dot(p1Vel);
                if(true)//dDot > 0.0)
                {
                    // Bounce
                    // Get momentum component in direction of collision
                    Eigen::Vector2d p0NewVel = ((p0Mass - p1Mass) / (p0Mass + p1Mass))*p0Vel
                                             + (2*p1Mass / (p0Mass + p1Mass))* p1Vel;

                    Eigen::Vector2d p1NewVel = (2*p0Mass / (p0Mass + p1Mass))*p0Vel
                                             - ((p0Mass - p1Mass) / (p0Mass + p1Mass))*p1Vel;

                    // Rotate to account for glance
                    // Get vector between planet centres
                    Eigen::Vector2d relDisP0P1 = (p1Pos - p0Pos).normalized();
                    Eigen::Vector2d relDisP1P0 = - relDisP0P1;
                    // Reflect
                    Eigen::Vector2d reflection0 = (p0Vel - 2 * (p0Vel.dot(relDisP1P0))*relDisP1P0).normalized();
                    Eigen::Vector2d reflection1 = (p1Vel - 2 * (p1Vel.dot(relDisP0P1))*relDisP0P1).normalized();

                    p0NewVel = reflection0 * p0NewVel.norm();
                    p1NewVel = reflection1 * p1NewVel.norm(); 

                    _vecPlanets[i].setVelocity(p0NewVel);
                    _vecPlanets[j].setVelocity(p1NewVel);

                    _vecPlanets[i]._bNeedUpdate = true;
                    _vecPlanets[j]._bNeedUpdate = true;
                    continue;
                }
                else
                {
                    // Absorb
                    double newMass = p0Mass + p1Mass;
                    double newRadius = sqrt(newMass / _dMassDensity);       // Preferably remove this sqrt somehow
                    planet pNew(newRadius, _dMassDensity);
                    pNew.setPosition(p0Mass > p1Mass ? p0Pos : p1Pos);
                    Eigen::Vector2d newVel = (p0Mass * p0Vel + p1Mass * p1Vel) / newMass;
                    pNew.setVelocity(newVel);
                    newPlanets.push_back(pNew);
                    _vecPlanets[i].setMass(-1.0); _vecPlanets[j].setMass(-1.0);
                    continue;
                }
            }
            double dForce = p0Mass * p1Mass / (p1Pos - p0Pos).squaredNorm();
            force += dForce * (p1Pos - p0Pos);
        }
        aForces[i] = force;
    }
    // std::cout << "Total momentum of system: " << totalMag.transpose() << "\n";
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
        planet._bNeedUpdate = false;
    }
}