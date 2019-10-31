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
    // addCustomPlanet(Eigen::Vector2d(0.,0.), Eigen::Vector2d(0.,0.), initPlanetRad, 10000.0, false);
    // planet initPlanet(initPlanetRad, _dMassDensity);
    // initPlanet.setPosition(Eigen::Vector2d(0., 0.));
    // initPlanet.setVelocity(Eigen::Vector2d(0.,0.));
    // _vecPlanets.push_back(initPlanet);
    // std::cout << "Initialized!\n";
    _initPlanetRad = initPlanetRad;
}

void solarSystem::reset()
{
    _vecPlanets.clear();
    init(_initPlanetRad);
}

void solarSystem::centre()
{
    planet &initPlanet = _vecPlanets[0];
    Eigen::Vector2d initPlanetCentre = initPlanet.getPosition();
    _vecPlanets[0].setPosition(Eigen::Vector2d(0.,0.));
    for (int i = 1; i < _nPlanets; ++i)
    {
        planet &p0 = _vecPlanets[i];
        _vecPlanets[i].setPosition(p0.getPosition() - initPlanetCentre);
    }
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
    planet newPlanet(r, _dMassDensity, false);     // Change this to just include all variables
    newPlanet.setPosition(pos);
    newPlanet.setVelocity(vel);

    _vecPlanets.push_back(newPlanet);
    ++_nPlanets;
}

void solarSystem::addCustomPlanet(Eigen::Vector2d pos,
                            Eigen::Vector2d vel,
                            double r, double rho, bool fixed)
{
    planet newPlanet(r, rho, fixed);
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
                    // Get vector between planet centres
                    Eigen::Vector2d relDisP0P1 = (p1Pos - p0Pos).normalized();
                    Eigen::Vector2d relDisP1P0 = - relDisP0P1;

                    // Momentum
                    Eigen::Vector2d p0Momentum = p0Mass * p0Vel;
                    Eigen::Vector2d p1Momentum = p1Mass * p1Vel;

                    // Eigen::Vector2d p0MomentumColComp = relDisP0P1;
                    // Eigen::Vector2d p1MomentumColComp = relDisP1P0;
                    Eigen::Vector2d p0MomentumColComp = relDisP0P1 * p0Momentum.norm();// * p0Momentum.normalized().dot(relDisP0P1.normalized());
                    Eigen::Vector2d p1MomentumColComp = (-relDisP0P1) * p1Momentum.norm();// * p1Momentum.normalized().dot((-relDisP0P1).normalized());
                    
                    Eigen::Vector2d p0NewMomentum = p0Momentum - p0MomentumColComp + p1MomentumColComp;
                    Eigen::Vector2d p1NewMomentum = p1Momentum - p1MomentumColComp + p0MomentumColComp;

                    Eigen::Vector2d p0NewVel = p0NewMomentum / p0Mass;
                    Eigen::Vector2d p1NewVel = p1NewMomentum / p1Mass;
                    // KE consideration
                    double dKE_before = 0.5 * (p0Mass*p0Vel.squaredNorm() + p1Mass*p1Vel.squaredNorm());
                    double dKE_after = 0.5 * (p0Mass*p0NewVel.squaredNorm() + p1Mass*p1NewVel.squaredNorm());
                    // std::cout << "KE before: " << dKE_before << ", KE after: " << dKE_after;
                    double dConserveFactor = dKE_before / dKE_after;

                    p0NewVel = p0NewVel * sqrt(dConserveFactor);
                    p1NewVel = p1NewVel * sqrt(dConserveFactor);
                    dKE_after = 0.5 * (p0Mass*p0NewVel.squaredNorm() + p1Mass*p1NewVel.squaredNorm());
                    // std::cout << ", KE after correction: " << dKE_after << ", dConserveFactor: " << dConserveFactor << "\n\n";

                    // Eigen::Vector2d p0MomentumColComp = relDisP0P1 * p0Momentum.norm() * p0Momentum.normalized().dot(relDisP0P1.normalized());
                    // Eigen::Vector2d p1MomentumColComp = (-relDisP0P1) * p1Momentum.norm() * p1Momentum.normalized().dot((-relDisP0P1).normalized());
                    
                    // Eigen::Vector2d p0MomentumColComp = relDisP0P1 * p0Momentum.dot(relDisP0P1);
                    // Eigen::Vector2d p1MomentumColComp = (-relDisP0P1) * p1Momentum.dot(-relDisP0P1);


                    // Eigen::Vector2d p0MomentumColComp = (p0Momentum) * (p0Vel.normalized()).dot(relDisP0P1);
                    // Eigen::Vector2d p1MomentumColComp = (p1Momentum) * (p1Vel.normalized()).dot(relDisP0P1);
                    // Subtract from original momentum vectors
                    // Eigen::Vector2d p0NewMomentum = p0Momentum - p0MomentumColComp + p1MomentumColComp;
                    // Eigen::Vector2d p1NewMomentum = p1Momentum - p1MomentumColComp + p0MomentumColComp;

                    // Conservation of momentum
                    // p0Momentum + p1Momentum = p0NewMomentum + p1NewMomentum
                    //                         = p0Momentum - p0MomentumColComp + p1MomentumColComp + p1Momentum - p1MomentumColComp + p0MomentumColComp
                    //                         = p0Momentum + p1Momentum

                    // std::cout << "Total momentum before: " << (p0Momentum + p1Momentum).transpose()
                    //         << ",   Total momentum after: " << (p0NewMomentum + p1NewMomentum).transpose() << "\n";
                    //         std::cout << "Momentum magnitude change of collision: " << (p0Momentum + p1Momentum).norm() - (p0NewMomentum + p1NewMomentum).norm() << "\n\n\n";
                    // Calculate new velocities and set
                    // Eigen::Vector2d p0NewVel = p0NewMomentum / p0Mass;
                    // Eigen::Vector2d p1NewVel = p1NewMomentum / p1Mass;


                    // // Bounce
                    // // Get momentum component in direction of collision
                    // Eigen::Vector2d p0NewVel = ((p0Mass - p1Mass) / (p0Mass + p1Mass))*p0Vel
                    //                          + (2*p1Mass / (p0Mass + p1Mass))* p1Vel;

                    // Eigen::Vector2d p1NewVel = (2*p0Mass / (p0Mass + p1Mass))*p0Vel
                    //                          - ((p0Mass - p1Mass) / (p0Mass + p1Mass))*p1Vel;

                    // // Rotate to account for glance
                    // // Get vector between planet centres
                    // Eigen::Vector2d relDisP0P1 = (p1Pos - p0Pos).normalized();
                    // Eigen::Vector2d relDisP1P0 = - relDisP0P1;
                    // // Reflect
                    // Eigen::Vector2d reflection0 = (p0Vel - 2 * (p0Vel.dot(relDisP1P0))*relDisP1P0).normalized();
                    // Eigen::Vector2d reflection1 = (p1Vel - 2 * (p1Vel.dot(relDisP0P1))*relDisP0P1).normalized();

                    // p0NewVel = reflection0 * p0NewVel.norm();
                    // p1NewVel = reflection1 * p1NewVel.norm(); 

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
                    planet pNew(newRadius, _dMassDensity, false);
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
        if(!planet._bFixed)
        {
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
}