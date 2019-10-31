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
                // Check for bounce or absorb
                Eigen::Vector2d p0Vel = p0.getVelocity();
                Eigen::Vector2d p1Vel = p1.getVelocity();
                double dDot = p0Vel.dot(p1Vel);
                if(true)//dDot > 0.0)
                {
                    // Bounce
                    // Get vector between planet centres
                    Eigen::Vector2d relDisP0P1 = (p1Pos - p0Pos).normalized();
                    // Eigen::Vector2d collisionPlane(relDisP0P1[1], relDisP0P1[0]);
                    // Get momentum component in direction of collision
                    Eigen::Vector2d p0Momentum = p0Mass * p0Vel;
                    Eigen::Vector2d p1Momentum = p1Mass * p1Vel;

                    // Eigen::Vector2d p0MomentumColComp = relDisP0P1 * (p0Momentum.normalized()).dot(relDisP0P1)
                    //                                     *p0Momentum.norm();
                    // Eigen::Vector2d p1MomentumColComp = -relDisP0P1 * (p1Momentum.normalized()).dot(-relDisP0P1)
                    //                                     *p1Momentum.norm();      
                    Eigen::Vector2d p0MomentumColComp = relDisP0P1 * p0Momentum.dot(relDisP0P1);
                    Eigen::Vector2d p1MomentumColComp = (-relDisP0P1) * p1Momentum.dot(-relDisP0P1);


                    // Eigen::Vector2d p0MomentumColComp = (p0Momentum) * (p0Vel.normalized()).dot(relDisP0P1);
                    // Eigen::Vector2d p1MomentumColComp = (p1Momentum) * (p1Vel.normalized()).dot(relDisP0P1);
                    // Subtract from original momentum vectors
                    Eigen::Vector2d p0NewMomentum = p0Momentum - p0MomentumColComp + p1MomentumColComp;
                    Eigen::Vector2d p1NewMomentum = p1Momentum - p1MomentumColComp + p0MomentumColComp;
                    std::cout << "Total momentum before: " << (p0Momentum + p1Momentum).transpose()
                            << ",   Total momentum after: " << (p0NewMomentum + p1NewMomentum).transpose() << "\n";
                            std::cout << "Momentum change of collision: " << (p0Momentum + p1Momentum - p0NewMomentum - p1NewMomentum).transpose() << "\n";
                    // Calculate new velocities and set
                    Eigen::Vector2d p0NewVel = p0NewMomentum / p0Mass;
                    Eigen::Vector2d p1NewVel = p1NewMomentum / p1Mass;
                    _vecPlanets[i].setVelocity(p0NewVel);
                    _vecPlanets[j].setVelocity(p1NewVel);
                    // Update position to avoid repeated collisions
                    _vecPlanets[i].setPosition(p0Pos + (0.001 * p0NewVel));
                    _vecPlanets[j].setPosition(p1Pos + (0.001 * p1NewVel));

                    // std::cout << "pos0 pos1: " << p0Pos.transpose() << "       " << p1Pos.transpose() << "\n";
                    // std::cout << "norm vec: " << relDisP0P1.transpose() << "\n";
                    // Eigen::Vector2d relDisP1P0 = - relDisP0P1;
                    // // Get momentum change
                    // Eigen::Vector2d p0mv = p0Mass * p0Vel; 
                    // Eigen::Vector2d p1mv = p1Mass * p1Vel;
                    // Eigen::Vector2d p0DeltaMv = - p0mv * p0Vel.normalized().dot(relDisP0P1);
                    // Eigen::Vector2d p1DeltaMv = - p1mv * p1Vel.normalized().dot(relDisP1P0);
                    // // Solve for velocities
                    // Eigen::Vector2d p0NewV = (p0mv + p0DeltaMv) / p0Mass;
                    // Eigen::Vector2d p1NewV = (p1mv + p1DeltaMv) / p1Mass;
                    // // Set
                    // _vecPlanets[i].setVelocity(p0NewV); _vecPlanets[j].setVelocity(p1NewV);
                    // _vecPlanets[i].setPosition(p0Pos + (0.001 * p0NewV)); _vecPlanets[j].setPosition(p1Pos + (0.001 * p1NewV));


                    // std::cout << "p0 pre and new vel: " << p0Vel.transpose() << "       " << -p0Vel.transpose() << "\n"
                    //  << "p1 pre and new vel: " << p1Vel.transpose() << "       " << -p1Vel.transpose() << "\n\n";
                    // _vecPlanets[i].setVelocity(p0Vel); _vecPlanets[j].setVelocity(-p1Vel);
                    // _vecPlanets[i].setPosition(p0Pos - (0.001 * p0Vel)); _vecPlanets[j].setPosition(p1Pos - (0.001 * p1Vel));
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