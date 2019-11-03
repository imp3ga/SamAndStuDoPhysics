#include "include/solarSystem.h"
#include <iostream>

solarSystem::solarSystem(double rho)
{
    _dMassDensity = rho;
}

void solarSystem::init(double initPlanetRad)
{
    // Add the initial large planet
    addPlanet(Eigen::Vector2d(0.,0.), Eigen::Vector2d(0.,0.), 100.);
    // addCustomPlanet(Eigen::Vector2d(0.,0.), Eigen::Vector2d(0.,0.), initPlanetRad, 10000.0, false);
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

    static int idx = 0;
    newPlanet.nIdx = idx;
    ++idx;

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

void solarSystem::updatePositionVelocity()
{
    Eigen::Vector2d aForces[_nPlanets - 1];
    for (int i = 0; i < _nPlanets; ++i)
    {
        planet &p0 = _vecPlanets[i];
        if (p0.needsUpdate())
        {
            continue;
        }
        Eigen::Vector2d p0Pos = p0.getPosition();
        double p0Mass = p0.getMass();
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
            if (p1.needsUpdate())
            {
                continue;
            }
            double dForce = p0Mass * p1Mass / (p1Pos - p0Pos).squaredNorm();
            force += dForce * (p1Pos - p0Pos);
        }
        aForces[i] = force;
    }

    for (int i = 0; i < _vecPlanets.size(); ++i)
        {
            planet &planet = _vecPlanets[i];
            if(!planet.isFixed())
            {   
                // Update velocity
                double mass = planet.getMass();
                Eigen::Vector2d acc = aForces[i] / mass;
                Eigen::Vector2d newVel = planet.getVelocity() + (0.001 * acc);      // Physics update is 1ms
                planet.setVelocity(newVel);
                // Update position
                Eigen::Vector2d newPos = planet.getPosition() + (0.001 * newVel);   // Physics update is 1ms
                _vecPlanets[i].setPosition(newPos);
                _vecPlanets[i].setNeedsUpdate(false);
            }
        }
}

void solarSystem::checkCollisions()
{
    std::vector<planet> newPlanets;
    for (int i = 0; i < _nPlanets; ++i)
    {
        planet &p0 = _vecPlanets[i];
        Eigen::Vector2d p0Pos = p0.getPosition();
        double p0Mass = p0.getMass();

        Eigen::Vector2d force(0., 0.);
        for (int j = 0; j < _nPlanets; ++j)
        {
            if(i == j)
            {
                continue;
            }
            planet &p1 = _vecPlanets[j];
            bool bAlreadyDone = false;
            for(int k = 0; k <p1._vecCurrentCollisions.size(); ++k)
            {
                if(p1._vecCurrentCollisions[k] == i)
                {
                    // std::cout << "Collision already done between i " << i << " and j " << j << "\n";
                    bAlreadyDone = true;
                }
            }
            if(bAlreadyDone)
            {
                continue;
            }

            Eigen::Vector2d p1Pos = p1.getPosition();
            double p1Mass = p1.getMass();
            // Check for collisions
            double dist = (p1Pos - p0Pos).norm();
            if (dist < p0.getRadius() + p1.getRadius())
            {
                p0._vecCurrentCollisions.push_back(j);
                p1._vecCurrentCollisions.push_back(i);
            }
        }
    }
    // for (int i = 0; i < _nPlanets; ++i)
    // {
    //     planet &p0 = _vecPlanets[i];
    //     std::vector<int> &vCollisions = p0._vecCurrentCollisions;
    //     if(vCollisions.size() > 0)
    //     {
    //         std::cout << "Planet " << i << " collides with: ";
    //         for(int j = 0; j < vCollisions.size(); ++j)
    //         {
    //             std::cout << vCollisions[j] << ", ";
    //         }
    //         std::cout << "\n";
    //     }
    // }

    // Do collisions
    for (int i = 0; i < _nPlanets; ++i)
    {
        planet &p0 = _vecPlanets[i];
        std::vector<int> &vCollisions = p0._vecCurrentCollisions;
        if(vCollisions.size() > 0)
        {           
            if(p0.needsUpdate() || _vecPlanets[vCollisions[0]].needsUpdate())
            {
                // std::cout << "Planet(s) have already been updated!\n";
                continue;
            }

            // Two body collision
            if(vCollisions.size() == 1)
            {
                // Check it's not actually a three body collision
                bool bThree = false;
                for(int j = 0; j < vCollisions.size(); ++j)
                {
                    if(_vecPlanets[vCollisions[j]]._vecCurrentCollisions.size() == 2)
                    {
                        bThree = true;
                    }
                }
                if(!bThree)
                {
                    planet &p1 = _vecPlanets[vCollisions[0]];
                    Eigen::Vector2d p0NewVel, p1NewVel;
                    twoBodyCollision(p0, p1, p0NewVel, p1NewVel);

                    _vecPlanets[i].setVelocity(p0NewVel);
                    _vecPlanets[vCollisions[0]].setVelocity(p1NewVel);

                    _vecPlanets[i].setNeedsUpdate(true);
                    _vecPlanets[vCollisions[0]].setNeedsUpdate(true);
                    // std::cout << "Planet velocities have been updated.\n";
                    _vecPlanets[i]._vecCurrentCollisions.clear();
                    _vecPlanets[vCollisions[0]]._vecCurrentCollisions.clear();
                }
            }

            // Three body collision
            if(vCollisions.size() == 2)
            {
                std::cout << "Three body collision!\n";
                planet &p1 = _vecPlanets[vCollisions[0]];
                planet &p2 = _vecPlanets[vCollisions[1]];

                double dMass_p0 = p0.getMass();
                double dMass_p1 = p1.getMass();
                double dMass_p2 = p2.getMass();

                Eigen::Vector2d p0NewVel, p1NewVel, p2NewVel, tmpVel;

                // First do (0+2) to 1
                // Find velocity of combined momentums of 0 and 2
                double dMass_p0p2 = dMass_p0 + dMass_p2;
                Eigen::Vector2d p0p2Vel = ((p0.getVelocity() * dMass_p0) + (p2.getVelocity() * dMass_p2)) / (dMass_p0p2);
                // Create new object
                planet tmp_p0p2(1.0, _dMassDensity, false);
                tmp_p0p2.setPosition(p0.getPosition());
                tmp_p0p2.setVelocity(p0p2Vel);
                tmp_p0p2.setMass(dMass_p0p2);
                // Perform collision
                twoBodyCollision(tmp_p0p2, p1, tmpVel, p1NewVel);

                // Second do (0+1) to 2
                // Find velocity of combined momentums of 0 and 1
                double dMass_p0p1 = dMass_p0 + dMass_p1;
                Eigen::Vector2d p0p1Vel = ((p0.getVelocity() * dMass_p0) + (p1.getVelocity() * dMass_p1)) / (dMass_p0p1);
                // Create new object
                planet tmp_p0p1(1.0, _dMassDensity, false);
                tmp_p0p1.setPosition(p0.getPosition());
                tmp_p0p1.setVelocity(p0p1Vel);
                tmp_p0p1.setMass(dMass_p0p1);
                // Perform collision
                twoBodyCollision(tmp_p0p1, p2, tmpVel, p2NewVel);

                // Finally calculate impact on 0
                Eigen::Vector2d p0NewVel_a, p0NewVel_b;
                twoBodyCollision(p0, p1, p0NewVel_a, tmpVel);
                twoBodyCollision(p0, p2, p0NewVel_b, tmpVel);
    
                _vecPlanets[i].setVelocity(p0NewVel_a + p0NewVel_b - p0.getVelocity());
                _vecPlanets[vCollisions[0]].setVelocity(p1NewVel);
                _vecPlanets[vCollisions[1]].setVelocity(p2NewVel);

                _vecPlanets[i].setNeedsUpdate(true);
                _vecPlanets[vCollisions[0]].setNeedsUpdate(true);
                _vecPlanets[vCollisions[1]].setNeedsUpdate(true);

                _vecPlanets[i]._vecCurrentCollisions.clear();
                _vecPlanets[vCollisions[0]]._vecCurrentCollisions.clear();
                _vecPlanets[vCollisions[1]]._vecCurrentCollisions.clear();
            }

            if(vCollisions.size() == 3)
            {
                std::cout << "Four body collision!\n";
            }
            if(vCollisions.size() == 4)
            {
                std::cout << "Five body collision!\n";
            }

            
        }
    }
    // _vecPlanets.insert(_vecPlanets.end(), newPlanets.begin(), newPlanets.end());
    // _nPlanets = _vecPlanets.size();
}

void solarSystem::twoBodyCollision(planet p0, planet p1, Eigen::Vector2d &p0NewVel, Eigen::Vector2d &p1NewVel)
{
    Eigen::Vector2d p0Pos = p0.getPosition();
    Eigen::Vector2d p1Pos = p1.getPosition();
    double p0Mass = p0.getMass();
    double p1Mass = p1.getMass();
    Eigen::Vector2d p0Vel = p0.getVelocity();
    Eigen::Vector2d p1Vel = p1.getVelocity();

    Eigen::Vector2d p0p1Displacement = (p0Pos - p1Pos).normalized();
    Eigen::Vector2d p1p0Displacement = - p0p1Displacement;

    p0NewVel = p0Vel - (2.*p1Mass/(p0Mass+p1Mass)) *
    (p0Vel - p1Vel).dot(p0p1Displacement) * p0p1Displacement;
    p1NewVel = p1Vel - (2.*p0Mass/(p0Mass+p1Mass)) *
    (p1Vel - p0Vel).dot(p1p0Displacement) * p1p0Displacement;
}

void solarSystem::update()
{
    updatePositionVelocity();
    checkCollisions();
   
}