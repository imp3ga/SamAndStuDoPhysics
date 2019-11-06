#include "include/solarSystem.h"
#include <iostream>
#include <Eigen/Dense>

solarSystem::solarSystem(double rho)
{
    _dMassDensity = rho;
}

void solarSystem::init(double initPlanetRad)
{
    // Add the initial large planet

    // Causes the sticking inside thing:
    // addPlanet(Eigen::Vector2d(0.,0.), Eigen::Vector2d(0.,0.), 100.);
    // addPlanet(Eigen::Vector2d(-200.,125.), Eigen::Vector2d(10000.,0.), 20.);
    // addPlanet(Eigen::Vector2d(200.,140.), Eigen::Vector2d(-10000.,0.), 20.);

    // Causes inside collision when break is being used:
    addPlanet(Eigen::Vector2d(0.,0.), Eigen::Vector2d(0.,0.), 100.);
    addPlanet(Eigen::Vector2d(500.,0.), Eigen::Vector2d(0.,0.), 20.);
    // addPlanet(Eigen::Vector2d(200.,200.), Eigen::Vector2d(0.,0.), 20.);
    // addPlanet(Eigen::Vector2d(200.,-200.), Eigen::Vector2d(0.,0.), 20.);


    // addPlanet(Eigen::Vector2d(0.,0.), Eigen::Vector2d(0.,0.), 100.);
    // addPlanet(Eigen::Vector2d(500.,500.), Eigen::Vector2d(-300.,-300.), 20.);
    // addPlanet(Eigen::Vector2d(-300.,300.), Eigen::Vector2d(200.,-200.), 20.);
    // addPlanet(Eigen::Vector2d(350.,-350.), Eigen::Vector2d(-230.,230.), 20.);
    // addPlanet(Eigen::Vector2d(-500.,0.), Eigen::Vector2d(30000.,0.), 50.);

    // addPlanet(Eigen::Vector2d(-640.,480.), Eigen::Vector2d(0., 0.), 50.);


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
    for (int i = 1; i < _vecPlanets.size(); ++i)
    {
        planet &p0 = _vecPlanets[i];
        _vecPlanets[i].setPosition(p0.getPosition() - initPlanetCentre);
    }
}

void solarSystem::getInfo(std::vector<Eigen::Vector2d> &allCentres,
                                std::vector<double> &allRadii)
{
    for (int i = 0; i < _vecPlanets.size(); ++i)
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

    newPlanet.nIdx = _nPlanetIdx++;

    _vecPlanets.push_back(newPlanet);
}

void solarSystem::addPlanet(planet p0)
{
    _vecPlanets.push_back(p0);
}

// Get the location of the planet in _vecPlanets by the planet's ID
int solarSystem::findByIdx(int idx)
{
    if(_vecPlanets.size() == 0)
    {
        std::cout << "_vecPlanets is empty!!" << std::endl;
    }
    for(int i = 0; i < _vecPlanets.size() ; ++i)
    {
        if(_vecPlanets[i].nIdx == idx)
        {
            return i;
        }
    }   
    return -1;
    // Need another return!! ///!!!!
}

void solarSystem::removePlanets()
{
    int nSize = _vecPlanets.size();
    for(int i = 0; i < nSize; ++i)
    {
        if(_vecPlanets[i].toBeRemoved())
        {
            std::cout << "Removing planets...\n";
            _vecPlanets.erase(_vecPlanets.begin() + i);
            --i;
            --nSize;
        }
    }

    // int nSize = _vecRemovePlanets.size();
    // for(int i = 0; i < nSize; ++i)
    // {
    //     std::cout << "Removing planets...\n";
    //     int nIdx = _vecRemovePlanets[i].nIdx;
    //     int nId = findByIdx(nIdx);
    //     if(nId >= 0)
    //     {
    //         _vecPlanets.erase(_vecPlanets.begin() + findByIdx(nIdx));
    //     }
    // }

    // for(auto it = _vecPlanets.begin(); it != _vecPlanets.end();)
    // {
    //     if(_vecPlanets[i].nIdx == nIdx)
    //     {
    //         it = _vecPlanets.erase(it);
    //         std::cout << "Erased planet with ID " << nIdx << std::endl;
    //     }
    //     else
    //     {
    //         ++it;
    //     }
    // }
}
void solarSystem::removePlanet(planet p0)
{
    _vecRemovePlanets.push_back(p0);
}

void solarSystem::addCustomPlanet(Eigen::Vector2d pos,
                            Eigen::Vector2d vel,
                            double r, double rho, bool fixed)
{
    planet newPlanet(r, rho, fixed);
    newPlanet.setPosition(pos);
    newPlanet.setVelocity(vel);

    _vecPlanets.push_back(newPlanet);
}

void solarSystem::updatePositionVelocity()
{
    std::vector<Eigen::Vector2d> vecForces;
    for (int i = 0; i < _vecPlanets.size(); ++i)
    {
        planet &p0 = _vecPlanets[i];
        // if (p0.needsUpdate())
        // {
        //     continue;
        // }
        Eigen::Vector2d p0Pos = p0.getPosition();
        double p0Mass = p0.getMass();
        Eigen::Vector2d force(0., 0.);
        for (int j = 0; j < _vecPlanets.size(); ++j)
        {
            if(i == j)
            {
                continue;
            }
            planet &p1 = _vecPlanets[j];
            bool bFragment = false;
            for(int k = 0; k < p0._vecDontCollideWithIds.size(); ++k)
            {
                if(p1.nIdx == p0._vecDontCollideWithIds[k])
                {
                    // std::cout << "nId " << p1.nIdx << " should not gravitationally interact with " << p0.nIdx << "\n";
                    bFragment = true;
                }
            }
            if(bFragment)
            {
                continue;
            }
            Eigen::Vector2d p1Pos = p1.getPosition();
            double p1Mass = p1.getMass();
            // if (p1.needsUpdate())
            // {
            //     continue;
            // }
            
            double dForce = 1E3 * p0Mass * p1Mass / pow((p1Pos - p0Pos).norm(), 2.0);
            std::cout << dForce << std::endl;

            // Eigen::Vector2d invRsqrd = (p1Pos - p0Pos).cwiseInverse();
            // force += p0Mass * p1Mass * invRsqrd;

            // double dForce = p0Mass * p1Mass / (p1Pos - p0Pos).squaredNorm();
            // std::cout << (p0Mass * p1Mass * invRsqrd).transpose() << std::endl;
            // std::cout << "planet with idx " << p0.nIdx << " and mass " << p0Mass << " at position " << p0Pos.transpose() << " with planet with idx " << p1.nIdx << " and mass " << p1Mass << " at position " << p1Pos.transpose() << " creates a force of size " << (p0Mass * p1Mass * invRsqrd).transpose() << std::endl;
            force += dForce * (p1Pos - p0Pos).normalized();
        }
        // std::cout << "Total force on planet with id " << p0.nIdx << " is " << force.transpose() << std::endl; 
        vecForces.push_back(force);
    }

    for (int i = 0; i < _vecPlanets.size(); ++i)
        {
            planet &planet = _vecPlanets[i];
            if(!planet.isFixed())
            {   
                // Update velocity
                double mass = planet.getMass();
                Eigen::Vector2d acc = vecForces[i] / mass;
                Eigen::Vector2d newVel = planet.getVelocity() + (0.001 * acc);      // Physics update is 1ms
                planet.setVelocity(newVel);
                // Update position
                Eigen::Vector2d newPos = planet.getPosition() + (0.001 * newVel);   // Physics update is 1ms
                _vecPlanets[i].setPosition(newPos);
                _vecPlanets[i].setNeedsUpdate(false);
            }
        }
    // std::cout << "FINISHED UPDATING!\n\n" << std::endl;
}

bool solarSystem::checkCollisions()
{
    bool bCollision = false;
    std::vector<planet> newPlanets;
    for (int i = 0; i < _vecPlanets.size(); ++i)
    {
        planet &p0 = _vecPlanets[i];
        Eigen::Vector2d p0Pos = p0.getPosition();
        double p0Mass = p0.getMass();

        for (int j = 0; j < _vecPlanets.size(); ++j)
        {
            if(i == j)
            {
                continue;
            }
            planet &p1 = _vecPlanets[j];
            bool bFragment = false;
            int id = 0;
            for(int k = 0; k < p0._vecDontCollideWithIds.size(); ++k)
            {
                if(p1.nIdx == p0._vecDontCollideWithIds[k])
                {
                    // std::cout << "nId " << p1.nIdx << " should not collide with " << p0.nIdx << "\n";
                    bFragment = true;
                    id = k;
                }
            }
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
            if (!bFragment && dist < p0.getRadius() + p1.getRadius())
            {
                // std::cout << "Collision!\n";
                bCollision = true;
                p0._vecCurrentCollisions.push_back(j); 
                p1._vecCurrentCollisions.push_back(i);
            }
            else if (bFragment && dist > p0.getRadius() + p1.getRadius())
            {
                // p0._vecDontCollideWithIds.erase(p0._vecDontCollideWithIds.begin() + id);
                p0._vecDontCollideWithIds.clear();
                p1._vecDontCollideWithIds.clear();
                p0.nIdx = _nPlanetIdx++;
                p1.nIdx = _nPlanetIdx++;
                // auto pnId  = std::find(p1._vecDontCollideWithIds.begin(),
                //                  p1._vecDontCollideWithIds.end(), p0.nIdx);
                // if(pnId == p1._vecDontCollideWithIds.end())
                // {
                //     std::cout << "ERROR! Could NOT find p0.nIdx in p1._vecDontCollideWithIds.end()" << std::endl;
                // }
                // p1._vecDontCollideWithIds.erase(pnId);
            }         
        }
    }
    // for (int i = 0; i < _vecPlanets.size(); ++i)
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
    return bCollision;
}

void solarSystem::resolveCollisions()
{
    // Do collisions
    for (int i = 0; i < _vecPlanets.size(); ++i)
    {
        planet &p0 = _vecPlanets[i];
        if(p0.toBeRemoved())
        {
            continue;
        }
        std::vector<int> &vCollisions = p0._vecCurrentCollisions;
        if(vCollisions.size() > 0)
        {           
            // if(p0.needsUpdate() || _vecPlanets[vCollisions[0]].needsUpdate())
            // {
                // std::cout << "Planet(s) have already been updated!\n";
                // continue;
            // }

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
                if(!bThree && !_vecPlanets[vCollisions[0]].toBeRemoved())
                {
                    // std::cout << "Two body collision!\n";
                    planet &p1 = _vecPlanets[vCollisions[0]];
                    twoBodyCollision(p0, p1, true);
                    continue;
                    }
            }

            // Three body collision
            if(vCollisions.size() == 2 && (!_vecPlanets[vCollisions[0]].toBeRemoved() &&
                                           !_vecPlanets[vCollisions[1]].toBeRemoved()))
            {
                // std::cout << "Three body collision!" << std::endl;
                planet &p1 = _vecPlanets[vCollisions[0]];
                planet &p2 = _vecPlanets[vCollisions[1]];
                if(p1.toBeRemoved())
                {
                    std::cout << "...actually a 2 body collision\n";
                    twoBodyCollision(p0, p2, true);
                    continue;
                }
                if(p2.toBeRemoved())
                {
                    std::cout << "...actually a 2 body collision\n";
                    twoBodyCollision(p0, p1, true);
                    continue;
                }

                double dMass_p0 = p0.getMass();
                double dMass_p1 = p1.getMass();
                double dMass_p2 = p2.getMass();

                Eigen::Vector2d p0NewVel, p1NewVel, p2NewVel, tmpVe, p0Vel;
                p0Vel = p0.getVelocity();

                planet p0_org = p0;
                planet p1_org = p1;
                planet p2_org = p2;

                // First do (0+2) to 1
                // Find velocity of combined momentums of 0 and 2
                double dMass_p0p2 = dMass_p0 + dMass_p2;
                Eigen::Vector2d p0p2Vel = ((p0.getVelocity() * dMass_p0) + (p2.getVelocity() * dMass_p2)) / (dMass_p0p2);
                // Create new object
                planet tmp_p0p2(p0.getRadius(), _dMassDensity, false);
                tmp_p0p2.setPosition(p0.getPosition());
                tmp_p0p2.setVelocity(p0p2Vel);
                tmp_p0p2.setMass(dMass_p0p2);
                // Perform collision
                twoBodyCollision(tmp_p0p2, p1, false);

                // Second do (0+1) to 2
                // Find velocity of combined momentums of 0 and 1
                double dMass_p0p1 = dMass_p0 + dMass_p1;
                Eigen::Vector2d p0p1Vel = ((p0Vel * dMass_p0) + (p1_org.getVelocity() * dMass_p1)) / (dMass_p0p1);
                // Create new object
                planet tmp_p0p1(p0.getRadius(), _dMassDensity, false);
                tmp_p0p1.setPosition(p0.getPosition());
                tmp_p0p1.setVelocity(p0p1Vel);
                tmp_p0p1.setMass(dMass_p0p1);
                // Perform collision
                twoBodyCollision(tmp_p0p1, p2, false);

                // Finally calculate impact on 0
                Eigen::Vector2d p0NewVel_a, p0NewVel_b;
                twoBodyCollision(p0, p1_org, false);
                p0NewVel_a = p0.getVelocity();
                twoBodyCollision(p0_org, p2_org, false);
                p0NewVel_b = p0_org.getVelocity();
    
                _vecPlanets[i].setVelocity(p0NewVel_a + p0NewVel_b - p0Vel);
                _vecPlanets[vCollisions[0]].setVelocity(p1.getVelocity());      ////!!!!!! just refering to itself
                _vecPlanets[vCollisions[1]].setVelocity(p2.getVelocity());

                _vecPlanets[i].setNeedsUpdate(true);
                _vecPlanets[vCollisions[0]].setNeedsUpdate(true);
                _vecPlanets[vCollisions[1]].setNeedsUpdate(true);

                _vecPlanets[i]._vecCurrentCollisions.clear();
                _vecPlanets[vCollisions[0]]._vecCurrentCollisions.clear();
                _vecPlanets[vCollisions[1]]._vecCurrentCollisions.clear();
            }

            if(vCollisions.size() == 3)
            {
                std::cout << "Four body collision!" << std::endl;
            }
            if(vCollisions.size() == 4)
            {
                std::cout << "Five body collision!" << std::endl;
            }
        }
    }
    removePlanets();

    // _vecPlanets.insert(_vecPlanets.end(), newPlanets.begin(), newPlanets.end());
    // _nPlanets = _vecPlanets.size();
}

void solarSystem::resolveRisidualCollisions()
{
    for (int i = 0; i < _vecPlanets.size(); ++i)
    {
        planet &p0 = _vecPlanets[i];
        std::vector<int> &vCollisions = p0._vecCurrentCollisions;
        if(vCollisions.size() > 0)
        {      
            p0.revertPosition();
        }
    }
    resolveCollisions();
}

void solarSystem::twoBodyCollision(planet &p0, planet &p1, bool bUpdate)
{
    Eigen::Vector2d p0Pos = p0.getPosition();
    Eigen::Vector2d p1Pos = p1.getPosition();
    double p0Mass = p0.getMass();
    double p1Mass = p1.getMass();
    Eigen::Vector2d p0Vel = p0.getVelocity();
    Eigen::Vector2d p1Vel = p1.getVelocity();

    Eigen::Vector2d p0p1Displacement = (p0Pos - p1Pos).normalized();
    Eigen::Vector2d p1p0Displacement = - p0p1Displacement;

    Eigen::Vector2d p0NewVel, p1NewVel;
    p0NewVel = p0Vel - (2.*p1Mass/(p0Mass+p1Mass)) *
    (p0Vel - p1Vel).dot(p0p1Displacement) * p0p1Displacement;
    p1NewVel = p1Vel - (2.*p0Mass/(p0Mass+p1Mass)) *
    (p1Vel - p0Vel).dot(p1p0Displacement) * p1p0Displacement;

    if(bUpdate)
    {
        int id0 = findByIdx(p0.nIdx);
        int id1 = findByIdx(p1.nIdx);

        _vecPlanets[id0].setVelocity(p0NewVel);
        _vecPlanets[id1].setVelocity(p1NewVel);
        _vecPlanets[id0].setNeedsUpdate(true);
        _vecPlanets[id1].setNeedsUpdate(true);
        // std::cout << "Planet velocities have been updated.\n";
        _vecPlanets[id0]._vecCurrentCollisions.clear();
        _vecPlanets[id1]._vecCurrentCollisions.clear();
        // std::cout << (p0Vel - p0NewVel).norm() << ", " << (p1Vel - p1NewVel).norm() << std::endl;
        if((p0Vel - p0NewVel).norm() > 9000.0)
        {
            std::cout << "Breaking planet " << id0 << std::endl;
            // breakPlanet(p0);
        }
        if((p1Vel - p1NewVel).norm() > 9000.0)
        {
            std::cout << "Breaking planet " << id1 << std::endl;
            // breakPlanet(p1);
        }
        

    }
    else
    {
        // std::cout << "Not updating velocity\n";
        p0.setVelocity(p0NewVel);
        p1.setVelocity(p1NewVel);
    }
    
}

void solarSystem::update()
{
    updatePositionVelocity();
    if(checkCollisions())
    {
        resolveCollisions();
        updatePositionVelocity();
        if(checkCollisions())
        {
            // std::cout << "Some collisions were not resolved!! Resolving risidual collisions...\n";
            resolveRisidualCollisions();
        }
    }

    // bool bRv = checkCollisions();
    // std::cout << "1) bRv is " << bRv << "\n";
    // resolveCollisions();
    // updatePositionVelocity();
    // bRv = checkCollisions();
    // std::cout << "2) bRv is " << bRv << "\n";
    // while(bRv)
    // {
    //     std::cout << "bRv is " << bRv << ", some collisions were not resolved!! Resolving risidual collisions...\n";
    //     resolveRisidualCollisions();
    //     bRv = checkCollisions();
    // }
}

void solarSystem::breakPlanet(planet p)
{
    Eigen::Vector2d pVel = p.getVelocity();
    Eigen::Vector2d pPos = p.getPosition();
    double pMass = p.getMass();

    double degtorad = 3.14159265358979 / 180.0;
    double theta = 30.0 * degtorad;
    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta),  cos(theta);
    Eigen::Vector2d p0Vel, p1Vel;
    p0Vel = pVel;// 0.5*R*pVel;
    p1Vel = pVel;// 0.5*R.transpose()*pVel;

    double newR = sqrt(pMass/(2.0*_dMassDensity));
    planet p0(newR, _dMassDensity, false), p1(newR, _dMassDensity, false);
    double pRadius = p.getRadius();
    // p0.setPosition(pPos + Eigen::Vector2d(pRadius + newR, newR) );
    // p1.setPosition(pPos + Eigen::Vector2d(pRadius + newR, - newR));
    Eigen::Vector2d u = pVel.normalized();
    Eigen::Vector2d u_n(u[1], u[0]);
    // p0.setPosition(pPos + u*(pRadius + newR) + u_n*newR);
    // p1.setPosition(pPos + u*(pRadius + newR) - u_n*newR);
    // p0.setPosition(pPos + u_n*newR + 5.*u);
    // p1.setPosition(pPos - u_n*newR + 5.*u);
    p0.setPosition(pPos);// - u*(pRadius - newR));
    p1.setPosition(pPos);// - u*(pRadius - newR));
    p0.setVelocity(p0Vel);
    p1.setVelocity(p1Vel);
    p0.setPrevPositionVelocity(Eigen::Vector2d(-500.0, -500.0), p0Vel);
    p1.setPrevPositionVelocity(Eigen::Vector2d(500.0, 500.0), p1Vel);
    p0.nIdx = _nPlanetIdx++;
    p1.nIdx = _nPlanetIdx++;

    p0._vecDontCollideWithIds.push_back(p1.nIdx);
    p1._vecDontCollideWithIds.push_back(p0.nIdx);

    p0.setNeedsUpdate(true);
    p1.setNeedsUpdate(true);

    std::cout << "New fragments: \n" 
              << "p :\n velocity: " << pVel.transpose() << "\n"
              << "radius: " << p.getRadius() << "\n"
              << "position: " << p.getPosition().transpose() << "\n" 
              << "p0 :\n velocity: " << p0Vel.transpose() << "\n"
              << "radius: " << newR << "\n"
              << "position: " << p0.getPosition().transpose() << "\n" 
              << "p1 :\n velocity: " << p1Vel.transpose() << "\n"
              << "radius: " << newR << "\n"
              << "position: " << p1.getPosition().transpose() << "\n" << std::endl; 
    addPlanet(p0);
    addPlanet(p1);
    _vecPlanets[findByIdx(p.nIdx)].setToBeRemoved();
}
