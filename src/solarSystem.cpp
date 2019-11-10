#include "include/planet.h"
#include "include/solarSystem.h"
#include <iostream>
#include <Eigen/Dense>


bool solarSystem::init(double dInitPlanetMass, double dInitMassDensity)
{
    _dInitPlanetMass = dInitPlanetMass;
    _dInitMassDensity = dInitMassDensity;
    addObject(dInitPlanetMass, Eigen::Vector2d(0., 0.),  Eigen::Vector2d(0., 0.), dInitMassDensity);
    // addObject(dInitPlanetMass/3., Eigen::Vector2d(100., 100.),  Eigen::Vector2d(-2000.,-2000.), dInitMassDensity);
    // addObject(100., Eigen::Vector2d(-100., -100.),  Eigen::Vector2d(600., 600.), dInitMassDensity);
    // addObject(100., Eigen::Vector2d(-100., 0.),  Eigen::Vector2d(0., 1000.), dInitMassDensity);
    // addObject(100., Eigen::Vector2d(100., 0.),  Eigen::Vector2d(0., -1000.), dInitMassDensity);
}

bool solarSystem::addObject(double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, double dMassDensity)
{
    planet *p = new planet(_nPlanetIdx++, dMass, position, velocity, _dRestCoef, dMassDensity, _vecObjects);
    _vecObjects.emplace_back(p);
    std::cout << "Added planet with ID " << p->getId() << " at " << p << " radius " << p->getRadius() << std::endl;
}

bool solarSystem::reset()
{
    _vecObjects.clear();
    init(_dInitPlanetMass, _dInitMassDensity);
}

bool solarSystem::centre()
{
    AstroObjectBase *pObjInit = _vecObjects[0];
    Eigen::Vector2d initPlanetCentre = pObjInit->getPosition();
    pObjInit->setPosition(Eigen::Vector2d(0.,0.));
    for (int i = 1; i < _vecObjects.size(); ++i)
    {
        AstroObjectBase *pObj = _vecObjects[i];
        pObj->setPosition(pObj->getPosition() - initPlanetCentre);
    }

    return true;
}

bool solarSystem::removeObject(const int nId)
{
    auto it = std::find_if(_vecObjects.begin(),
                        _vecObjects.end(),
                        [nId] (AstroObjectBase* pObj) 
                        {
                            return nId == pObj->getId();
                        });
    if (it == _vecObjects.end())
    {
        std::cout << "ERROR! Could not find id " << nId << " to be removed." << std::endl;
        return false;
    }
    else
    {
        _vecToBeRemoved.push_back(it);
        return true;
    }
}

bool solarSystem::removeObjects()
{
    for (int i = 0; i < _vecToBeRemoved.size(); ++i)
    {
        auto it = _vecToBeRemoved[i];
        delete(*it);
        _vecObjects.erase(it);
    }
}

bool solarSystem::update()
{
    int nObjects = _vecObjects.size();
    bool bRv = checkCollisions();
    for (int i = 0; i < nObjects; ++i)
    {      
        planet *pPlanet = static_cast<planet*>(_vecObjects[i]);
        if(!pPlanet)
        {
            std::cout << "pPlanet null, exit" << std::endl;
            exit(1);
        }
        bRv = _vecObjects[i]->updateForces();
    }
     for (int i = 0; i < nObjects; ++i)
    {      
        planet *pPlanet = static_cast<planet*>(_vecObjects[i]);
        if(!pPlanet)
        {
            std::cout << "pPlanet null, exit" << std::endl;
            exit(1);
        }
        bRv = _vecObjects[i]->updatePositionVelocity();
    }
}

bool solarSystem::checkCollisions()
{
    bool bCollision = false;
    const int nObjects = _vecObjects.size();
    for (int i = 0; i < nObjects; ++i)
    {   
        planet *pPlanet0 = dynamic_cast<planet*>(_vecObjects[i]);
        for (int j = 0; j < nObjects; ++j)
        {
            if (i == j)
            {
                continue;
            }
            planet *pPlanet1 = dynamic_cast<planet*>(_vecObjects[j]);
            if (pPlanet1->collidesWith(pPlanet0))
            {
                // Already done
                continue;
            }
            double dDistance = pPlanet0->getDistanceBetween(pPlanet1);
            double dRadSum = pPlanet1->getRadius() + pPlanet0->getRadius();
            if (dDistance <= dRadSum)
            {
                bCollision = true;
                pPlanet0->addCollision(pPlanet1);
                pPlanet1->addCollision(pPlanet0);
            }
        }
    }
    return bCollision;
}

bool solarSystem::getGlVertices(std::vector<std::vector<Eigen::Vector2d>> &vecVertices)
{
    for (int i = 0; i < _vecObjects.size(); ++i)
    {
        std::vector<Eigen::Vector2d> colVertices;
        planet *pPlanet = dynamic_cast<planet*>(_vecObjects[i]);
        if (nullptr != pPlanet)      // Not entirely sure this is going to work for differentiating types
        {
            // Planet
            double dRadius = pPlanet->getRadius();
            Eigen::Vector2d pos = pPlanet->getPosition();
            for(float arc = 0; arc < 2 * CONST_PI; arc += 0.5)
            {
                Eigen::Vector2d coord;
                coord(0) = dRadius*(cos(arc)) + pos[0];
                coord(1) = dRadius*(sin(arc)) + pos[1];
                colVertices.push_back(coord);
            }
        }
        vecVertices.push_back(colVertices);        
    }
}