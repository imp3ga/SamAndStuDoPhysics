#ifndef ASTROOBJBASE_H_
#define ASTROOBJBASE_H_

#include <Eigen/Core>
#include <vector>

#define CONST_PI 3.14159265358979

class AstroObjectBase 
{
    public:
        AstroObjectBase(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
                        double dRestCoef, double dMassDensity, std::vector<AstroObjectBase*> &refVecObjects);
        double getMass();
        double getKE();
        Eigen::Vector2d getPosition();                          // Public so container can work out collisions
        Eigen::Vector2d getVelocity();                          // Could be private?
        // std::vector<AstroObjectBase*> _vecCurrentCollisions;     // Container will tell add collisions
        bool addCollision(AstroObjectBase *pObj1);
        bool removeCollision(AstroObjectBase *pObj1);
        bool updateForces();
        bool addForce(Eigen::Vector2d force);
        int getId();
        double getDistanceBetween(AstroObjectBase *pObj1);

        bool setPosition(Eigen::Vector2d position);         // Dont really want to do this but need it for centering
        bool collidesWith(AstroObjectBase *pObj1);
        bool doesNotInteractWith(int nId);
        bool addDoesNotInteractWith(int nId);
        bool doesNotCollideWith(int nId);
        bool addDoesNotCollideWith(int nId);
        bool removeDoesNotCollideWith(int nId);

    // protected:
        bool _bNeedUpdate = false, _bFixed = false;
        std::vector<AstroObjectBase*> _vecCollisions;
        Eigen::Vector2d _position, _velocity, _force, _momentum;
        std::vector<int> _vecDontInteractIds, _vecDontCollideIds;
        AstroObjectBase* findById(int nId);

    
    // private:
        virtual bool calculateForceCollisions() = 0;
    
        bool updatePositionVelocity();
        bool calculateForceGravity();

        double _G = 1E6;

        int _nId;
        double _dMass, _dMassDensity, _dRestCoef = 1.0;
        std::vector<AstroObjectBase*> *_pVecObjects;
};

#endif