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
        std::vector<AstroObjectBase*> _vecCurrentCollisions;     // Container will tell add collisions
        bool addCollision(AstroObjectBase *pObj1);
        bool update();
        int getId();
        double getDistanceBetween(AstroObjectBase *pObj1);

        bool setPosition(Eigen::Vector2d position);         // Dont really want to do this but need it for centering
        bool collidesWith(AstroObjectBase *pObj1);

    protected:
        bool _bNeedUpdate = false, _bFixed = false;
    
    private:
        virtual bool calculateForceCollisions() = 0;
    
        bool updatePositionVelocity();
        bool calculateForceGravity();

        int _nId;
        double _dMass, _dMassDensity, _dRestCoef = 1.0;
        std::vector<AstroObjectBase*> *_pVecObjects, _vecCollisions;
        std::vector<int> _vecDontInteractIds;
        Eigen::Vector2d _position, _velocity, _force, _momentum;
};

#endif