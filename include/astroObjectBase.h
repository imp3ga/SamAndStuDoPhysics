#ifndef ASTROOBJBASE_H_
#define ASTROOBJBASE_H_

#include <Eigen/Core>
#include <vector>

#define CONST_PI 3.14159265358979

class AstroObjectBase 
{
    public:
        
        //Constructors
        
        AstroObjectBase(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, 
                        double dRestCoef, double dMassDensity);
        
        //Getters

        double getMass();
        double getKE();
        Eigen::Vector2d getPosition();                          // Public so container can work out collisions
        Eigen::Vector2d getVelocity();                          // Could be private?
        std::vector<AstroObjectBase*> _vecCurrentCollisions;     // Container will tell add collisions
        int getId();
        double getDistanceBetween(AstroObjectBase *pObj1);
        std::vector<int> getDontInteractIDs();

        //Setters
        
        bool setPosition(Eigen::Vector2d position);         // Dont really want to do this but need it for centering
        bool hasAlreadyInteracted(std::vector<int> _otherDontInteractIds);
        void setAlreadyInteracted(bool value);

         //Methods
        
        bool addCollision(AstroObjectBase *pObj1);
        bool update(std::vector<AstroObjectBase*> *refVecObjects);
        bool collidesWith(AstroObjectBase *pObj1);
        
    protected:

        int _nId;
        Eigen::Vector2d _position, _velocity, _force, _momentum;
        

    private:

        bool _bNeedUpdate = false, _bFixed = false, _alreadyInteracted = false;
        double _dMass, _dMassDensity, _dRestCoef = 1.0;
        std::vector<int> _vecDontInteractIds;
        std::vector<AstroObjectBase*> _vecCollisions;
        bool calculateForceGravity(std::vector<AstroObjectBase*> *refVecObjects);
        virtual bool calculateForceCollisions(std::vector<AstroObjectBase*> *refVecObjects) = 0;
        bool updatePositionVelocity();
    

       
};

#endif