#include <Eigen/Core>
#include <vector>
#define CONST_PI 3.14159265358979

class AstroObjectBase 
{
    public:
        void setMass(const double m);
        void setPosition(const Eigen::Vector2d s);
        void setVelocity(const Eigen::Vector2d v);
        void setFixed(bool bFixed);
        void setNeedsUpdate(bool bUpdate);
        double dampingFactor(){return _dRestCoef;};

        std::vector<int> _vecCurrentCollisions;

        double getMass(){return _dMass;};
        double getKE(){return 0.5 * _dMass * _velocity.squaredNorm();}
        Eigen::Vector2d getPosition(){return _position;};
        Eigen::Vector2d getVelocity(){return _velocity;};
        bool needsUpdate(){return _bNeedUpdate;};
        bool isFixed(){return _bFixed;};

        void revertPosition();
        

    protected:
        double  _dMass, _dRestCoef = 1.0;
        bool _bNeedUpdate, _bFixed = false;
        Eigen::Vector2d _position, _velocity;
        Eigen::Vector2d _positionPrev = Eigen::Vector2d(0., 0.);
        Eigen::Vector2d _velocityPrev = Eigen::Vector2d(0., 0.);

};