#include <Eigen/Core>

#define CONST_PI 3.14159265358979

class AstroObjectBase 
{
    public:
        void setMass(const double m);
        void setPosition(const Eigen::Vector2d s);
        void setVelocity(const Eigen::Vector2d v);
        void setFixed(bool bFixed);
        void setNeedsUpdate(bool bUpdate);

        double getMass(){return _dMass;};
        Eigen::Vector2d getPosition(){return _position;};
        Eigen::Vector2d getVelocity(){return _velocity;};
        bool needsUpdate(){return _bNeedUpdate;};
        bool isFixed(){return _bFixed;};

    protected:
        double  _dMass;
        bool _bNeedUpdate, _bFixed = false;
        Eigen::Vector2d _position, _velocity;
};