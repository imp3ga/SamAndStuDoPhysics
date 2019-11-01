#include <Eigen/Core>

#define CONST_PI 3.14159265358979

class AstroObjectBase 
{
    public:
        void setMass(const double m);
        void setPosition(const Eigen::Vector2d s);
        void setVelocity(const Eigen::Vector2d v);

        double getMass(){return dMass;};
        Eigen::Vector2d getPosition(){return position;};
        Eigen::Vector2d getVelocity(){return velocity;};
        bool _bNeedUpdate = false;
        bool _bFixed = false;

    protected:
        double  dMass;
        Eigen::Vector2d position, velocity;
};