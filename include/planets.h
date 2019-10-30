#include <Eigen/Core>
#include <vector>

#include <ctime>
#include <chrono>

#define CONST_PI 3.14159265358979

class AstroObjectBase{
    protected:
        Eigen::Vector2d position, velocity;
        double dMass;
        
    public:
        void setMass(const double m);
        void setPosition(const Eigen::Vector2d s);
        void setVelocity(const Eigen::Vector2d v);

        double getMass();
        Eigen::Vector2d getPosition();
        Eigen::Vector2d getVelocity();
};

class planet : public AstroObjectBase{
    public:
        planet(const double r, const Eigen::Vector2d v);

        void setMass(const double m);
        void setPosition(const Eigen::Vector2d s);
        void setVelocity(const Eigen::Vector2d v);
        void setRadius(const double r);

        double getMass(){return dMass;};
        Eigen::Vector2d getPosition(){return position;};
        Eigen::Vector2d getVelocity(){return velocity;};
        double getRadius(){return dRadius;};

    private:
        double dRadius;
};

typedef std::chrono::high_resolution_clock Clock;

const double _dHalfWindowWidth = 640.0, _dHalfWindowHeight = 480.0, _dMassDensity = 1000.0;
std::chrono::system_clock::time_point _initTime;
std::vector<planet> _solarSystem;
Eigen::Vector2d _initXY;
