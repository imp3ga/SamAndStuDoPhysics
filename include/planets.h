#include <Eigen/Core>
#include <vector>

#include <ctime>
#include <chrono>

#define CONST_PI 3.14159265358979

class planet{
    public:
        planet(const double r, const Eigen::Vector2d v);
        void setRadius(const double r);
        void setMass(const double m);
        void setPosition(const Eigen::Vector2d s);
        void setVelocity(const Eigen::Vector2d v);

        double getRadius(){return dRadius;};
        double getMass(){return dMass;};
        Eigen::Vector2d getPosition(){return position;};
        Eigen::Vector2d getVelocity(){return velocity;};

    private:
        double dRadius, dMass;
        Eigen::Vector2d position, velocity;
};

typedef std::chrono::high_resolution_clock Clock;

const double _dHalfWindowWidth = 640.0, _dHalfWindowHeight = 480.0, _dMassDensity = 1000.0;
std::chrono::system_clock::time_point _initTime;
std::vector<planet> _solarSystem;
Eigen::Vector2d _initXY;
