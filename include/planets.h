#include <Eigen/Core>
#include <vector>

#include <ctime>
#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

class planet{
    public:
        planet(double r);
        void setRadius(double r);
        void setMass(double m);
        void setPosition(Eigen::Vector2d s);
        void setVelocity(Eigen::Vector2d v);

        double getRadius(){return dRadius;};
        double getMass(){return dMass;};
        Eigen::Vector2d getPosition(){return position;};
        Eigen::Vector2d getVelocity(){return velocity;};
        

    private:
        double dRadius, dMass;
        Eigen::Vector2d position, velocity;
};

const double _dHalfWindowWidth = 640.0, _dHalfWindowHeight = 480.0, _dMassDensity = 1000.0;
// std::chrono::time_point _initTime;
std::chrono::system_clock::time_point _initTime;
std::vector<planet> solarSystem;
Eigen::Vector2d _initXY;
