#include "include/planet.h"
// #include "include/solarSystem.h"
#include <Eigen/Core>
#include <vector>
class solarSystem
{
    public:
        solarSystem(double rho) ;
        void init(double initPlanetRad);
        void reset();
        void centre();
        void addPlanet(Eigen::Vector2d pos, Eigen::Vector2d vel, double r);
        void addCustomPlanet(Eigen::Vector2d pos, Eigen::Vector2d vel, double r, double rho, bool fixed);
        void update(void);
        void getInfo(std::vector<Eigen::Vector2d> &allCentres,
                            std::vector<double> &allRadii);
        void twoBodyCollision(planet p0, planet p1, Eigen::Vector2d &p0NewVel, Eigen::Vector2d &p1NewVel);

    private:
        int _nPlanets = 0;
        double _dMassDensity, _initPlanetRad, _dampingFactor = 1.0;//0.95;
        std::vector<planet> _vecPlanets;
        void updatePositionVelocity();
        void checkCollisions();
};