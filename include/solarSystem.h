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
        void addPlanet(planet p0);
        void removePlanet(planet p0);
        void removePlanets();
        void addCustomPlanet(Eigen::Vector2d pos, Eigen::Vector2d vel, double r, double rho, bool fixed);
        void update(void);
        void getInfo(std::vector<Eigen::Vector2d> &allCentres,
                            std::vector<double> &allRadii);
        void twoBodyCollision(planet &p0, planet &p1, bool bUpdate);
        void breakPlanet(planet p0);
        int findByIdx(int idx);
    private:
        double _dMassDensity, _initPlanetRad, _dampingFactor = 1.0;//0.95;
        std::vector<planet> _vecPlanets, _vecRemovePlanets;
        void updatePositionVelocity();
        bool checkCollisions();
        void resolveCollisions();
        void resolveRisidualCollisions();

        int _nPlanetIdx = 0;
};