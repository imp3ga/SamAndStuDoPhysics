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
        void addPlanet(Eigen::Vector2d pos, Eigen::Vector2d vel, double r);
        void update(void);
        void getInfo(std::vector<Eigen::Vector2d> &allCentres,
                            std::vector<double> &allRadii);
    private:
        int _nPlanets = 0;
        double _dMassDensity, _initPlanetRad;
        std::vector<planet> _vecPlanets;
};