#include "include/astroObjectBase.h"
#include <Eigen/Core>
// #include <Eigen/Dense>
#include <vector>
class solarSystem
{
    public:
        // solarSystem();
        bool init(double dInitPlanetMass, double dMassDensity);
        bool addObject(int nId, double dMass, Eigen::Vector2d position, Eigen::Vector2d velocity, double dMassDensity);
        bool reset();
        bool centre();
        bool removeObject(const int nId);
        bool update();


    private:
        bool checkCollisions();
        bool removeObjects();

        int _nPlanetIdx = 0;
        double _dRestCoef = 1.0, _dInitPlanetMass, _dInitMassDensity;
        Eigen::MatrixXd objectDistances; 
        std::vector<AstroObjectBase*> _vecObjects;
        std::vector<std::vector<AstroObjectBase*>::iterator> _vecToBeRemoved;
};