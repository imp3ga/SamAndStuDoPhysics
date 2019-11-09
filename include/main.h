#include "include/solarSystem.h"
#include "include/flowContainer.h"
#include "include/tests.h"

#include <vector>
#include <ctime>
#include <chrono>

// #include "include/planet.h"

typedef std::chrono::high_resolution_clock Clock;
const double _dHalfWindowWidth = 640.0, _dHalfWindowHeight = 480.0;
double _dMassDensity = 1000.0;
std::chrono::system_clock::time_point _initTime;
solarSystem _solarSystem(_dMassDensity);
Eigen::Vector2d _initXY;
enum mode{planetsMode, flowMode};
int mode = flowMode;

double t = 0.0;

//Flow Stuff
void cycleMode();
flowContainer _flowContainer(5000, _dHalfWindowWidth * 2, _dHalfWindowHeight * 2);