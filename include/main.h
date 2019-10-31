#include "include/solarSystem.h"

#include <vector>
#include <ctime>
#include <chrono>

// #include "include/planet.h"

typedef std::chrono::high_resolution_clock Clock;
const double _dHalfWindowWidth = 640.0, _dHalfWindowHeight = 480.0;
double _dMassDensity = 1000.0;
std::chrono::system_clock::time_point _initTime;
solarSystem _solarSystem(1000.0);
Eigen::Vector2d _initXY;