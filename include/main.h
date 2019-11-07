#include "include/solarSystem.h"
// #include "include/tests.h"

#include <vector>
#include <ctime>
#include <chrono>

// #include "include/planet.h"

solarSystem _system;

typedef std::chrono::high_resolution_clock Clock;
const double _dHalfWindowWidth = 640.0, _dHalfWindowHeight = 480.0;
double _dInitMassDensity = 1.0, _dMouseDownSensitivity = 5000.0, _dMouseMoveSensitivity = 5.0;
std::chrono::system_clock::time_point _initTime;
Eigen::Vector2d _initXY;