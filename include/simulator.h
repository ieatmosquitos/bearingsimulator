#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "SFML/Window.hpp"
#include "SFML/Graphics.hpp"

#include <iostream>
#include <fstream>

struct Landmark{
  double x;
  double y;
  
  Landmark(double x, double y) {this->x = x; this->y = y;};
};

struct RobotPose{
  double x;
  double y;
  double theta;
};

#endif  // SIMULATOR_H
