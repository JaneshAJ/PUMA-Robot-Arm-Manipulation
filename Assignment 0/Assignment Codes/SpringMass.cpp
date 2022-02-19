#include "SpringMass.h"

#include <ostream>
#include <fstream>

using namespace std;

// define gravity constant
const double SpringMass::GRAVITY = 10;
const double SpringMass::SPRING_CONST = 7;
const double SpringMass::MASS = 30;

// TODO SpringMass constructor
  
SpringMass::SpringMass(double pos_init, double vel_init, double pos_eqm, double vel_eqm)
{

}
SpringMass::~SpringMass() {
  // TODO clean up stuff from HEAP  
}

// TODO SpringMass simulation step
int SpringMass::step() {
  return -1;
}

// TODO SpringMass configuration getter
bool SpringMass::getConfiguration(int t, Vec2d& state) const {
  return false;
}

// TODO SpringMass current simulation time getter
int SpringMass::getCurrentSimulationTime() const {
  return -1;
}



