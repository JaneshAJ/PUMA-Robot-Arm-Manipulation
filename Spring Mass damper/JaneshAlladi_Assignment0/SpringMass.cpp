#include "SpringMass.h"

#include <ostream>
#include <fstream>
#include <vector>

using namespace std;

// define gravity constant
const double SpringMass::GRAVITY = 10;
const double SpringMass::SPRING_CONST = 7;
const double SpringMass::MASS = 30;

// TODO SpringMass constructor
  
SpringMass::SpringMass(double pos_init, double vel_init, double pos_eqm, double vel_eqm)
{

  pos_eqm1 = pos_eqm;
  vel_eqm1 = vel_eqm;

  Vec2d obj;
  obj.time = 0;
  obj.x = pos_init;
  obj.y = vel_init;

  values.push_back(obj);
}

SpringMass::~SpringMass() {
  // TODO clean up stuff from HEAP  
}

// TODO SpringMass simulation step
int SpringMass::step()
{
  Vec2d obj;

  obj.y = (--values.end())->y - ((SPRING_CONST / MASS) * ((--values.end())->x - pos_eqm1));
  obj.x = (--values.end())->x + obj.y;
  obj.time = (--values.end())->time + 1;

  values.push_back(obj);
  return obj.time;
}

// TODO SpringMass configuration getter
bool SpringMass::getConfiguration(int t, Vec2d &state) const
{

  for (auto i = values.begin(); i != values.end(); ++i)
  {
    if (i->time == t)
    {
      state.time = i->time;
      state.x = i->x;
      state.y = i->y;
      return true;
    }
  }

  return false;
}

// TODO SpringMass current simulation time getter
int SpringMass::getCurrentSimulationTime() const
{
  return (--values.end())->time;
}



