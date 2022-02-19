#include "SpringDamperMass.h"

// TODO
// Define your methods here

int SpringDamperMass::step()
{

    Vec2d obj;

    obj.y = (--values.end())->y - ((SPRING_CONST / MASS) * ((--values.end())->x - pos_eqm1)) - ((damping_coeff / MASS) * (--values.end())->y);
    obj.x = (--values.end())->x + obj.y;
    obj.time = (--values.end())->time + 1;

    values.push_back(obj);
    return obj.time;
}
