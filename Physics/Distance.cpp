#include "Distance.h"

using namespace King;

Distance King::operator*(const Velocity &velIn, const UnitOfMeasure::Time &t) 
{ 
    return Distance(velIn.Get_magnitude() * t, velIn.Get_unit_direction()); 
}
Distance King::operator*(const UnitOfMeasure::Time &t, const Velocity &velIn) 
{ 
    return Distance(velIn.Get_magnitude() * t, velIn.Get_unit_direction()); 
}




