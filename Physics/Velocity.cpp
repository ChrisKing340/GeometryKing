#include "Velocity.h"

using namespace King;

Velocity King::operator*(const UnitOfMeasure::Time & t, const Acceleration & accIn)
{
    return Velocity(static_cast<float>(accIn.Get_magnitude()) * static_cast<float>(t), accIn.Get_unit_direction());
}

Velocity King::operator*(const Acceleration & accIn, const UnitOfMeasure::Time & t)
{
    return Velocity(static_cast<float>(accIn.Get_magnitude()) * static_cast<float>(t), accIn.Get_unit_direction());
}

