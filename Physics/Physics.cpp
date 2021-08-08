#include "Physics.h"

using namespace King;
using namespace UnitOfMeasure;
using namespace Physics;

/******************************************************************************
*   Force
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Force& in) { return os << "{" << " Dir: " << in.Get_unit_direction() << " Mag: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Force& in) { return os << L"{" << L" Dir: " << in.Get_unit_direction() << L" Mag: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Force& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Force& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Force& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Force& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction());}
// operators
// methods
// functions
/******************************************************************************
*   Torque
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Torque& in) { return os << "{" << " Dir: " << in.Get_unit_direction() << " Mag: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Torque& in) { return os << L"{" << L" Dir: " << in.Get_unit_direction() << L" Mag: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Torque& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Torque& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Torque& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Torque& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
// methods
// functions
/******************************************************************************
*   Acceleration
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Acceleration& in) { return os << "{" << " Dir: " << in.Get_unit_direction() << " Mag: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Acceleration& in) { return os << L"{" << L" Dir: " << in.Get_unit_direction() << L" Mag: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Acceleration& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Acceleration& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Acceleration& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Acceleration& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
Acceleration King::operator/(const King::Force& f, const UnitOfMeasure::Mass& m)
{
    return Acceleration(static_cast<float>(f.Get_magnitude()) / static_cast<float>(m), f.Get_unit_direction());
}
// methods
// functions
/******************************************************************************
*   AngularAcceleration
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const AngularAcceleration& in) { return os << "{" << " EulerXYZ: " << in.Get_unit_direction() << " AngleMag: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const AngularAcceleration& in) { return os << L"{" << L" EulerXYZ: " << in.Get_unit_direction() << L" AngleMag: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, AngularAcceleration& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, AngularAcceleration& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const AngularAcceleration& from) { j = json{ {"AngleMag", from.Get_magnitude()}, {"EulerXYZ", from.Get_unit_direction()} }; }
void King::from_json(const json& j, AngularAcceleration& to) { j.at("AngleMag").get_to(to.Get_magnitude()); j.at("EulerXYZ").get_to(to.Get_unit_direction()); }
// operators
AngularAcceleration King::operator/(const Torque& tIn, const UnitOfMeasure::Inertia& inertiaIn)
{
    return AngularAcceleration(tIn / static_cast<float>(inertiaIn));
}
Torque King::operator*(const UnitOfMeasure::Inertia& inertiaIn, const AngularAcceleration& angAccelIn)
{
    return Torque(static_cast<float3>(angAccelIn) * static_cast<float>(inertiaIn));
}
Torque King::operator*(const AngularAcceleration& angAccelIn, const UnitOfMeasure::Inertia& inertiaIn)
{
    return Torque(static_cast<float3>(angAccelIn)* static_cast<float>(inertiaIn));
}
// methods
// functions
/******************************************************************************
*   AngularVelocity
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const AngularVelocity& in) { return os << "{" << " EulerXYZ: " << in.Get_unit_direction() << " EulerXYZ: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const AngularVelocity& in) { return os << L"{" << L" EulerXYZ: " << in.Get_unit_direction() << L" EulerXYZ: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, AngularVelocity& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, AngularVelocity& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const AngularVelocity& from) { j = json{ {"AngleMag", from.Get_magnitude()}, {"EulerXYZ", from.Get_unit_direction()} }; }
void King::from_json(const json& j, AngularVelocity& to) { j.at("AngleMag").get_to(to.Get_magnitude()); j.at("EulerXYZ").get_to(to.Get_unit_direction()); }
// operators
AngularVelocity King::operator*(const UnitOfMeasure::Time& t, const AngularAcceleration& accIn)
{
    return AngularVelocity(static_cast<float>(accIn.Get_magnitude())* static_cast<float>(t), accIn.Get_unit_direction());
}

AngularVelocity King::operator*(const AngularAcceleration& accIn, const UnitOfMeasure::Time& t)
{
    return AngularVelocity(static_cast<float>(accIn.Get_magnitude())* static_cast<float>(t), accIn.Get_unit_direction());
}
Acceleration King::operator*(const Distance& rIn, const AngularVelocity& angularVelIn)
{
    // normal acceleration is along r
    auto dir = -rIn.Get_unit_direction();

    auto mag = angularVelIn.Get_magnitude();
    auto magSq = mag * mag; // rad^2/s^2

    Acceleration an(static_cast<float>(magSq) * rIn.Get_magnitude(), dir ); // m/s^2
    return an;
}
Acceleration King::operator*(const AngularVelocity& angularVelIn, const Distance& rIn)
{
    // normal acceleration is along r
    auto dir = -rIn.Get_unit_direction();

    auto mag = angularVelIn.Get_magnitude();
    auto magSq = mag * mag; // rad^2/s^2

    Acceleration an(static_cast<float>(magSq)* rIn.Get_magnitude(), dir); // m/s^2
    return an;
}
// methods
// functions
/******************************************************************************
*   Velocity
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Velocity& in) { return os << "{" << " Dir: " << in.Get_unit_direction() << " Mag: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Velocity& in) { return os << L"{" << L" Dir: " << in.Get_unit_direction() << L" Mag: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Velocity& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Velocity& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Velocity& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Velocity& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
Velocity King::operator*(const UnitOfMeasure::Time & t, const Acceleration & accIn)
{
    return Velocity(static_cast<float>(accIn.Get_magnitude()) * static_cast<float>(t), accIn.Get_unit_direction());
}

Velocity King::operator*(const Acceleration & accIn, const UnitOfMeasure::Time & t)
{
    return Velocity(static_cast<float>(accIn.Get_magnitude()) * static_cast<float>(t), accIn.Get_unit_direction());
}
// methods
// functions
/******************************************************************************
*   Distance
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Distance& in) { return os << "{" << " Dir: " << in.Get_unit_direction() << " Mag: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Distance& in) { return os << L"{" << L" Dir: " << in.Get_unit_direction() << L" Mag: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Distance& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Distance& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Distance& from) { j = json{ {"Mag", from.Get_magnitude()}, {"Dir", from.Get_unit_direction()} }; }
void King::from_json(const json& j, Distance& to) { j.at("Mag").get_to(to.Get_magnitude()); j.at("Dir").get_to(to.Get_unit_direction()); }
// operators
Distance King::operator*(const Velocity& velIn, const UnitOfMeasure::Time& t)
{
    return Distance(velIn.Get_magnitude() * t, velIn.Get_unit_direction());
}
Distance King::operator*(const UnitOfMeasure::Time& t, const Velocity& velIn)
{
    return Distance(velIn.Get_magnitude() * t, velIn.Get_unit_direction());
}
Distance King::operator*(const King::Quaternion& qIn, const King::Distance& dIn)
{
    auto dir = qIn * dIn.Get_unit_direction().GetVecConst();
    return Distance(dIn.Get_magnitude(), dir);
}
Distance King::operator*(const King::Distance& dIn, const King::Quaternion& qIn)
{
    auto dir = FloatPoint3(DirectX::XMVector3InverseRotate(dIn.Get_unit_direction(), qIn.GetVecConst()));
    return Distance(dIn.Get_magnitude(), dir);
}
UnitOfMeasure::Energy King::operator*(const Force& fIn, const Distance& dIn)
{
    UnitOfMeasure::Energy e(fIn.Get_magnitude() * fIn.Get_magnitude()); // N * m = J
    return e;
}
UnitOfMeasure::Energy King::operator*(const Distance& dIn, const Force& fIn)
{
    UnitOfMeasure::Energy e(fIn.Get_magnitude() * fIn.Get_magnitude()); // N * m = J
    return e;
}
// methods
// functions
/******************************************************************************
*   Rotation
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Rotation& in) { return os << "{" << " EulerXYZ: " << in.Get_unit_direction() << " AngleMag: " << in.Get_magnitude() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Rotation& in) { return os << L"{" << L" EulerXYZ: " << in.Get_unit_direction() << L" AngleMag: " << in.Get_magnitude() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Rotation& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
std::wistream& King::operator>> (std::wistream& is, Rotation& out) { return is >> out._magnitude >> out._unit_direction; } // binary in
// json
void King::to_json(json& j, const Rotation& from) { j = json{ {"AngleMag", from.Get_magnitude()}, {"EulerXYZ", from.Get_unit_direction()}, {"Quat", from.GetQuaternion()} }; }
void King::from_json(const json& j, Rotation& to) { j.at("AngleMag").get_to(to.Get_magnitude()); j.at("EulerXYZ").get_to(to.Get_unit_direction()); j.at("Quat").get_to(to.GetQuaternion()); }
// operators
Rotation King::operator*(const AngularVelocity& velIn, const UnitOfMeasure::Time& t)
{
    return Rotation(velIn.Get_magnitude() * t, velIn.Get_unit_direction());
}
Rotation King::operator*(const UnitOfMeasure::Time& t, const AngularVelocity& velIn)
{
    return Rotation(velIn.Get_magnitude() * t, velIn.Get_unit_direction());
}
// methods
// functions
/******************************************************************************
*   Position
******************************************************************************/
// Streams
std::ostream& King::operator<< (std::ostream& os, const Position& in) { return os << "{" << " Pos: " << in.Get_position() << " }"; } // text out
std::wostream& King::operator<< (std::wostream& os, const Position& in) { return os << L"{" << L" Pos: " << in.Get_position() << L" }"; } // text out
std::istream& King::operator>> (std::istream& is, Position& out) { return is >> out._position; } // binary in
std::wistream& King::operator>> (std::wistream& is, Position& out) { return is >> out._position; } // binary in
// json
void King::to_json(json& j, const Position& from) { j = json{ {"Pos", from.Get_position()} }; }
void King::from_json(const json& j, Position& to) { j.at("Pos").get_to(to.Get_position()); }
// operators
// methods
// functions
Position King::Physics::MechanicsKinematics_Trajectory(const Position& initialPosIn, const Velocity& initialVelIn, const Acceleration& constAccelIn, const UnitOfMeasure::Time& tIn)
{
    // p = p0 + v0 t + 1/2 a t^2
    Position p = initialPosIn + initialVelIn * tIn + constAccelIn * tIn * tIn * 0.5f;
    return p;
}

Position King::Physics::MechanicsKinematics_TrajectoryPositionAtTimeWithNegativeYGravity(const Position& initialPosIn, const Velocity& initialVelIn, const UnitOfMeasure::Time& tIn)
{
    // p = p0 + v0 t + 1/2 g t^2
    auto g = Acceleration(UnitOfMeasure::gravity, float3(0.f, -1.0f, 0.f));
    Position p = initialPosIn + initialVelIn * tIn + g * tIn * tIn * 0.5f;
    return p;
}

UnitOfMeasure::Time King::Physics::MechanicsKinematics_TrajectoryTimeAtMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn)
{
    // t1 = v0Y / g
    // t1 = v0 sin(theta) / g
    UnitOfMeasure::Time t1 = UnitOfMeasure::Speed(initialVelIn.GetVector().GetY()) / gravity;
    return t1;
}
// h = v0Y^2 / 2g
UnitOfMeasure::Length King::Physics::MechanicsKinematics_TrajectoryHeightAtMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn)
{
    UnitOfMeasure::Speed v0Y(initialVelIn.GetVector().GetY());
    UnitOfMeasure::Length dy = (v0Y * v0Y) / (gravity * 2.f);
    return dy;
}
// Hooke's Law
// F = -k d ; k = spring constant, d = displacement, and - because the force is opposite the direction of displacement
UnitOfMeasure::Energy King::Physics::MechanicsWork_SpringWorkFromDistance(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Distance& dIn)
{
    float displacement(unitVectorSpringLineOfMotion.DotProduct(dIn));

    UnitOfMeasure::Energy work;
    work = 0.5f * (float)kSpringConstantIn * (float)displacement * (float)displacement;
    return work;
}
UnitOfMeasure::Energy King::Physics::MechanicsWork_SpringWorkFromTwoPositions(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Position& spring_p0In, const Position& p1In, const Position& p2In)
{
    auto d01In = p1In - spring_p0In;
    auto d02In = p2In - spring_p0In;

    auto l01 = unitVectorSpringLineOfMotion.DotProduct(d01In);
    auto l02 = unitVectorSpringLineOfMotion.DotProduct(d02In);

    auto displacement = l02 - l01;

    UnitOfMeasure::Energy work;
    work = 0.5f * (float)kSpringConstantIn * (float)displacement * (float)displacement;
    return work;
}

// d = F / k ; if force is in spring line of motion, a positive force causes a positive displacement
Distance King::Physics::MechanicsWork_SpringDistanceFromForce(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Force& fIn)
{
    auto projectedForce = unitVectorSpringLineOfMotion.DotProduct(fIn);
    return Distance(projectedForce / kSpringConstantIn, unitVectorSpringLineOfMotion);
}
/******************************************************************************
*   PhysicsState
******************************************************************************/
// Streams
// json
void King::to_json(json& j, const PhysicsState& from)
{ 
    j = json { 
        {"_angularAcceleration", from._angularAcceleration},
        {"_angularVelocity", from._angularVelocity},
        {"_angularMomentum", from._angularMomentum},
        {"_pointOnAxisOfRotationLocalSpace", from._pointOnAxisOfRotationLocalSpace},
        {"_rotation", from._rotation},
        {"_principalMomentsOfInertia", from._principalMomentsOfInertia},
        {"_productsOfInertia", from._productsOfInertia},
        {"_linearAcceleration", from._linearAcceleration},
        {"_linearVelocity", from._linearVelocity},
        {"_linearMomentum", from._linearMomentum},
        {"_positionWorldSpace", from._positionWorldSpace},
        {"_linearKineticEnergy", from._linearKineticEnergy},
        {"_angularKineticEnergy", from._angularKineticEnergy},
        {"_potentialEnergy", from._potentialEnergy}
        }; 
}
void King::from_json(const json& j, PhysicsState& to)
{ 
    j.at("_angularAcceleration").get_to(to._angularAcceleration);
    j.at("_angularVelocity").get_to(to._angularVelocity);
    j.at("_angularMomentum").get_to(to._angularMomentum);
    j.at("_pointOnAxisOfRotationLocalSpace").get_to(to._pointOnAxisOfRotationLocalSpace);
    j.at("_rotation").get_to(to._rotation);
    j.at("_principalMomentsOfInertia").get_to(to._principalMomentsOfInertia);
    j.at("_productsOfInertia").get_to(to._productsOfInertia);
    j.at("_linearAcceleration").get_to(to._linearAcceleration);
    j.at("_linearVelocity").get_to(to._linearVelocity);
    j.at("_linearMomentum").get_to(to._linearMomentum);
    j.at("_positionWorldSpace").get_to(to._positionWorldSpace);
    j.at("_linearKineticEnergy").get_to(to._linearKineticEnergy);
    j.at("_angularKineticEnergy").get_to(to._angularKineticEnergy);
    j.at("_potentialEnergy").get_to(to._potentialEnergy);
}
// operators
// methods
/******************************************************************************
*   PhysicsRigidBody
******************************************************************************/
// Streams
// json
// operators
// methods
void King::PhysicsRigidBody::Update(const UnitOfMeasure::Time& dtIn)
{
    // i is known (make sure it is seeded in the constructor)

    // Work
    Force netForce;
    _forcesActingOnBody;
    for (auto& appliedForce : _forcesActingOnBody)
    {
        Force& F = appliedForce.first;
        Distance& r = appliedForce.second;
        auto l = (UnitOfMeasure::Length)r;

        netForce += F;

        // *** TO DO ***
        // rotation
    }
    _forcesActingOnBody.clear();

    // Newton's 2nd law
    i._linearAcceleration = netForce / _mass;

    // constant over time step
    f._linearAcceleration = i._linearAcceleration;
    f._angularAcceleration = i._angularAcceleration;

    // calculate
    f._linearVelocity = i._linearVelocity + f._linearAcceleration * dtIn; // m/s
    f._angularVelocity = i._angularVelocity + f._angularAcceleration * dtIn; // rad/s

    f._positionWorldSpace = i._positionWorldSpace + f._linearVelocity * dtIn;
    f._rotation = i._rotation + f._angularVelocity * dtIn;

    // depends on geometry and direction of revolution
    // *** TO DO ***
    if ((bool)f._rotation && f._rotation != i._rotation)
    {
        float r = 1.0f; // radius
        f._principalMomentsOfInertia = 0.4f * _mass * r * r; // sphere
        f._productsOfInertia = float3(0.f, 0.f, 0.f); // sphere
    }

    // Momentum
    f._linearMomentum = f._linearVelocity * _mass; // g * m/s
    //f._angularMomentum = f._angularVelocity * f._principalMomentsOfInertia; // g * rad/s *** TODO *** implement momentum classes math operators
    // Newton's 1st law is conservation of momentum

    // Energy
    f._potentialEnergy = f._positionWorldSpace.Get_position().GetY() * _mass * gravity; // kg m^2 /s^2 = 1 joule
    f._linearKineticEnergy = f._linearVelocity.Get_magnitude() * f._linearVelocity.Get_magnitude() * _mass * 0.5f;
    f._angularKineticEnergy;
}


// functions

// Functions to remind us of what velocity and acceleration is without their magnitudes, the tangent and principle unit vector of motion.  If Acceleration is zero, we are moving in a straight line.
// mathematics of vectors, tangent and normals, https://ltcconline.net/greenl/courses/202/vectorFunctions/tannorm.htm
float3 __vectorcall King::UnitTangentVector(Velocity velIn) { return velIn.Get_unit_direction(); }

// The analogue to the slope of the tangent line is the direction of the tangent line. Since velocity is the derivative of position, it is a tangent function to position.
float3 __vectorcall King::UnitNormalPrincipleVector(Acceleration accIn) { return accIn.Get_unit_direction(); }

// accIn = (at * ͢T) + (an * ͢N)
//  ͢aT = (at * ͢T) ; magnitude and direction
Acceleration __vectorcall King::AccelerationTangentialComponent(Acceleration accIn, Velocity velIn) { Acceleration aT; aT.Set_unit_direction(UnitTangentVector(velIn)); aT.Set_magnitude(float3::DotProduct(accIn.GetVector(), aT.Get_unit_direction()).GetX()); return aT; }
//  ͢aN = (an * ͢N) ; magnitude and direction
Acceleration __vectorcall King::AccelerationNormalComponent(Acceleration accIn, Velocity velIn) { Acceleration aN; aN.Set_unit_direction(UnitNormalPrincipleVector(accIn)); aN.Set_magnitude(float3::CrossProduct(velIn.GetVector(), accIn.GetVector()).GetMagnitude() / velIn.Get_magnitude()); return aN; }
