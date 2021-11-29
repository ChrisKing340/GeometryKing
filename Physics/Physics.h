#pragma once
// 3rdPart namespace
#include "..\..\json\single_include\nlohmann\json.hpp"
using json = nlohmann::json; // for convenience
// King namespace
#include "UnitOfMeasure.h" // has own namespace King::UnitOfMesure::

// Forces
#include "Force.h"
// Linear
#include "Acceleration.h"
#include "Velocity.h"
#include "Position.h"
#include "Distance.h"
// Rotational
#include "Torque.h"
#include "AngularAcceleration.h"
#include "AngularVelocity.h"
#include "Rotation.h"
// Properties
#include "PhysicsMaterial.h"
#include "PhysicsState.h"
//#include "PhysicsObject.h"
#include "PhysicsRigidBody.h"

// Symbols: 𝛼𝛽𝛾𝜃𝛷𝜏𝜔𝜌𝜋𝜎𝜇𝜆𝜀𝛥ζ 𝑖𝑗𝑘 𝑚𝑛𝑟𝑠𝑡 𝒾𝒿𝓀𝓁𝓂𝓃𝒹𝒶𝒷𝒸 ±° ⊙⊚ ∫∬∭∮∯∰∝∞∟∠∡∑√∛∜∴≈⨯• ͢   ͢𝑖  ͢𝑗  ͢𝑘

/******************************************************************************
*	Physic Basics
*
*	Newton's 1st Law:
*		An object at rest stays at rest and an object in motion stays in motion
*		unless acted upon by a net sum external force. Known as Law of inertia.
*	Newton's 2nd Law:
*		The acceleration of an object as produced by a net force is directly
*		proportional to the magnitude of the net force, in the same direction
*		as the net force, and inversely proportional to the mass of the object.
*       Linear:
*			a = ∑ ͢F / m
*			∑ ͢F = m • ͢a
*	Newton's 3rd Law:
*		For every action, there is an equal and opposite reaction.
*	Momentum:
*		Conservation of momentum. An object which is moving has momentum. The
*		amount of momentum (p) possessed by the moving object is the product of
*		mass (m) and velocity (v).
*			p = m • ͢v
*		Change in momentum:
*			dp/dt = m • d ͢v / dt = m • ͢a
*		With the 2nd Law:
*			m • ͢a = (i = 0...n) ∑ Fi
*	Impulse:
*		The product of force (F) and time (t) over which the force is applied.
*		Number for forces (n) acting during the time interval (∆t).
*			Impulse = ᶴF dt
*			Impulse = ∑F/n • ∆t = Fave • ∆t ; slug * ft / s = lbf * s
*		Impulse = Momentum Change
*			Impulse = dp = m • v2 - m • v1
*			Fave • ∆t = dp/dt
*			dp/dt = m • d ͢v / dt = m • ∆ ͢v / ∆t; assumes time step is constant
*		When two object collide (1) (2), the time of contact is the same
*			t1 = t2
*		The force exerted on object 1 (F1) is equal in magnitude and opposite
*		in direction to the force exerted on object 2 (F2) (Newton's 3rd Law).
*			F1 = - F2
*		The momentum change experienced by object 1 is equal in magnitude and
*		opposite in direction to the momentum change experienced by object 2.
*			m1 • ∆ ͢v1 = - m2 • ∆ ͢v2
*		And so, the sum of the momentum of object 1 and the momentum of object
*		2 before the collision is equal to the sum of the momentum of object 1
*		and the momentum of object 2 after the collision.  v1' and v2' will
*		represent the velocities of objects 1 and 2 after the collision.
*			m1 • ͢v1 + m2 • ͢v2 = m1 • ͢v1' + m2 • ͢v2'
******************************************************************************/

// note https://www.euclideanspace.com/physics/dynamics/inertia/rotation/rotationfor/index.htm#angularacceleration goes into interesting thoughts about frames of reference for physics engines

// Mechanics
//    Kinematics
//        Trajectory

namespace King {
    // Curved motion, use component accelerations
    float3 __vectorcall UnitTangentVector(Velocity velIn); // The analogue to the slope of the tangent line is the direction of the tangent line. Since velocity is the derivative of position, it is a tangent function to position.
    float3 __vectorcall UnitNormalPrincipleVector(Acceleration accIn); // principle unit vector. Geometrically, for a non straight curve, this vector is the unique vector that point into the curve.
    // acceleration = ͢aT + ͢aN
    // acceleration = (at * ͢T) + (an * ͢N) ; at = acceleration tangential magnitude component; an = acceleration normal magnitude component
    //  ͢aT = (at * ͢T) ; magnitude and direction
    Acceleration __vectorcall AccelerationTangentialComponent(Acceleration accIn, Velocity velIn);
    //  ͢aN = (an * ͢N) ; magnitude and direction
    Acceleration __vectorcall AccelerationNormalComponent(Acceleration accIn, Velocity velIn);

namespace Physics {
    // Templates
    //class alignas(16) class Sphere : public PhysicsRigidBody {};
    //PhysicsRigidBody cube(const float xDim, const float yDim, const float zDim);

    //*** MECHANICS ***
    // all of the below are a subset topics belonging to mechanics

    //*** Kinematics *** 
    // Kinematics is most useful with the force on an object is constant, and therefore acceleration is constant (such as the force of gravity).
    // p = p0 + v0 t + 1/2 a t^2
    Position MechanicsKinematics_Trajectory(const Position& initialPosIn, const Velocity& initialVelIn, const Acceleration& constAccelIn, const UnitOfMeasure::Time& tIn);
    // p = p0 + v0 t + 1/2 g t^2
    Position MechanicsKinematics_TrajectoryPositionAtTimeWithNegativeYGravity(const Position& initialPosIn, const Velocity& initialVelIn, const UnitOfMeasure::Time& tIn);
    // t1 = v0Y / g
    UnitOfMeasure::Time MechanicsKinematics_TrajectoryTimeAtMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn);
    // h = v0Y^2 / 2g
    UnitOfMeasure::Length MechanicsKinematics_TrajectoryHeightAtMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn);
    
    // *** Dynamics of Rotations ***
    // AngularVelocity class has methods to calculate linear acceleration from the rotational motion
    // ͢a = ͢a0 + 𝛼 x ͢r + ͢𝜔 x ( ͢𝜔 x ͢r )
    // ͢a = King::AngularVelocity::CalculateLinearAccelerationFrom(const Acceleration& a0In, const AngularAcceleration& alphaIn, const Distance& rIn)
    // recall that ͢a = ͢at + ͢an
    // ͢an = r • | ͢𝜔|^2 ; with direction along radius (and opposite) to maintain curviture
    //  ͢at = ͢a - ͢an
    // AngularVelocity class also has a method to calculate the linear velocity at the end of a radius about the angular velocity axis of rotation
    // v = ͢𝜔 x ͢r ; 

    //*** Work ***
    // Work links the concept of force and energy and is most useful when force varies with time, and therefore acceleration is not constant
    // Use operators defined in class Distance for Energy = Force * Distance
    // Springs
    UnitOfMeasure::Energy MechanicsWork_SpringWorkFromDistance(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Distance&);
    UnitOfMeasure::Energy MechanicsWork_SpringWorkFromTwoPositions(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Position& spring_p0In, const Position& p1In, const Position& p2In);
    Distance MechanicsWork_SpringDistanceFromForce(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Force&);

    //*** Thermodynamics ***
    // Since UnitOfMeasure::Energy is a scalar, math in conserving it is straight forward. For completion of mechanics discussion, some basics here:
    // 1st law: ∆E = Q - W
    // ∆E is the total change in the energy of the system
    // Q is the heat added to the system
    // W is the work performed on the system by the environment
    // law states that energy is conserved, therefore a change in the systems energy from two states must either exchange work or heat.
    // Work on the external environment (negative work, which is outward)
    // Q release heat to the environment (negative heat, which is outward)
    // ∆E = ∆U + ∆PE + ∆KE
    // ∆E = StateFinal - StateInitial
    // PE = m * h * g
    // KE = 1/2 m * v^2
    // U = P * V = m * R(of the gas) * T ; T in kelvin (note this is the ideal gas law, so assumed uncompressed gas)
    // Q = combustion or other source through mechanisms of radiation, conduction, or convection
}
}
