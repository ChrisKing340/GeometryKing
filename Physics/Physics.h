#pragma once
// 3rdPart namespace
#include "..\..\json\single_include\nlohmann\json.hpp"
using json = nlohmann::json; // for convenience
// King namespace
#include "UnitOfMeasure.h" // has own namespace King::UnitOfMesure::
#include "Force.h"
#include "Torque.h"
#include "Acceleration.h"
#include "AngularAcceleration.h"
#include "Velocity.h"
#include "AngularVelocity.h"
#include "Rotation.h"
#include "Distance.h"
#include "Position.h"

#include "PhysicsMaterial.h"
#include "PhysicsState.h"
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

// Mechanics
//    Kinematics
//        Trajectory

namespace King {

namespace Physics {
    // Templates
    //class alignas(16) class Sphere : public PhysicsRigidBody {};
    //PhysicsRigidBody cube(const float xDim, const float yDim, const float zDim);

    //*** MECHANICS ***

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
    //*** Work ***
    // Work links the concept of force and energy and is most useful when force varies with time, and therefore acceleration is not constant
    // Use operators defined in class Distance for Energy = Force * Distance
    UnitOfMeasure::Energy MechanicsWork_SpringWorkFromDistance(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Distance&);
    UnitOfMeasure::Energy MechanicsWork_SpringWorkFromTwoPositions(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Position& spring_p0In, const Position& p1In, const Position& p2In);
    Distance MechanicsWork_SpringDistanceFromForce(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Force&);
}
}
