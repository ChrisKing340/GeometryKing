# GeometryKing
Generic game engine foundation.  All the basics of 2D and 3D geometry and model IO.  C++ generic Classes with no rendering dependencies.

GeometryKing contains the base SIMD data types class wrappers to seamlessly accelerate your code using the popular DirectXMath library (not dependent on DirectX). From this base, GeometryKing defines geometry types, classes and methods that make the basis of 2D and 3D games, simulators, and engineering applications.

Compiled with Visual Studio 2019, C++17, 64 Bit Windows 10

This code is a small part of a fully functional DirectX 12 game engine and physics simulator of mine. Typical usage is load, store, manipulate, and interact with geometry constructs and models.  My game engine, not included here, uses DirectX12 to define graphics render classes that inherit from GeometryKing model class, for example, to add methods that copy data into the DirectX 12 GPU buffers ready for rendering, among other functionality.  I hope you enjoy this work which is free to you and covered under the MIT license.

For the latest version of DirectXMath, visit:
<https://github.com/Microsoft/DirectXMath>

For the latest version of json visit:
<https://github.com/nlohmann/json>

## Geometry foundation

    #include "2DGeometryKing\2DGeometry.h"
    class Line2DF; // SIMD
    class Triangle2DF; // SIMD
    class Rectangle2DF; // SIMD
    class Rectangle2D; // not accelerated, replaces Windows RECT class thourgh conversions
    class Circle2DF; // SIMD

    #include "3DGeometryKing\3DGeometry.h" 
    class Collidable; // pure virtual to create policy of which basic geometry interacts with each other
    class Pose; // SIMD scale, rotation, and translation
    class Point; // SIMD individual point float[4] with w=1 to allow for matrix multiplication to translate
    class Line; // SIMD two points
    class Path; // SIMD multiple points in sequence
    class Ray; // SIMD origin and direction
    class Plane; // SIMD float[4]
    class Triangle; // SIMD three points;
    class Quad; // SIMD four points
    class Sphere; // SIMD float[4]
    class Capsule; // SIMD two points & a float
    class Box;  // SIMD two points; AABB and as a OBB by passing in a Quaternion to methods
    class Frustum; // SIMD six planes and eight points
    
## Geometry Mesh and Models

Complex models require more than just simple geometry shapes.  Multiple meshes of triangles with different properties for the faces and each vertex must be stored, read, and manipulated for more complex shapes and actions to occur.  Also, for efficiency, indexed buffers allow for removal of duplicate data.  In our system, Models classes own the data and all other classes reference into it.  Our MemoryBlock buffers are therefore allocated in Model and then passed by pointer to other objects.  We also allow for wire frame by building line meshes and not just triangle meshes.  SkinnedModel is our high level object class for use in your engine.

    // 3D Complex objects, working with vertex buffer references:
    class LineIndexed; // int[2] into a vertex buffer
    class TriangleIndexed; // int[3] into a vertex buffer
    
    // 3D Meshs, data is not owned and rather referenced from the model that stores it.
    class LineMesh;
    class TriangleMesh;
    
    // 3D Structure that matches verticies to an underlining connected hierarchy
    class BoneHierarchy; //skelton structure with transforms, index, and weights to influence verticies
    
    // 3D Models containing meshes.  Models own the master data that is shared by different meshes. 
    class HeightGrid; // initalized using a TriangleMesh, keeps a rectangular grid of float heights in the XZ domain
    class LineModel; // from lines
    class Model; // from triangles
    class SkinnedModel; // Model with a pointer to BoneHierarchy data (if it is not skinned then pointer is not initalized)
    
    // Helpers
    class VertexAttrib;
    class VertexFormat;

## Math foundation

    #include "MathSIMD\MathSIMD.h"
    // unique data types built on Single Instruction Multiple Data, SIMD, DirectXMath library of intrinsics for speed and simple implementation
    class UIntPoint2; // not accelerated
    class IntPoint2; // not accelerated
    class IntPoint3; // not accelerated
    class FloatPoint2; // SIMD
    class FloatPoint3; // SIMD
    class FloatPoint4; // SIMD
    class Quaternion; // SIMD
    // type defines to use in your generic code base and easily change out when moving to different libraries
    typedef UIntPoint2      uint2;
    typedef IntPoint2       int2;
    typedef IntPoint3       int3;
    typedef FloatPoint2     float2;
    typedef FloatPoint3     float3;
    typedef FloatPoint4     float4;
    typedef Quaternion      quat; 

## General utilities    

    #include "General\MemoryBlock.h"
    class MemoryBlock
    #include "General\TextFileParse.h"
    class TextFileParse
    
## Physics foundation

Foundation classes for measurement stored as a scalar. Each has base math functionality, IO streaming, and json serialization.  The classes just keeps one value using SI unit of measure.  String literals implemented to allow definition with the unit of measure desired (SI, English, British Engineering). 
Ex: Length(10_m)
Ex: Length(10_ft)

    #include "Physics\UnitOfMeasure.h"
    namespace UnitOfMeasure;
    class Mass; // scalar in kg
    class Length; // scalar in m
    class Area; // scalar in m^2
    class Volume; // scalar in m^3
    class Energy; // scalar in Joules
    class Power; // scalar Watts (J/s)
    class Strength; // scalar part of a Force vector in N
    class Accel; // scalar part of an Acceleration vector in m/s^2
    class Speed; // scalar part of a Velocity vector in m/s
    class Motion; // scalar part of Momentum vector in Joules
    class Temperature; // scalar in degrees Celsius
    class Time; // scalar in secounds
    
    class Inertia; // scalar in kg * m^2
    class AngularMotion; // scalar part of angular momentum in radians
    class Angle; // scalar in radians
    class AngularStrength; // scalar part of torqe in N * m (J)
    class AngularAccel; // scalar part of angular acceleration in radians
    class AngularSpeed; // scalar part of angular velocity in radians
    
    const Accel gravity;
    const Speed speedOfSoundInAir;
    
    #include "Physics.h"
    class Force ; // keeps a UnitOfMeasure::Strength and a unit direction vector
    // Linear
    class Acceleration ; // keeps a scalar UnitOfMeasure::Accel and a unit direction vector // operator for Acceleration = Force / Mass
    class Velocity ; // keeps a scalar UnitOfMeasure::Speed and a unit direction vector // operator for Velocity = Acceleration * Time
    class Position ; // keeps 3 floats for x,y,z and operators for arithmetic with Distance
    class Distance ; // keeps a scalar UnitOfMeasure::Length and a unit direction vector // operator for Distance = Velocity * Time
    // Rotational
    class Torque ; // keeps a scalar UnitOfMeasure::AngularStrength and a unit direction vector that the torque acts to spin (axis of rotation)
    class AngularAcceleration ; // keeps a scalar UnitOfMeasure::AngularAccel and a unit direction vector that the acceleration acts to spin (axis of rotation)
    class AngularVelocity ; // keeps a scalar UnitOfMeasure::AngularSpeed and a unit direction vector that the velocity acts to spin (axis of rotation)
    class Rotation ; // keeps a scalar UnitOfMeasure::Angle and a unit direction vector (axis of rotation) spun about
    
    // Functions to solve physics cases
    // Usesful to show how to use the scalars and classes in solving equations in physics that model the real world
    #include "Physics.h"
    namespace Physics;
    
    Case: Trajectories
    Case 0: position along a trajectory, p = p0 + v0 t + 1/2 a t^2
        Position    MechanicsKinematics_Trajectory(const Position& initialPosIn, const Velocity& initialVelIn, const Acceleration& constAccelIn, const UnitOfMeasure::Time& tIn);
    Case 1: a = gravity, p = p0 + v0 t + 1/2 g t^2
        Position    MechanicsKinematics_TrajectoryPositionAtTimeWithNegativeYGravity(const Position& initialPosIn, const Velocity& initialVelIn, const UnitOfMeasure::Time& tIn); 
    Case 2: time to reach maximum height, t = v0y / g
        Time        MechanicsKinematics_TrajectoryTimeAtMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn);
    Case 3: height at maximum reached, h = v0y^2 / (2g)
        Length      MechanicsKinematics_TrajectoryHeightAtMaximumHeightWithNegativeYGravity(const Velocity& initialVelIn);
    
    Case: Springs
    Case 0: work done on spring by displacement distance
        Energy      MechanicsWork_SpringWorkFromDistance(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Distance&);
    Case 1: work done on spring by dispalcement from position p1 to p2
        Energy      MechanicsWork_SpringWorkFromTwoPositions(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Position& spring_p0In, const Position& p1In, const Position& p2In);
    Case 2:  distance spring displaced from an applied force
        Distance    MechanicsWork_SpringDistanceFromForce(const float& kSpringConstantIn, const float3& unitVectorSpringLineOfMotion, const Force&);
