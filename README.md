# GeometryKing
Generic game engine foundation.  All the basics of 2D and 3D geometry and model IO.  C++ generic Classes with no rendering dependencies.

GeometryKing contains the base SIMD data types class wrappers to seamlessly accelerate your code using the popular DirectXMath library (not dependent on DirectX). From this base, GeometryKing defines geometry types, classes and methods that make the basis of 2D and 3D games, simulators, and engineering applications.  Start with class SkinnedModel to dive into the high level functionality of the library.

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

Foundation classes to represent a unit of measure with a scalar.  The classes just keeps one value with internal storage as a SI unit of measure.  String literals implemented to allow definition with the unit of measure desired.  Operator overloading to act as a base type and also supports streams and json from/to;
Ex: Length(10_m)
Ex: Length(10_ft)

    #include "Physics\UnitOfMeasure.h"
    namespace UnitOfMeasure;
    class Mass; // scalar
    class Length; // scalar
    class Area; // scalar
    class Volume; // scalar
    class Energy; // scalar
    class Power; // scalar
    class Strength; // scalar part of a Force vector
    class Accel; // scalar part of an Acceleration vector
    class Speed; // scalar part of a Velocity vector
    class Temperature; // scalar
    class Time; // scalar
    const Accel gravity;
    const Speed speedOfSoundInAir;
    
    #include "Physics\Force.h"
    class Force ; // keeps a UnitOfMeasure::Strength and a unit direction vector
    
    #include "Physics\Acceleration.h"
    class Acceleration ; // keeps a UnitOfMeasure::Accel and a unit direction vector
    // operator for Acceleration = Force / Mass
    
    #include "Physics\Velocity.h"
    class Velocity ; // keeps a UnitOfMeasure::Speed and a unit direction vector
    // operator for Velocity = Acceleration * Time
    
    #include "Physics\Distance.h"
    class Distance ; // keeps a UnitOfMeasure::Length and a unit direction vector
    // operator for Distance = Velocity * Time

    #include "Physics\Position.h"
    class Position ; // keeps 3 floats for x,y,z and operators for arithmetic with Distance
    
