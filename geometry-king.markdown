---
title: Geometry King
date: 2020-05-31 21:10:00 Z
layout: jekyll-theme-cayman
---

# GeometryKing

Generic, render independent, geometric game engine. Intended as the foundation for made from scratch game engines with all the basics of 2D/3D geometry, model loading and saving, physics body simulation, and contact solvers.  C\+\+ generic Classes include geometry foundations, meshes and models, SIMD math foundation, physics foundation, physics simulation, and contact solvers.  All foundational classes support json streams to serialize and easily transmit your data structures across a network, cout, or save to local files.

Compiled with Visual Studio 2019, C\+\+17, 64 Bit Windows 10

This code is a small part of a fully functional DirectX 12 game engine and physics simulator. Typical usage is load, store, manipulate, and interact with geometry constructs and models.

GeometryKing contains the base SIMD data types class wrappers to seamlessly accelerate your code using DirectXMath library (not dependent on DirectX) for cross platform intrinsic functions.

For the latest version of DirectXMath, visit:
[https://github.com/Microsoft/DirectXMath](https://github.com/Microsoft/DirectXMath)

For the latest version of json visit:
[https://github.com/nlohmann/json](https://github.com/nlohmann/json)

## Geometry foundation
   
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
    class Box;  // SIMD two points
    class Fustrum; // SIMD six planes and eight points

    #include "2DGeometryKing\2DGeometry.h"
    class Line2DF; // SIMD
    class Triangle2DF; // SIMD
    class Rectangle2DF; // SIMD
    class Rectangle2D; // not accelerated, replaces Windows RECT class thourgh conversions
    class Circle2DF; // SIMD

## Geometry Mesh and Models

Complex models require more than just simple geometry shapes.  Multiple meshes of triangles with different properties for the faces and each vertex must be stored, read, and manipulated for more complex shapes and actions to occur.  Also, for efficiency, indexed buffers allow for removal of duplicate data.  In our system, Models classes own the data and all other classes reference into it.  Our MemoryBlock buffers are therefore allocated in Model and then passed by pointer to other objects.  We also allow for wire frame by building line meshes and not just triangle meshes.  SkinnedModel is our high level object class for use in your engine.

    // 3D Models containing meshes.  Models own the master data that is shared by different meshes. 
    class HeightGrid; // initalized using a TriangleMesh, keeps a rectangular grid of float heights in the XZ domain
    class LineModel; // from lines
    class Model; // from triangles
    class SkinnedModel; // Model with a pointer to BoneHierarchy data (if it is not skinned then pointer is not initalized)  
    
    // 3D Structure that matches verticies to an underlining connected hierarchy
    class BoneHierarchy; //skelton structure with transforms, index, and weights to influence verticies
    

    // 3D Meshs, data is not owned and rather referenced from the model that stores it.
    class LineMesh;
    class TriangleMesh;
    
    // 3D Complex objects, working with vertex buffer references:
    class LineIndexed; // int[2] into a vertex buffer
    class TriangleIndexed; // int[3] into a vertex buffer
    
    // Helpers
    class VertexAttrib;
    class VertexFormat;

## Math foundation

    #include "MathSIMD\MathSIMD.h"
    // unique data types built on Single Instruction Multiple Data, SIMD, DirectXMath library of intrinsics for speed and simple implementation
    class FloatPoint2; // SIMD
    class FloatPoint3; // SIMD
    class FloatPoint4; // SIMD
    class Quaternion; // SIMD

    class UIntPoint2; // not accelerated
    class IntPoint2; // not accelerated
    class IntPoint3; // not accelerated

## General utilities

    #include "General\MemoryBlock.h"
    class MemoryBlock

    #include "General\TextFileParse.h"
    class TextFileParse

## Physics foundation

Foundation classes represent a unit of measure with a scalar.  The classes just keeps one value with internal storage as a SI unit of measure.  String literals implemented to allow definition with the unit of measure desired.  Operator overloading to act as a base type and also supports streams and json from/to;
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
    
## Physics simulation

Modeling of force, acceleration, and velocity of dynamic bodies. Combined with GeometryKing primitives of models (and conversion on Model class to vertext and index buffers for rendering) you can simulate almost anything from the real world and render it.

    #include "Physics\Physics.h"

    class Force ; // keeps a UnitOfMeasure::Strength and a unit direction vector
    class Acceleration ; // keeps a UnitOfMeasure::Accel and a unit direction vector; operator for Acceleration = Force / Mass
    class Velocity ; // keeps a UnitOfMeasure::Speed and a unit direction vector; operator for Velocity = Acceleration * Time
    class Distance ; // keeps a UnitOfMeasure::Length and a unit direction vector (essentially a distance to); operator for Distance = Velocity * Time
    class Position ; // keeps 3 floats for x,y,z and operators for arithmetic with Distance

## Physics collision resolution

TO DO: working on collision points of contact and solvers for varying geometries. Will complete the features list for a generic render independent geometric game engine.