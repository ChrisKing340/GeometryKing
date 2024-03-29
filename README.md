## New 2.6 Release of 3D Geometry King

Math and Physics have been separated from GeometryKing for use and tutorial from the main code base.

# 2D & 3D GeometryKing

Generic, render independent, geometric game engine foundation. Intended as the foundation for "made from scratch" game engines.
Physics body simulation with intrinsic SIMD acceleration from projects:

   Math:    [https://github.com/ChrisKing340/MathSIMD](https://github.com/ChrisKing340/MathSIMD)
   
   Physics: [https://github.com/ChrisKing340/PhysicsKing](https://github.com/ChrisKing340/PhysicsKing)
   
C\+\+ classes support json for data transport, 2DGeometryKing has ImageBlock with Draw(...) methods from primitives to a memory buffer, and 3DGeometryKing has Model with constructors to convert 3D primitives to Models with vertex and index buffers ready for rendering. 

Physics classes represent real world motion while the geometry classes handling collision detection. High degree of inline code for readability and compiled only if used in your projects.

2D Example export from class ImageTga after being drawn from 2D primities of Triangle2DF, Circle2DF, and Rectangle2DF:

![image](https://user-images.githubusercontent.com/15188055/192162868-6b863a96-c34d-49b2-83d2-d2f9f18e1af1.png)

ImageTga maintains our render independent status as well as visualization of usage cases (path finding, intersection tests, transforms, clipping, etc.) of geometry. Provides a way to test your code before connecting your rendering backend.

Compiled with Visual Studio 2019, C\+\+17, 64 Bit Windows 10

For the latest version of DirectXMath, visit:
[https://github.com/Microsoft/DirectXMath](https://github.com/Microsoft/DirectXMath)

For the latest version of json visit:
[https://github.com/nlohmann/json](https://github.com/nlohmann/json)

This code is the foundation of a fully functional DirectX 12 game engine and physics simulator.

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
    class Polygon2DF; // SIMD

## Geometry Mesh and Models

Complex models require more than just simple geometry shapes.  Multiple meshes of triangles with different properties for the faces and each vertex must be stored, read, and manipulated for more complex shapes and actions to occur.  Also, for efficiency, indexed buffers allow for removal of duplicate data.  In our system, Models classes own the data and all other classes reference into it.  Our MemoryBlock buffers are therefore allocated in Model and then passed by pointer to other objects.  We also allow for wire frame by building line meshes and not just triangle meshes.

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
    https://github.com/ChrisKing340/MathSIMD 
    unique data types built on Single Instruction Multiple Data, SIMD, DirectXMath library of intrinsics for speed and simple implementation
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

## Physics collision resolution 

TO DO: working on collision points of contact and solvers for varying geometries. When complete, finishes the features list for a generic render independent geometric game engine.
