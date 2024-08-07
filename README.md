## New 2.7 Release of 3D Geometry King

09/9/2023 - Version 2.7 - Added Model copy constructors and assignment operators for Triangle and Quad classes, which in turn
        call the CreateMeshFrom(...) methods to construct the mesh from the primitive.
        Also continued work on Contact class adding impulse accumulation and Sphere on Sphere point detection
        Added Sphere::FindNearestPointFromSphere(const float3& pt3In)
        Collidable class received sleep tracking variable for physics simulations
        Added _boneHierarchy into Model class which depreciates SkinnedModel. Model now loads and writes the optional SkinnedModel
        data at the EOF. Identical to the original SkinnedModel v1 format. Only adds a pointer to Model if not used so the need
        for a separate class to save a 64 bit pointer per model was considered not worth the savings.

# 2D & 3D GeometryKing

Purpose: Generic, render independent, geometric game engine foundation. 
Intended as the foundation for any type of game and software requiring 3D or 2D representation. No rendering is provided, instead, the basics needed to for representing 2D and 3D geometry, collision detection, math acceleration, and model generation of vertex and index buffers to translate class primitives into meshes ready for a rendering pipeline you provide. 2D is managed in memory blocks rather than vertex and index buffers as are the 3D models. Therefore, they can be saved and stored easily in a raw or image format for viewing. C\+\+ class example has ImageBlock that inherits from MemoryBlock that adds a Draw(...) method to draw our class primitives (such as line) into its memory buffer. 3DGeometryKing files have similar methodes within classes such as Model of CreateMesh(...) to convert 3D primitives to Models with vertex and index buffers ready for rendering. 

2D Example export from class ImageTga after being drawn from 2D primities of Triangle2DF, Circle2DF, and Rectangle2DF:

![image](https://user-images.githubusercontent.com/15188055/192162868-6b863a96-c34d-49b2-83d2-d2f9f18e1af1.png)

ImageTga maintains our render independent status as well as visualization of usage cases (path finding, intersection tests, transforms, clipping, etc.) of geometry. Provides a way to test your code before connecting your rendering backend.

Compiled with Visual Studio 2022, C\+\+17, 64 Bit Windows 11

Dependencies, visit:
   json:    [https://github.com/nlohmann/json](https://github.com/nlohmann/json)
   
   Math:    [https://github.com/ChrisKing340/MathSIMD](https://github.com/ChrisKing340/MathSIMD)
   
   Physics: [https://github.com/ChrisKing340/PhysicsKing](https://github.com/ChrisKing340/PhysicsKing)

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
    class Rectangle2D; // not accelerated, replaces Windows RECT class through conversions
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

## General utilities

    #include "General\MemoryBlock.h"
    class MemoryBlock

    #include "General\TextFileParse.h"
    class TextFileParse

## Physics collision resolution 

TO DO: working on collision points of contact and solvers for varying geometries. When complete, finishes the features list for a generic render independent geometric game engine.

https://youtu.be/vbz0JtT7-vk?si=KqjE_mo5nLp_SyZB
