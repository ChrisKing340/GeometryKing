## New 2.8 Release of 3D Geometry King

10/15/2024 - Version 2.8 - Added several new methods to return intersection points to basic geometries. This is used to return contact points for collision responses outside the library. Also added a new primitive class, Pyramid, and intersection code added to most of the other primitive geometries. Previous version added class Capsule and class Collided to store multiple contacts between geometries. The foundation is now within the library to implement physics into your games and simulations. The response handling to collisions is outside this library, as that is out of scope. The intent of this library is foundational, to build your own engine from it. An example, using the library as base geometries you might implement a culling function before you render, checking each object (or actor object in this example) for a fixed distance from the camera:

    
    unsigned int CullObjectsOutofViewFrustumDynamic(const Frustum& wF, float3 cameraPosition)
    {
    unsigned int culled(0);
    const float radius = 25000.f;

    // dynamic objects
    for (const auto& i : actorCache)
    {
        const Box& bb = i.second->GetBoundingBox();
        const auto& loc = bb.GetCenter();

        if ((cameraPosition - loc).GetMagnitudeEst() < radius && wF.Intersect(bb))
            i.second->SetCulledFlag(false);
        else
        {
            i.second->SetCulledFlag(true);
            ++culled;
        }
    }

    return culled;
    }

# 2D & 3D GeometryKing

Purpose: Generic, render independent, geometric game engine foundation. 
Intended as the foundation for any type of game and software requiring 3D or 2D representation. No rendering is provided, instead, the basics needed to for representing 2D and 3D geometry, collision detection, math acceleration, and model generation of vertex and index buffers to translate class primitives into meshes ready for a rendering pipeline you provide. 2D is managed in memory blocks rather than vertex and index buffers as are the 3D models. Therefore, they can be saved and stored easily in a raw or image format for viewing. C\+\+ class example has ImageBlock that inherits from MemoryBlock that adds a Draw(...) method to draw our class primitives (such as line) into its memory buffer. 3DGeometryKing files have similar methodes within classes such as Model of CreateMesh(...) to convert 3D primitives to Models with vertex and index buffers ready for rendering. 

3D Example (Models created from 3D primities of Pyramid, Sphere, and Box)

![image](https://github.com/user-attachments/assets/b2f6a2b9-4caa-415d-9154-c15210d4a212)

2D Example (export from class ImageTga after being drawn from 2D primities of Triangle2DF, Circle2DF, and Rectangle2DF)

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
    class Capsule; // SIMD two points & a float
    class Box;  // SIMD two points
    class Fustrum; // SIMD six planes and eight points
    class Pyramid; // SIMD one point and two floats
    class Collided; // Vector to store multiple contacts between geometries and methods to determine contacts

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

TO DO: working on collision points of contact and solvers for varying geometries. When complete, finishes the features list for a generic render independent geometric game engine. This is targeted for Release 3.0

https://youtu.be/vbz0JtT7-vk?si=KqjE_mo5nLp_SyZB
