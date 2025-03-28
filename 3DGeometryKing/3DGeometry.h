/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          3DGeometryKing

Description:    3D Geometry with accelerated SIMD math. Collection of classes
                for basic 3D primitives and mesh creation. Frame work for models
                keeping master data and indicies into vertex buffers. Models
                keep material properties, meshes, bounding geometry from primitives,
                and then extends this into other classes for line meshes, triangle
                meshes, and skinned models. Transforms and optimization functionality
                included as well as model loading and saving in wave front .obj
                format and a native binary formal .kng for fast loading on released
                applications. JSON external library used with << overloads to easily
                serialized our classes data types to cout, files, or networking.
                Much of what any 3D developer needs we have assembled here and he/she
                just needs to add a rendering front end.

                No dependencies outside of standard library, math intrinsics, and JSON.
                See MathSIMD.h for dependency on external intrinsics and JSON libraries.

Usage:          3D Software development

References:     Code not original or substantially created by me will be cited here:

                1) Book Title: Game Programming Gems II
                Chapter Title: Fast, Robust Intersection of 3D Line Segments
                Author: Graham Rhodes
                Revisions: 05-Apr-2001 - GSR. Original.
 
                2) M�ller, Tomas; Trumbore, Ben (1997). "Fast, Minimum Storage Ray-Triangle Intersection". Journal of Graphics Tools. 2: 21�28.
                https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

                3) Douglas Gregory (2018). OBB vs Sphere collision detection method answer. Game Development Q&A on forum stackexchange.com.
                https://gamedev.stackexchange.com/questions/163873/separating-axis-theorem-obb-vs-sphere

                4) Johnny Huynh (2009). "Separating Axis Theorem for Oriented Bounding Boxes". Site www.jkh.me
                https://www.jkh.me/files/tutorials/Separating%20Axis%20Theorem%20for%20Oriented%20Bounding%20Boxes.pdf
                    Which expands the equations into the 15 cases set forth in the original separating axis theorem paper
                    by S. Gottschalk, M. C. Lin, D. Manocha, University of North Carolina, 1996.
                    http://gamma.cs.unc.edu/SSV/obb.pdf.

Contact:        https://chrisking340.github.io/GeometryKing/

Copyright (c) 2020, 2021, 2022, 2023, 2024, 2025 Christopher H. King

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#pragma once
// needed for C++17 which had breaking change for std::make_shared
#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif

#ifndef __cplusplus
#error 3DGeometry requires C++
#endif

#if defined(_MSC_VER) && (_MSC_VER < 1920)
#error requires Visual C++ 2019 or later.
#endif

/*
    REVISION HISTORY:
    7/25/2021 - Version 2 - Functionality added in many different classes between minor releases and major code
    merged in to expand the usefulness of the library to make models. For example, Model received the ability to
    optimize vertex order, reverse normals, reverse triangle windings to match normals, and create meshes from
    other objects such as lines, paths, boxes, and sphere objects.  Previous minor releases added classes such
    as capsules, collision detection in all classes, and intersection code between all of the objects.

    11/28/2021 - Version 2.1 - Classes Model & SkinnedModel (derived) gained method CreateMeshFrom(const Path& p), 
        a closed path linking first two points to the last point, then marching down the closed path to fill it in. 
        This is an efficient way to fill polygons as opposed of the alternative of all points to a central point to 
        triangulate.
    12/12/2021 - Version 2.2 - Classes Model & SkinnedModel (derived) gained method CreateMeshFrom(const Triangle& tri); 
        CreateMeshFrom(const Quad& quad);
        Added additional initializer lists to class constructors for Point, Line, Ray, Triangle, Quad and Path
    12/13/2021 - Version 2.2.1 - Corrections/edits to 2.2
    12/20/2021 - Version 2.3 - Class Path gained method to convert the 3D Path to a 2D Polygon2DF (in XY plane). Works best
        for a planar Path and this conversion rotates it into the XY plane.
    03/05/2022 - Version 2.4 - Class Sphere gained methods for inertia, surface area and volume
    05/21/2022 - Version 2.5 - Classes Path and Line gained methods for offset, transforming them by the distance specified 
        along an "outward" normal to CCW. A plane must be specified that the normal lays within.
    01/21/2023 - Version 2.6 - Separated physics basic classes into their own library. This makes it possible to adopt the physics
        library if you are not using geometry in your projects. Collision of 2D and 3D objects will be integrated in the
        Geometry library still to combine it all together for real world applications. 
        Added class Momentum to physics library.
    09/9/2023 - Version 2.7 - Added Model copy constructors and assignment operators for Triangle and Quad classes, which in turn
        call the CreateMeshFrom(...) methods to construct the mesh from the primitive.
        Also continued work on Contact class adding impulse accumulation and Sphere on Sphere point detection
        Added Sphere::FindNearestPointFromSphere(const float3& pt3In)
        Collidable class received sleep tracking variable for physics simulations
        Added _boneHierarchy into Model class which depreciates SkinnedModel. Model now loads and writes the optional SkinnedModel
        data at the EOF. Identical to the original SkinnedModel v1 format. Only adds a pointer to Model if not used so the need
        for a separate class to save a 64 bit pointer per model was considered not worth the savings.
    09/23/2023 - Version 2.7.1 - Minor code clean up, added additional scoping, eliminated intelisense warnings in math ordering for
        ambiguous (more than one) available operators. Removed FloatPoint3 class naming and used the alias float3 for a future
        transition to permanently changing the math library from FloatPoint3 class to float3 class naming.
    11/27/2023 - Version 2.7.2 - Minor comment and code organization changes
    09/28/2024 - Version 2.8.0 - Added several new methods to return intersection points:
            Line::Intersects(const Triangle& triangleIn, float3* intersectionOut = nullptr)
            Line::Intersects(const Quad& quadIn, float3* intersectionOut = nullptr)
            Ray::Intersects(const Triangle& quadIn, float3* intersectionOut = nullptr)
            Ray::Intersects(const Quad& quadIn, float3* intersectionOut = nullptr)
            Triangle::Contains(const Point& ptIn)
            Triangle::Intersects(const Line& lineIn, float3* intersectionOut)
            Triangle::Intersects(const Triangle& triIn, float3* intersectionOut)
            Triangle::Intersects(const Quad& qIn, float3* intersectionOut)
            Triangle::Intersects(const Plane& pIn, float3* intersectionOut)
            Triangle::Intersects(const Sphere& sIn, float3* intersectionOut)
            Triangle::FindNearestPoint(const float3& pt3In) const
            Plane::Intersects(const Triangle& triIn, float3* intersectionOut)
            Box::Intersects(const Line &lineIn, float3* intersectionOut) const;
            Box::Intersects(const Triangle& rhs, float3* intersectionOut) const;
        Added class Pyramid with interpoperability with existing geometric objects and Model class to enable models of Pyramids
        and collision testing.

    WIP
    Version 3.0 Planned release - Oriented box on oriented box and sphere collision tests. Previously extended our axis 
        aligned bounding box in version 1 to oriented by passing in a orientation quaterion. WIP code to implement
        separating axis theorem. Goal is to finish the start of a physics engine by taking the OB on OB contacts and penetration
        and implementing a momentum based response to resolve the collisions (this part is out of scope of this code base).
        
        Plan: is to support OB and Sphere collisions in the v3 release candidate: 
            1st: Sphere on Sphere 
            2nd: Sphere on oriented box
            3rd: Oriented box on oriented box
        Goal: Geometry3D objects to be the base class of 3D worlds with collission detection built in. Proper handling
            by a resolving physics code requires points of contact which is provided by our class Collided initialized 
            with the object in contact and then keeps as a vector the contact points and also in class any other needed
            information to properly handle it (like weak pointers to the objects themselves, accumulated impulses to warm
            start and optimize mulitple objects in contact, etc.)
        Experimental code: 
            Function SAT_OBBonOBB(...); 
            Function SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(...)
        11/27/2023 Note: Work is done in the game engine and implemented outside of this code base for sphere on sphere.
        Need is for an object manager for response handling which is out of scope of this repository. You can find the 
        Sphere on Sphere code in Geometry3D::Collided class which is the goal of this repository to perform collission 
        detection with base and objects.
*/

//// WIP: Collsision testing, these are TEMPORARIES for experimentation
//#include "..\PhysicsKing\Physics\CK_Cube.h"
//#include "..\PhysicsKing\Physics\CK_CubeCollision.h"

// Compile with code files 3DGeometry.cpp and CK_CubeCollision.cpp

constexpr auto KING_3DGEOMETRY_VERSION_MAJOR = 2;
constexpr auto KING_3DGEOMETRY_VERSION_MINOR = 7;
constexpr auto KING_3DGEOMETRY_VERSION_PATCH = 2;

#include <DirectXCollision.h> // *** TO DO *** included for compatibility until all our classes are fully implemented
#include <vector>
#include <map>
#include <filesystem>
#include <time.h>

// Communication format
#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

// Simple memory handling
#include "..\General\MemoryBlock.h"

// Simple SIMD (single instruction multiple data) using DirectXMath accelerated library
#include "..\..\MathSIMD\MathSIMD.h"

// Physics building blocks
// Mainly use Position and Distance classes as a foundation
// We define for example Point : public Position, Collidable
#include "..\..\PhysicsKing\Physics.h"

// Geometry building blocks
#include "..\2DGeometryKing\2DGeometry.h"

using namespace std;
using namespace DirectX;

namespace King {
    // 3D objects
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
    class Pyramid; // SIMD one point and two floats
    class Collided; // Vector to store multiple contacts between geometries and methods to determine contacts

    // 3D Modeling with meshes:
    // Design is that models own the master data. Data is referenced by different meshes within the model clas. 
    class Material; // keeps material information and texture names
    class ModelScaffold; // base class for models of different geometries
    class BoneHierarchy; // 3D Structure that matches verticies to an underlining connected hierarchy

    class LineModel; // Lines
    class Model; // Triangles; Utilize Model::CreateMeshFrom(...) to create vertex data from geometries

    // *** TO DO ** Depreciate SkinnedModel. The pointer has been moved into Model and file type v2 was added to support the loading and saving of data
    class SkinnedModel; // Model with a pointer to BoneHierarchy data (if it is not skinned then pointer is not initalized)

    class HeightGrid; // Memory block of heights. Initalize using a TriangleMesh, keeps a rectangular grid of float heights in the XZ domain. Interpolates the heights along the mesh to create just a height field from it and a world bounding box with min/max heights (for collision detection)

    // 3D Meshs
    class LineMesh;
    class TriangleMesh;

    // Helper 3D Complex objects, working with vertex buffer references similar to class Triangle and Line:
    class LineIndexed; // int[2] into a vertex buffer
    class TriangleIndexed; // int[3] into a vertex buffer 

    // Helpers
    class VertexAttrib;
    class VertexFormat;
    //enum IndexFormat;

    // Functions
    // *** TO DO *** Move test functions into Collided class. Sphere on Sphere implemented, next OBB on Sphere
    // *** TO DO *** WIP an oriented box (OB) or (OBB, oriented bounding box) on Sphere collision test using nearest point intersection testing from reference #3
    // *** TO DO *** WIP an oriented box (OB) or (OBB, oriented bounding box) on OBB collision test using separating axis theorem. Objective is a contact point.
    class Collided;
    std::vector<Collided>    SAT_OBBonOBB(const Box& A, const King::Quaternion& qA, const Box& B, const King::Quaternion& qB);
    Collided                 SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(const std::vector<float3>& cornersA, const std::vector<float3>& cornersB, const float3& nonSeparatingAxis);

    // Classes
    /******************************************************************************
    *    VertexAttrib
    ******************************************************************************/
    class alignas(16) VertexAttrib
    {
    public:
        static std::vector<std::string>     description; // to describe enumDesc as strings.  Global initialized in 3DGeometry.cpp
        enum enumFormat
        {
            format_none = 0, format_byte8, format_byte8x2, format_byte8x3, format_byte8x4, format_int16, format_int16x2, format_int16x3, format_int32, format_float32, format_int32x3, format_float32x2, format_float32x3, format_int32x4, format_float32x4,
            formats
        };
        enum enumDesc
        {   position = 0, color, textureCoord, normal, tangent, bitangent, boneWeights, boneIndicies,
            types
        };

        uint16_t    _offset = 0; // bytes
        enumFormat  _format = format_none; // data format
        enumDesc    _desc   = types; // data descriptor

    public:
        VertexAttrib() = default;
        VertexAttrib(const VertexAttrib & in) : _offset(in._offset), _format(in._format), _desc(in._desc) {} // copy assign

        const auto &                        GetDescription() const { return _desc; }
        std::string                         GetDescriptionString() const { return description[_desc]; }
        const auto &                        GetFormat() const { return _format; }
        const auto &                        GetOffset() const { return _offset; }
        uint32_t                            GetByteSize() const;
    };
    /******************************************************************************
    *    VertexFormat
    ******************************************************************************/
    constexpr auto KING_MAX_ATTRIBUTES_VERTEXFORMAT = 8;
    
    class alignas(16) VertexFormat
    {
    public:
        VertexAttrib    attributes[KING_MAX_ATTRIBUTES_VERTEXFORMAT];
        uint16_t        nextAttribute = 0;
    public:
        VertexFormat() = default;
        VertexFormat(const VertexFormat & otherIn) { for (uint16_t i = 0; i < KING_MAX_ATTRIBUTES_VERTEXFORMAT; ++i) attributes[i] = otherIn.attributes[i]; nextAttribute = otherIn.nextAttribute; } // copy assign

        void Destroy() { nextAttribute = 0; }

        bool                    Has(VertexAttrib::enumDesc descIn) const;
        bool                    IsFirst(VertexAttrib::enumDesc descIn) const;

        uint16_t                SetNext(VertexAttrib::enumDesc descIn, VertexAttrib::enumFormat formatIn);

        uint32_t                GetByteSize() const;
        uint32_t                GetAttributeByteStart(uint16_t indexIn) const;
        auto                    GetAttribute(uint16_t indexIn) const { return attributes[indexIn]; }
        uint16_t                GetAttributeIndexFromDescription(VertexAttrib::enumDesc descIn) const; // error returns UINT16_MAX 
        auto                    GetAttributeDescriptionFromIndex(uint16_t indexIn) const { return attributes[indexIn]._desc; }
        auto                    GetNumAttributes() const { return nextAttribute; }
    };
    /******************************************************************************
    *    IndexFormat
    ******************************************************************************/
    enum IndexFormat
    {
        euint32 = 1
    };
    /******************************************************************************
    *    Pose - a pose transformation used in animations of verticies
    *    also used as an object's position and orientation
    *    References:    https://en.wikipedia.org/wiki/Orthogonal_matrix
    ******************************************************************************/
    class alignas(16) Pose
    {
        /* structures */
        /* variables */
    public:
    protected:
        float3                              _scale = g_XMOne3;
        quat                                _rotation; // defaults to unit quaternion
        float3                              _rotationOrigin; // defaults to zero components
        float3                              _translation; // defaults to zero components
        /* methods */
    public:
        // Creation/Life cycle
        Pose() = default;
        Pose(const Pose &M) { *this = M; } // involke copy assignment operator
        Pose(Pose &&M) noexcept { *this = std::move(M); } // involke move assignment operator
        explicit Pose(const DirectX::XMMATRIX &M4x4); // involke conversion copy assignment operator
        explicit Pose(const DirectX::XMFLOAT4X4 &F4x4); // involke conversion copy assignment operator
        explicit Pose(const quat rotate, const float3 translate) : _rotation(rotate), _translation(translate) {}
        explicit Pose(const quat rotate, const float3 translate, const float3 scale) : _rotation(rotate), _translation(translate), _scale(scale) { ; }
        explicit Pose(quat rotate) : _rotation(rotate) {}
        explicit Pose(float3 translate) : _translation(translate) {}
        explicit Pose(const XMMATRIX &M3x3, float3 translate);
        virtual ~Pose() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Pose*>(p)); }
        inline Pose & operator= (const Pose &in) // copy assign
        {
            _rotationOrigin = in._rotationOrigin;
            _rotation = in._rotation;
            _scale = in._scale;
            _translation = in._translation;
            return *this;
        }
        inline Pose & operator= (Pose &&in) noexcept// move assign
        {
            if (this != &in)
            {
                _rotationOrigin = std::move(_rotationOrigin);
                _rotation = std::move(in._rotation);
                _scale = std::move(in._scale);
                _translation = std::move(in._translation);
            }
            return *this;
        }
        // note: Animate.Update(deltaTime) will (-) values, (*) by float deltaTime, and then += deltaValue
        inline Pose & operator= (const DirectX::XMMATRIX &M); // conversion copy assignment
        inline Pose & operator= (const DirectX::XMFLOAT4X4 &A); // conversion copy assignment
        inline Pose operator+ (const Pose &in) const
        {
            Pose rtn;
            rtn.SetRotationOrigin(in._rotationOrigin + _rotationOrigin);
            rtn.SetScale(King::Lerp(_scale,in._scale, float3(0.5f))); // average
            rtn.SetRotation(in._rotation + _rotation); // non-communtative, order matters
            rtn.SetTranslation(_translation + in._translation);
            return rtn;
        }
        inline Pose operator- (const Pose &in) const
        {
            Pose rtn;
            rtn.SetRotationOrigin(_rotationOrigin - in._rotationOrigin);
            rtn.SetScale(King::Lerp(_scale, in._scale, float3(0.5f))); // average
            rtn.SetRotation(in._rotation - _rotation); // non-communtative, order matters
            rtn.SetTranslation((float3)_translation - (float3)in._translation);
            return rtn;
        }
        inline Pose operator* (const Pose &in) const
        {
            Pose rtn;
            rtn.SetRotationOrigin(_rotation * in._rotationOrigin + _rotationOrigin);
            rtn.SetScale(_scale * in._scale);
            rtn.SetRotation(in._rotation * _rotation); // non-communtative, order matters
            rtn.SetTranslation(_rotation * in._translation + _translation);
            return rtn;
        }
        inline Pose & operator+= (const Pose &in) { *this = *this + in; return *this; }
        inline Pose & operator-= (const Pose &in) { *this = *this - in; return *this; }
        inline Pose & operator*= (const Pose &in) { *this = *this * in; return *this; }
        inline Pose operator~ () const;
        // rotate
        inline Pose operator* (const quat &in);
        inline Pose & operator*= (const quat &in);
        // transform
        inline Pose operator* (const float &in) const
        {
            Pose rtn;
            //rtn.SetScale(_scale * in);
            rtn.SetRotation(_rotation * in);
            rtn.SetTranslation(_translation * in);
            return rtn;
        }
        inline Pose & operator*= (const float &in) { *this = *this * in; return *this; }
        // translate
        inline Pose operator+ (const float3 &in);
        inline Pose & operator+= (const float3 &in);
        inline Pose operator- (const float3 &in);
        inline Pose & operator-= (const float3 &in);
        // vector transformation
        inline float3 operator* (float3 vec) const;
        inline float4 operator* (float4 vec) const;
        // Conversions
        inline operator DirectX::XMMATRIX() const { return Get_XMMATRIX(); }
        inline operator DirectX::XMFLOAT4X4() const { return Get_XMFLOAT4X4(); }
        // Functionality
        Pose                                Interpolate(const Pose &in, const float fractionIn) const; // common slang is Lerp, short for linear interpolation
        // Accessors
        inline auto&                        GetScale() { return _scale; }
        inline auto&                        GetRotationOrigin() { return _rotationOrigin; }
        inline auto&                        GetRotation() { return _rotation; }
        inline auto&                        GetTranslation() { return _translation; }
        // constant
        inline const auto&                  GetScale() const { return _scale; }
        inline const auto&                  GetRotation() const { return _rotation; }
        inline const auto&                  GetRotationOrigin() const { return _rotationOrigin; }
        inline const auto&                  GetTranslation() const { return _translation; }

        inline DirectX::XMMATRIX            Get_XMMATRIX() const { auto rtn = DirectX::XMMatrixAffineTransformation(_scale, _rotationOrigin, _rotation, _translation.GetVecConst()); assert(!XMMatrixIsNaN(rtn)); return rtn; }
        inline DirectX::XMFLOAT4X4          Get_XMFLOAT4X4() const { DirectX::XMFLOAT4X4 rtn; DirectX::XMStoreFloat4x4(&rtn, DirectX::XMMatrixAffineTransformation(_scale, _rotationOrigin, _rotation, _translation.GetVecConst())); return rtn; }
        // Assignments
        inline void                         Set(const quat rotation, const float3 scale, const float3 translation) { _rotation = rotation; _scale = scale; _translation = translation; }
        inline void                         SetFromMatrix(DirectX::XMMATRIX m) { DirectX::XMMatrixDecompose(&_scale.v, &_rotation.v, &_translation.v, m); _rotationOrigin.SetZero(); }
        inline void                         SetRotation(const quat rotation) { _rotation = rotation; _rotation.Validate(); }
        inline void                         SetRotationOrigin(const float3 rotationOrigin) { _rotationOrigin = rotationOrigin; }
        inline void                         SetRotationAboutXYZAxis(const float radiansX, const float radiansY, const float radiansZ) { _rotation = DirectX::XMQuaternionRotationRollPitchYaw(radiansX, radiansY, radiansZ); }
        inline void                         SetRotationAboutXAxis(const float &radians) { _rotation = DirectX::XMQuaternionRotationRollPitchYaw(radians, 0.0f, 0.0f); }
        inline void                         SetRotationAboutYAxis(const float &radians) { _rotation = DirectX::XMQuaternionRotationRollPitchYaw(0.0f, radians, 0.0f); }
        inline void                         SetRotationAboutZAxis(const float &radians) { _rotation = DirectX::XMQuaternionRotationRollPitchYaw(0.0f, 0.0f, radians); }
        inline void                         SetScale(const float3 scale) { _scale = scale; }
        inline void                         SetTranslation(const float3 translation) { _translation = translation;  }
        // Input & Output
        friend std::ostream& operator<< (std::ostream &os, const Pose &in);
        friend std::istream& operator>> (std::istream &is, Pose &out);
        friend std::wostream& operator<< (std::wostream &os, const Pose &in);
        friend std::wistream& operator>> (std::wistream &is, Pose &out);
        friend void to_json(json& j, const Pose & from);
        friend void from_json(const json& j, Pose & to);
    };
    std::ostream& operator<< (std::ostream &os, const King::Pose &in);
    std::istream& operator>> (std::istream &is, King::Pose &out);
    std::wostream& operator<< (std::wostream &os, const King::Pose &in);
    std::wistream& operator>> (std::wistream &is, King::Pose &out);
    void to_json(json& j, const Pose & from);
    void from_json(const json& j, Pose & to);

    /******************************************************************************
    *    Collidable - Describes which 3D objects may interact.  Detection
    *    is implemented in each object class.  By making this pure virtual,
    *    we are enforcing a policy of which basic geometry will be supported by
    *    all classes.
    ******************************************************************************/
    class Collidable
    {
    protected:
        bool isSleeping = false; // physics simulation use
    public:
        virtual bool        Collision(Collidable const& in) const = 0; // for double dispatch

        virtual bool        Collision(Point const& pointIn) const = 0;
        virtual bool        Collision(Ray const& rayIn) const = 0;
        virtual bool        Collision(Line const& lineIn) const = 0;
        virtual bool        Collision(Plane const& planeIn) const = 0;
        virtual bool        Collision(Sphere const& sphereIn) const = 0;
        virtual bool        Collision(Capsule const& capsuleIn) const = 0;
        virtual bool        Collision(Box const& boxIn) const = 0;
        virtual bool        Collision(Frustum const& frustumIn) const = 0;
        virtual bool        Collision(Pyramid const& pyramidIn) const = 0;
 
        inline const bool   IsSleeping() const { return isSleeping; }
        inline void         Sleep() { isSleeping = true; }
        inline void         WakeUp() 
                            {  
                                // check so a break point can be placed here to explore what woke up the object
                                if (isSleeping)
                                    isSleeping = false;
                            }
    };
    /******************************************************************************
    *    Point - Combines our position data type
    *       with our collidable type so that positions are extended to interact
    *       as geometry with other geometry.
    ******************************************************************************/
    class alignas(16) Point : public Position, Collidable
    {
        /* variables */
    public:

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Point>    Create() { return std::make_shared<Point>(); }
        static std::unique_ptr<Point>    CreateUnique() { return std::make_unique<Point>(); }
        // Construction/Destruction
        Point() = default;
        Point(const float3 &locationIn) : Position(locationIn) { ; }
        explicit inline Point(const Distance &distanceIn) : Position(distanceIn) { ; }
        explicit inline Point(const DirectX::XMVECTOR &vIn) : Position(vIn) { ; }
        explicit inline Point(const DirectX::XMVECTORF32 &vIn) : Position(vIn) { ; }
        explicit inline Point(const FloatPoint4 &ptIn) : Position(ptIn) { ; }
        explicit inline Point(const Point &in) { *this = in; } // forward to copy assignment
        explicit inline Point(Point &&in) noexcept { *this = std::move(in); } // forward to move assignment
        explicit Point(std::initializer_list<float3> il) { assert(il.size() == 1); Set(*il.begin()); }

        virtual ~Point() = default;

        // Conversions
        inline explicit operator float() const { return King::float3::Magnitude(float3(v)); }
        inline explicit operator UnitOfMeasure::Length() const { return UnitOfMeasure::Length(King::float3::Magnitude(float3(v))); }
        inline explicit operator DirectX::XMVECTOR() const { return GetVecConst(); }
        inline operator float3() const { return float3(v); } // allow implicit for a default behavior
        inline explicit operator FloatPoint4() const { return FloatPoint4(v, 1.0f); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Point*>(p)); }
        inline Point & operator= (const XMVECTOR &in) { Position::operator=(in); return *this; } // copy assignment
        //inline Point & operator= (const XMVECTORF32 &in) { Position::operator=(in); return *this; } // copy assignment
        inline Point & operator= (const float3 &in) { Position::operator=(in); return *this; } // copy assignment
        inline Point & operator= (const FloatPoint4 &in) { Position::operator=(in); return *this; } // copy assignment
        inline Point & operator= (const Point &in) { Position::operator=(in); return *this; } // copy assignment
        inline Point & operator= (Point &&in) noexcept { Position::operator=(std::move(in)); return *this; } // move assignment

        inline Point operator- (const Point &rhs) const { Point ptOut(static_cast<float3>(*this) - static_cast<float3>(rhs)); return ptOut; }
        inline Point operator+ (const Point &rhs) const { Point ptOut(static_cast<float3>(*this) + static_cast<float3>(rhs)); return ptOut; }
        inline Point operator* (const Point &rhs) const { Point ptOut(static_cast<float3>(*this) * static_cast<float3>(rhs)); return ptOut; }
        inline Point operator/ (const Point &rhs) const { Point ptOut(static_cast<float3>(*this) / static_cast<float3>(rhs)); return ptOut; }
        inline Point & operator-= (const Point &rhs) { *this = *this - rhs; return *this; }
        inline Point & operator+= (const Point &rhs) { *this = *this + rhs; return *this; }
        inline Point & operator*= (const Point &rhs) { *this = *this * rhs; return *this; }
        inline Point & operator/= (const Point &rhs) { *this = *this / rhs; return *this; }

        inline Point operator- (const Distance &rhs) const { Point ptOut(static_cast<float3>(*this) - static_cast<float3>(rhs)); return ptOut; }
        inline Point operator+ (const Distance &rhs) const { Point ptOut(static_cast<float3>(*this) + static_cast<float3>(rhs)); return ptOut; }
        inline Point operator* (const Distance &rhs) const { Point ptOut(static_cast<float3>(*this) * static_cast<float3>(rhs)); return ptOut; }
        inline Point operator/ (const Distance &rhs) const { Point ptOut(static_cast<float3>(*this) / static_cast<float3>(rhs)); return ptOut; }
        inline Point & operator-= (const Distance &rhs) { *this = *this - rhs; return *this; }
        inline Point & operator+= (const Distance &rhs) { *this = *this + rhs; return *this; }
        inline Point & operator*= (const Distance &rhs) { *this = *this * rhs; return *this; }
        inline Point & operator/= (const Distance &rhs) { *this = *this / rhs; return *this; }

        inline Point operator- (const float3 &rhs) const { Point ptOut(static_cast<float3>(*this) - (rhs)); return ptOut; }
        inline Point operator+ (const float3 &rhs) const { Point ptOut(static_cast<float3>(*this) + (rhs)); return ptOut; }
        inline Point operator* (const float3 &rhs) const { Point ptOut(static_cast<float3>(*this) * (rhs)); return ptOut; }
        inline Point operator/ (const float3 &rhs) const { Point ptOut(static_cast<float3>(*this) / (rhs)); return ptOut; }
        inline Point & operator-= (const float3 &rhs) { *this = *this - rhs; return *this; }
        inline Point & operator+= (const float3 &rhs) { *this = *this + rhs; return *this; }
        inline Point & operator*= (const float3 &rhs) { *this = *this * rhs; return *this; }
        inline Point & operator/= (const float3 &rhs) { *this = *this / rhs; return *this; }

        inline Point & operator*= (const DirectX::XMMATRIX &m) { *this = float3(DirectX::XMVector4Transform(static_cast<FloatPoint4>(*this), m)); return *this; }
        friend Point operator* (Point lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)

        // Functionality
        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch  
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        virtual bool                        Collision(Plane const& planeIn) const override;
        virtual bool                        Collision(Capsule const& capsuleIn) const override;
        virtual bool                        Collision(Frustum const& frustumIn) const override { return false; }
        virtual bool                        Collision(Pyramid const& pyramidIn) const override;

        inline bool                         Intersects(const float3& pointIn) const;
        inline bool                         Intersects(const Point& pointIn) const;
        // Assignments
        // I/O
        friend std::ostream& operator<< (std::ostream& os, const Point& in);
        friend std::istream& operator>> (std::istream& is, Point& out);
        friend std::wostream& operator<< (std::wostream& os, const Point& in);
        friend std::wistream& operator>> (std::wistream& is, Point& out);
        friend void to_json(json& j, const Point& from);
        friend void from_json(const json& j, Point& to);
    };
    //std::ostream& operator<< (std::ostream& os, const King::Point& in);
    //std::istream& operator>> (std::istream& is, King::Point& out);
    //std::wostream& operator<< (std::wostream& os, const King::Point& in);
    //std::wistream& operator>> (std::wistream& is, King::Point& out);
    //void to_json(json& j, const Point& from);
    //void from_json(const json& j, Point& to);

    /******************************************************************************
    *    Line - A single 3D line segment
    *    pt1 ------- pt2
    ******************************************************************************/
    class alignas(16) Line : public Collidable
    {
        /* variables */
    public:
        float3 pt[2];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Line>    Create() { return std::make_shared<Line>(); }
        static std::unique_ptr<Line>    CreateUnique() { return std::make_unique<Line>(); }
        // Construction/Destruction
        Line() = default;
        Line(const Line &in) { *this = in; } // forward to copy assignment
        Line(Line &&in) noexcept { *this = std::move(in); } // forward to move assignment
        Line(const float3 &pt1In, const float3 &pt2In) { pt[0] = pt1In; pt[1] = pt2In; }
        explicit Line(std::initializer_list<float3> il) { assert(il.size() == 2); pt[0] = *il.begin(); pt[1] = *il.end(); }

        virtual ~Line() = default;

        // Conversions
        inline operator float3() const { return pt[1] - pt[0]; } // allow implicit for a default behavior
        explicit inline operator Distance() const { return Distance(pt[0], pt[1]); } // allow implicit for a default behavior
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Line*>(p)); }
        Line & operator= (const Line &in) = default; // copy assignment
        Line & operator= (Line &&in) = default; // move assignment
        inline Line & operator*= (const DirectX::XMMATRIX &m) { pt[0] = DirectX::XMVector4Transform(FloatPoint4(pt[0], 1.0f), m); pt[1] = DirectX::XMVector4Transform(FloatPoint4(pt[1], 1.0f), m); return *this; }
        friend Line operator* (Line lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        Quad operator* (const Distance& rhs) const;
        // Comparators
        inline bool operator==  (const Line& rhs) { return pt[0] == rhs.pt[0] && pt[1] == rhs.pt[1]; }
        inline bool operator!=  (const Line& rhs) { return (pt[0] != rhs.pt[0] || pt[1] != rhs.pt[1]); }
        // Functionality
        void                                Offset(const float distIn, Plane offsetPlaneIn); // modifies the line

        inline bool __vectorcall            Intersects(const Point &pointIn) const;
        inline bool __vectorcall            Intersects(const Line &lineIn, float3 *intersectionOut = nullptr) const;
        inline bool __vectorcall            Intersects(const Triangle& triangleIn, float3* intersectionOut = nullptr) const;
        inline bool __vectorcall            Intersects(const Quad& quadIn, float3* intersectionOut = nullptr) const;
        float3 __vectorcall                 FindNearestPointOnLineSegment(const float3 &pointIn) const;
        // Accessors
        const auto &                        GetVertex(const uint32_t vertexIndexIn) const { return pt[vertexIndexIn]; }
        float                               GetLength() const { return float3::Magnitude(pt[1] - pt[0]); }
        float3                              GetLengthVector() const { return (pt[1] - pt[0]); }
        float3                              GetDirectionFrom0to1() const { float3 rtn(pt[1] - pt[0]); rtn.MakeNormalize(); return rtn; }
        float3                              GetDirectionFrom1to0() const { float3 rtn(pt[0] - pt[1]); rtn.MakeNormalize(); return rtn; }
        float3                              GetCenter() const { return (pt[0] + pt[1]) * 0.5f; }
        float3                              GetPerpendicular() const; // infinte solutions, returns only 1 semi-optimized. By rotating this vector about our line, you may retrieve all solutions
        float3 __vectorcall                 GetPerpendicular(float3 firstPlaneIn) const; // two solutions, to get the other negate the vector returned
        // Assignments
        inline void __vectorcall            Set(const float3 pt1In, const float3 pt2In) { pt[0] = pt1In; pt[1] = pt2In; }
        inline void __vectorcall            Set(const Line &in) { pt[0] = in.pt[0]; pt[1] = in.pt[1]; }
        inline void __vectorcall            SetVertex(const uint32_t vertexIndexIn, const float3 &in) { pt[vertexIndexIn] = in; }

        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        virtual bool                        Collision(Plane const& planeIn) const override;
        virtual bool                        Collision(Capsule const& capsuleIn) const override;
        virtual bool                        Collision(Frustum const& frustumIn) const override { return false; }
        virtual bool                        Collision(Pyramid const& pyramidIn) const override;
    };
    /******************************************************************************
    *    LineIndexed - Vertex indicies representing a connected line segment
    *        references an external buffer as an array of byte sequences as described
    *        by vertexFormat. Note: this is a helper data class to enable
    *        convienent handling. Not intended to store and use data outside of
    *        a temporary instance case.
    *    pt[0] ------- pt[1]
    ******************************************************************************/
    class alignas(16) LineIndexed
    {
        /* variables */
    public:
        uint32_t pt[2]; // index points to vertex buffer
    private:
        uint8_t *vb = nullptr; // vertex buffer referenced
        VertexFormat vertexFormat;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<LineIndexed>    Create() { return std::make_shared<LineIndexed>(); }
        static std::unique_ptr<LineIndexed>    CreateUnique() { return std::make_unique<LineIndexed>(); }
        // Construction/Destruction
        LineIndexed() = default;
        LineIndexed(const LineIndexed &in) { *this = in; } // forward to copy assignment
        LineIndexed(LineIndexed &&in) noexcept { *this = std::move(in); } // forward to move assignment
        LineIndexed(const uint8_t *vbIn, const VertexFormat &vertexFormatIn, const uint32_t &pt1In, const uint32_t &pt2In) { pt[0] = pt1In; pt[1] = pt2In; }

        virtual ~LineIndexed() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<LineIndexed*>(p)); }
        LineIndexed & operator= (const LineIndexed &in) = default; // copy assignment
        LineIndexed & operator= (LineIndexed &&in) = default; // move assignment
        // Comparators
        inline bool operator==  (const LineIndexed& rhs) { return pt[0] == rhs.pt[0] && pt[1] == rhs.pt[1]; }
        inline bool operator!=  (const LineIndexed& rhs) { return (pt[0] != rhs.pt[0] || pt[1] != rhs.pt[1]); }
        // Functionality
        inline bool __vectorcall            Intersects(Line &lineIn, float3 *intersectionOut = nullptr) { assert(vb != nullptr); Line(float3(GetVertex(0)),float3(GetVertex(1))).Intersects(lineIn, intersectionOut); }
        float3 __vectorcall                 FindNearestPointOnLineSegment(const float3 &pointIn) { assert(vb != nullptr); return Line(float3(GetVertex(0)), float3(GetVertex(1))).FindNearestPointOnLineSegment(pointIn); }
        // Accessors
        uint8_t *                           GetVertex(const uint32_t indexIn) const { return vb + pt[indexIn] * vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        float3                              GetVertexPosition(const uint32_t indexIn) const;
        // Assignments
        inline void __vectorcall            Set(const LineIndexed &in) { pt[0] = in.pt[0]; pt[1] = in.pt[1]; }
        inline void                         SetVertex(const uint32_t vertexIndexIn, const uint32_t &in) { assert(vertexIndexIn <2); pt[vertexIndexIn] = in; }
        // Assignments
        void                                Set(uint8_t *vbIn, const VertexFormat &vertexFormatIn, const uint32_t &pt1In, const uint32_t &pt2In) { vb = vbIn; vertexFormat = vertexFormatIn; pt[0] = pt1In; pt[1] = pt2In; }
        void                                SetVB(uint8_t *vbIn) { vb = vbIn; }
        void                                SetVertexFormat(VertexFormat vertexFormatIn) { vertexFormat = vertexFormatIn; }
        void                                SetIndex(const uint32_t &pt1In, const uint32_t &pt2In) { pt[0] = pt1In; pt[1] = pt2In; }
    };
    /******************************************************************************
    *   Path - A 3D path made up of Positions (x,y,z, 1.0f)
    *   Can transform all the positions at once with a matrix
    *   front ------- pt2------- back
    ******************************************************************************/
    class alignas(16) Path : public std::vector<Position>
    {
        /* variables */
    public:

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Path>    Create() { return std::make_shared<Path>(); }
        static std::unique_ptr<Path>    CreateUnique() { return std::make_unique<Path>(); }
        // Construction/Destruction
        Path() = default;
        explicit Path(const Polygon2DF& polyIn) { for (const auto& pt : polyIn._pt) { push_back(Position(pt)); } }
        Path(const Path &in) { *this = in; } // forward to copy assignment
        Path(Path &&in) noexcept { *this = std::move(in); } // forward to move assignment
        explicit Path(std::initializer_list<float3> il) { clear(); reserve(il.size()); for (const auto& ea : il) { push_back(ea); } }

        virtual ~Path() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Path*>(p)); }
        Path & operator= (const Path &in) = default; // copy assignment
        Path & operator= (Path &&in) = default; // move assignment
        inline Path& operator-= (float3 in) { for (Position& pt : *this) pt -= in; return *this; }
        friend Path operator- (Path lhs, const float3 in) { lhs -= in; return lhs; } // invokes std::move(lhs)
        inline Path& operator+= (float3 in) { for (Position& pt : *this) pt += in; return *this; }
        friend Path operator+ (Path lhs, const float3 in) { lhs += in; return lhs; } // invokes std::move(lhs)
        inline Path& operator*= (const DirectX::XMMATRIX &m) { for(Position& pt : *this) pt.Set( DirectX::XMVector4Transform(pt.GetVecConst(), m) ); return *this; }
        friend Path operator* (Path lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Conversions
        inline operator King::Polygon2DF() const { return Get_Polygon2DF(); }
        Polygon2DF                          Get_Polygon2DF() const;
        // Functionality
        void                                Offset(const float dist, Plane offsetPlaneIn);
        // Accessors
        // Assignments
        // Input & Output
        friend std::ostream& operator<< (std::ostream& os, const Path& in);
    };
    std::ostream& operator<< (std::ostream& os, const King::Path& in);
    /******************************************************************************
    *    Ray
    *    origin *---> direction (normalized)
    ******************************************************************************/
    class alignas(16) Ray : public Collidable
    {
        /* variables */
    public:
        float3 origin;
        float3 direction; // normalized on input

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Ray>    Create() { return std::make_shared<Ray>(); }
        static std::unique_ptr<Ray>    CreateUnique() { return std::make_unique<Ray>(); }
        // Construction/Destruction
        Ray() = default;
        Ray(const Ray &in) { *this = in; } // forward to copy assignment
        Ray(Ray &&in) noexcept { *this = std::move(in); } // forward to move assignment
        Ray(const float3 &originIn, const float3 &directionIn) { origin = originIn; direction = directionIn; direction.MakeNormalize(); }
        explicit Ray(std::initializer_list<float3> il) { assert(il.size() == 2); origin = *il.begin(); direction = *il.end(); }

        virtual ~Ray() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Ray*>(p)); }
        Ray & operator= (const Ray &in) = default; // copy assignment
        Ray & operator= (Ray &&in) = default; // move assignment
        inline Ray & operator*= (const DirectX::XMMATRIX &m) { origin = float3(DirectX::XMVector4Transform(static_cast<DirectX::XMVECTOR>(origin), m)); return *this; }
        friend Ray operator* (Ray lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Functionality
        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Plane const& planeIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Capsule const& capsuleIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        virtual bool                        Collision(Frustum const& frustumIn) const override { return false; }
        virtual bool                        Collision(Pyramid const& pyramidIn) const override;

        bool                                Intersects(const Point& pointIn) const;
        bool                                Intersects(const Ray& rayIn, Position *ptOut) const;
        bool                                Intersects(const Line& lineIn, Position* ptOut) const;
        bool __vectorcall                   Intersects(const Triangle& quadIn, float3* intersectionOut = nullptr) const;
        bool __vectorcall                   Intersects(const Quad& quadIn, float3* intersectionOut = nullptr) const;

        float3                              FindNearestPoint(const Point& lineIn) const;
        // Accessors
        float3 const &                      GetDirection() const { return direction; }
        float3 const &                      GetOrigin() const { return origin; }
        // Assignments
        void                                SetDirection(const float3 & directionIn) { direction = directionIn; direction.MakeNormalize(); }
        void                                SetOrigin(const float3& originIn) { origin = originIn; }
        // I/O
        friend std::ostream& operator<< (std::ostream& os, const Ray& in);
        friend std::istream& operator>> (std::istream& is, Ray& out);
        friend std::wostream& operator<< (std::wostream& os, const Ray& in);
        friend std::wistream& operator>> (std::wistream& is, Ray& out);
        friend void to_json(json& j, const Ray& from);
        friend void from_json(const json& j, Ray& to);
    };
    //std::ostream& operator<< (std::ostream& os, const King::Ray& in);
    //std::istream& operator>> (std::istream& is, King::Ray& out);
    //std::wostream& operator<< (std::wostream& os, const King::Ray& in);
    //std::wistream& operator>> (std::wistream& is, King::Ray& out);
    //void to_json(json& j, const Ray& from);
    //void from_json(const json& j, Ray& to);

    /******************************************************************************
    *    Plane
    *      \---------\
    *       \(origin) \
    *        \    *-----> (normal)
    *         \---------\
    *    w component keeps the scalar where orgin = -(normal) * s
    *    sx'+ sy'+ sz' + s = 0 ; homogeneous plane equation
    ******************************************************************************/
    class alignas(16) Plane : public FloatPoint4, public Collidable
    {
    public:
        // Construction/Destruction
        inline explicit Plane() { v = DirectX::XMVectorZero(); }
        inline explicit Plane(const float distFromOrigin, const float3 normalToPlane) { v = DirectX::XMPlaneNormalizeEst(normalToPlane); v = DirectX::XMVectorSetW(v, distFromOrigin); } // should distFromOrigin be negated?
        explicit Plane(const Position origin, const float3 normalToPlane);
        explicit Plane(const Position& point1, const Position& point2, const Position& point3); // input points should be CCW
        inline explicit Plane(float x, float y, float z, float s) { v = FloatPoint4(x,y,z,s); } // should distFromOrigin be negated?
        inline Plane(const DirectX::XMVECTOR vec) { v = vec; }
        inline Plane(const FloatPoint4 in) { v = in; }
        inline Plane(const Plane &in) { v = in; } // copy 
        inline Plane(Plane &&in) noexcept { v = std::move(in); } // move
        virtual ~Plane() = default;
        // Operators 
        Plane & operator= (const Plane &in) = default; // copy assignment
        Plane & operator= (Plane &&in) = default; // move assignment
        inline Plane & operator= (const FXMVECTOR &in) { v = in; return *this; }
        inline Plane & operator*= (const DirectX::XMMATRIX &m) { v = DirectX::XMPlaneTransform(v, m); return *this; }
        inline Plane operator* (const DirectX::XMMATRIX &m) { return Plane(DirectX::XMPlaneTransform(v, m)); }
        friend Plane operator* (Plane lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        explicit operator bool() const { return !DirectX::XMVector4IsNaN(v); } // valid
        bool operator !() const { return DirectX::XMVector4IsNaN(v); } // invalid
        // Comparators
        inline bool operator==  (const Plane &rhs) { return DirectX::XMVector4Equal(v, rhs.v); }
        inline bool operator!=  (const Plane &rhs) { return DirectX::XMVector4NotEqual(v, rhs.v); }
        // Functionality
        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Plane const& planeIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Capsule const& capsuleIn) const override { return false; }
        virtual bool                        Collision(Box const& boxIn) const override;
        virtual bool                        Collision(Frustum const& frustumIn) const override { return false; }
        virtual bool                        Collision(Pyramid const& pyramidIn) const override;

        inline bool __vectorcall            Intersects(const float3 pointIn) const;
        inline bool __vectorcall            Intersects(const Point &pointIn) const;
        bool                                Intersects(const Line& lineIn, float3* ptOut) const;
        inline bool __vectorcall            Intersects(const Line& lineIn, Point *ptOut) const;
        inline bool __vectorcall            Intersects(const Ray& rayIn, Point* ptOut) const;
        bool __vectorcall                   Intersects(const Plane& planeIn, Line* lineOut) const;
        inline bool __vectorcall            Intersects(const Sphere &sphereIn) const;
        inline bool __vectorcall            Intersects(const Triangle& triIn, float3* intersectionOut) const;
        inline bool __vectorcall            Intersects(const Box& boxIn) const;
        float3 __vectorcall                 FindNearestPointOnPlane(const float3& pointIn) const { auto n = GetNormal(); return pointIn - Dot(pointIn, n) * n; }
        // Accessors
        float3                              GetNormal(void) const { return float3(*this); }
        float3                              GetOrigin(void) const { return -float3(*this) * GetW(); } // w keeps the scalar to translate unit vector to origin

        float                               DistanceFromPoint(float3 point) const { return Dot(point, GetNormal()) + GetW(); } // projects the point onto the plane and adds the perpendicular distance of the origin
        float                               DistanceFromPoint(Point point) const { return Dot(static_cast<FloatPoint4>(point), v); } // homogeneous (ax,ay,az,a) if a=1
    private:
        // Helpers
        bool                                FindIntersectionWithEdge(const float3& v0, const float3& v1, float3& intersectionOut) const;
    };
    /******************************************************************************
    *    TriangleIndexed
    *        references an external buffer as an array of byte sequences as described
    *        by vertexFormat.
    *            edge2
    *       pt0 ------- pt2
    *           \     /
    *     edge1  \   / edge3
    *             \ /
    *             pt1
    *    CCW rotation for RHS face normals
    ******************************************************************************/
    class alignas(16) TriangleIndexed
    {
        /* variables */
    public:
        uint32_t pt[3]; // index points to vertex buffer
    private:
        uint8_t *vb = nullptr; // vertex buffer
        VertexFormat vertexFormat;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<TriangleIndexed>    Create() { return std::make_shared<TriangleIndexed>(); }
        static std::unique_ptr<TriangleIndexed>    CreateUnique() { return std::make_unique<TriangleIndexed>(); }
        // Construction/Destruction
        TriangleIndexed() = default;
        TriangleIndexed(const TriangleIndexed &in) { *this = in; } // forward to copy assignment
        TriangleIndexed(TriangleIndexed &&in) { *this = std::move(in); } // forward to move assignment
        TriangleIndexed(uint8_t *vbIn, const VertexFormat vertexFormatIn, const uint32_t &pt1In, const uint32_t &pt2In, const uint32_t &pt3In) { Set(vbIn, vertexFormatIn, pt1In, pt2In, pt3In); }

        virtual ~TriangleIndexed() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<TriangleIndexed*>(p)); }
        TriangleIndexed & operator= (const TriangleIndexed &in) = default; // copy assignment
        TriangleIndexed& operator= (TriangleIndexed&& in) noexcept { *this = in; return *this; } // copy assign instead as we do not own the buffer
        // Functionality
        bool __vectorcall                   Intersects(const Ray &rayIn, float3 *intersectPointOut) { return Intersects(rayIn.origin,rayIn.direction, intersectPointOut); }
        bool __vectorcall                   Intersects(const float3 &rayOriginIn, const float3 &rayDirectionIn, float3 *intersectPointOut);
        // Accessors
        float3                              GetFaceNormal() { assert(vb != nullptr); return float3::CrossProduct(GetEdge1(), GetEdge2()); }
        float3                              GetEdge1() { assert(vb != nullptr); return GetVertexPosition(1) - GetVertexPosition(0); }
        float3                              GetEdge2() { assert(vb != nullptr); return GetVertexPosition(2) - GetVertexPosition(0); }
        float3                              GetEdge3() { assert(vb != nullptr); return GetVertexPosition(2) - GetVertexPosition(1); }
        
        uint8_t *                           GetVertex(const uint32_t indexIn) const { return vb + pt[indexIn] * vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        float3                              GetVertexPosition(const uint32_t indexIn) const;
        // Assignments
        void                                Set(uint8_t *vbIn, const VertexFormat &vertexFormatIn, const uint32_t &pt1In, const uint32_t &pt2In, const uint32_t &pt3In) { vb = vbIn; vertexFormat = vertexFormatIn; pt[0] = pt1In; pt[1] = pt2In; pt[2] = pt3In; }
        void                                SetVB(uint8_t *vbIn) { vb = vbIn; }
        void                                SetVertexFormat(VertexFormat vertexFormatIn) { vertexFormat = vertexFormatIn; }
        void                                SetIndex(const uint32_t &pt1In, const uint32_t &pt2In, const uint32_t &pt3In) { pt[0] = pt1In; pt[1] = pt2In; pt[2] = pt3In; }
    };
    /******************************************************************************
    *    Triangle
    *            edge2
    *       pt0 ------- pt2
    *           \     /
    *     edge1  \   / edge3
    *             \ /
    *             pt1
    *    CCW rotation for RHS face normals
    ******************************************************************************/
    class alignas(16) Triangle
    {
        /* variables */
    private:
        float3    pt[3];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Triangle>    Create() { return std::make_shared<Triangle>(); }
        static std::unique_ptr<Triangle>    CreateUnique() { return std::make_unique<Triangle>(); }
        // Construction/Destruction
        Triangle() = default;
        Triangle(const Triangle &in) { *this = in; } // forward to copy assignment
        Triangle(Triangle &&in) noexcept { *this = std::move(in); } // forward to move assignment
        Triangle(TriangleIndexed &in) { pt[0] = in.GetVertex(0); pt[1] = in.GetVertex(1); pt[2] = in.GetVertex(2); }
        explicit Triangle(const float3 &pt1In, const float3 &pt2In, const float3 &pt3In) { pt[0] = pt1In; pt[1] = pt2In; pt[2] = pt3In; }
        // Construction Initializer
        Triangle(std::initializer_list<float3> il) { assert(il.size() <= 3); size_t count = 0; for (auto & each : il) { pt[count] = each; ++count; } }

        virtual ~Triangle() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Triangle*>(p)); }
        Triangle & operator= (const Triangle &in) = default; // copy assignment
        Triangle & operator= (Triangle &&in) = default; // move assignment
        // Functionality
        bool __vectorcall                   Contains(const Point& ptIn) const;

        bool __vectorcall                   Intersects(const Ray& rayIn, float3* intersectPointOut) const { const float3 o = rayIn.origin; const float3 d = rayIn.direction; return Intersects(o, d, intersectPointOut); }
        bool __vectorcall                   Intersects(const float3 &rayOriginIn, const float3 &rayDirectionIn, float3 *intersectPointOut) const;
        bool __vectorcall                   Intersects(const Line& lineIn, float3* intersectionOut) const { return lineIn.Intersects(*this, intersectionOut); }
        bool __vectorcall                   Intersects(const Triangle& triIn, float3* intersectionOut) const;
        bool __vectorcall                   Intersects(const Quad& qIn, float3* intersectionOut) const;
        bool __vectorcall                   Intersects(const Plane& pIn, float3* intersectionOut) const;
        bool __vectorcall                   Intersects(const Sphere& sIn, float3* intersectionOut) const;

        float3 __vectorcall                 FindNearestPoint(const float3& pt3In) const;
        // Accessors
        [[deprecated("Use GetNormalCCW() or GetNormalCW()")]] float3 GetFaceNormal() { return float3::CrossProduct(GetEdge1(), GetEdge2()); }
        float3                              GetNormalCCW() const { float3 c(Cross(GetEdge1(), GetEdge2())); c.MakeNormalize(); return c; } // RHS
        float3                              GetNormalCW() const { float3 c(Cross(GetEdge2(), GetEdge1())); c.MakeNormalize(); return c; } // RHS
        float3                              GetEdge1() const { return pt[1] - pt[0]; }
        float3                              GetEdge2() const { return pt[2] - pt[0]; }
        float3                              GetEdge3() const { return pt[2] - pt[1]; }
        float3                              GetCenter() const { float3 c = pt[0] + pt[1] + pt[2]; c *= 0.3333333f; return c; } // centroid

        auto &                              GetVertex(const int vertNumber_0_1_2) const { return pt[vertNumber_0_1_2]; }
        // Assignments
        inline void _vectorcall             Set(const float3 pt1In, const float3 pt2In, const float3 pt3In) { pt[0] = pt1In; pt[1] = pt2In; pt[2] = pt3In; }
    };
    /******************************************************************************
    *    Quad
    *            edge4                   Alternate Triangles (Alt1, Alt2)
    *       pt0 -------> pt2              --------
    *           | T1 / |                  | \ T2 |
    *     edge1 |  /   | edge3            |   \  |
    *           V/  T2 V                  | T1  \|
    *       pt1 -------> pt3              --------
    *             edge2
    ******************************************************************************/
    class alignas(16) Quad
    {
        /* variables */
    public:
        float3    pt[4];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Quad>    Create() { return std::make_shared<Quad>(); }
        static std::unique_ptr<Quad>    CreateUnique() { return std::make_unique<Quad>(); }
        // Construction/Destruction
        Quad() = default;
        Quad(const Quad &in) { *this = in; } // forward to copy assignment
        Quad(Quad &&in) noexcept { *this = std::move(in); } // forward to move assignment
        Quad(const float3 &pt0In, const float3 &pt1In, const float3 &pt2In, const float3 &pt3In) { pt[0] = pt0In; pt[1] = pt1In; pt[2] = pt2In; pt[3] = pt3In; }
        Quad(const DirectX::XMFLOAT3 &pt0In, const DirectX::XMFLOAT3 &pt1In, const DirectX::XMFLOAT3 &pt2In, const DirectX::XMFLOAT3 &pt3In) { pt[0] = pt0In; pt[1] = pt1In; pt[2] = pt2In; pt[3] = pt3In; }
        Quad(const float3 *pArrayIn) { assert(pArrayIn); pt[0] = pArrayIn[0]; pt[1] = pArrayIn[1]; pt[2] = pArrayIn[2]; pt[3] = pArrayIn[3]; }
        Quad(const DirectX::XMFLOAT3 *pArrayIn) { assert(pArrayIn); pt[0] = pArrayIn[0]; pt[1] = pArrayIn[1]; pt[2] = pArrayIn[2]; pt[3] = pArrayIn[3]; }
        Quad(const Line edge4, Line edge2) : Quad(edge4.GetVertex(0), edge2.GetVertex(0), edge4.GetVertex(1), edge2.GetVertex(1)) {}
        Quad(std::initializer_list<float3> il) { assert(il.size() <= 4); size_t count = 0; for (auto & each : il) { pt[count] = each; ++count; } }

        virtual ~Quad() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Quad*>(p)); }
        Quad & operator= (const Quad &in) = default; // copy assignment
        Quad & operator= (Quad &&in) = default; // move assignment
        // Functionality
        bool                                Intersects(const Ray& ray, float3* intersectPointOut = nullptr);
        bool                                Intersects(const float3 &rayOriginIn, const float3 &rayDirectionIn, float3 *intersectPointOut);
        bool __vectorcall                   Intersects(const Triangle& triIn, float3* intersectionOut) const;

        void                                MoveBy(const float3 deltaIn) { for(int i=0;i<4;++i) pt[i]+=deltaIn; }
        std::vector<Quad>                   SubDivide();

        void                                Scale(const float scaleIn) { for (int i = 0; i<4; ++i) pt[i] *= scaleIn; }
        void                                Scale(const float3 scaleIn) { for (int i = 0; i<4; ++i) pt[i] *= scaleIn; }
        // Accessors
        float3                              GetNormalCCW() const { float3 c(Cross(GetEdge1(), GetEdge4())); c.MakeNormalize(); return c; } // RHS, // note: Cross is LHS for DirectX, swapped the terms for RHS
        float3                              GetNormalCW() const { float3 c(Cross(GetEdge4(), GetEdge1())); c.MakeNormalize(); return c; } // RHS
        float3                              GetVertex(const unsigned int& i) const { assert(i<4); return pt[i]; }
        float3                              GetCenter() const { float3 r = pt[0] + pt[3]; r *= 0.5f; return r; }
        float3                              GetEdge1() const { return pt[1] - pt[0]; }
        float3                              GetEdge2() const { return pt[3] - pt[1]; }
        float3                              GetEdge3() const { return pt[3] - pt[2]; }
        float3                              GetEdge4() const { return pt[2] - pt[0]; }
        float3                              GetSeam() const { return pt[2] - pt[1]; }
        float3                              GetSeamAlt() const { return pt[3] - pt[0]; }
        auto                                GetTriangle1() const { return Triangle(pt[0], pt[1], pt[2]); }
        auto                                GetTriangle2() const { return Triangle(pt[2], pt[1], pt[3]); }
        auto                                GetTriangleAlt1() const { return Triangle(pt[0], pt[1], pt[3]); }
        auto                                GetTriangleAlt2() const { return Triangle(pt[3], pt[2], pt[0]); }
        // Assignments
        void                                Set(Quad &in) { for(int i=0;i<4;++i) pt[i]=in.pt[i]; }
    };
    /******************************************************************************
    *    Sphere
    *
    ******************************************************************************/
    class alignas(16) Sphere : public FloatPoint4, public Collidable
    {
    public:
        // Construction/Destruction
        inline Sphere() { FloatPoint4::Set(float3(), abs(1.0f)); }
        inline Sphere(const float radius) { FloatPoint4::Set(float3(), abs(radius)); }
        inline Sphere(const float3 center, const float radius = 1.0f) { FloatPoint4::Set(center, abs(radius)); }
        Sphere(const float x, const float y, const float z, const float radius) { FloatPoint4::Set(x, y, z, abs(radius)); }

        inline explicit Sphere(const DirectX::XMVECTOR &vec) { v = vec; }
        inline explicit Sphere(const FloatPoint4 in) { v = in; }
        inline Sphere(const Sphere &in) { v = in; } // copy 
        inline Sphere(Sphere &&in) noexcept { v = std::move(in); } // move
        virtual ~Sphere() = default;
        // Operators 
        Sphere & operator= (const Sphere &in) = default; // copy assignment
        Sphere & operator= (Sphere &&in) = default; // move assignment
        //inline Sphere& operator= (const Box& in) { v = FloatPoint4(in.GetCenter(), in.GetRadius()); return *this; }
        inline Sphere & operator= (const FXMVECTOR &in) { v = in; return *this; }
        inline Sphere& operator*= (const DirectX::XMMATRIX& m) { auto r = GetW(); v = DirectX::XMVector4Transform(FloatPoint4(v, 1.0f), m); SetW(r* DirectX::XMVectorGetX(m.r[0])); return *this; } //supports uniform scaling by using x axis only but could be non-uniformed
        friend Sphere operator* (Sphere lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Comparators
        inline bool operator==  (const Sphere &rhs) { return DirectX::XMVector4Equal(v, rhs.v); }
        inline bool operator!=  (const Sphere &rhs) { return DirectX::XMVector4NotEqual(v, rhs.v); }
        // Functionality
        DirectX::XMMATRIX                   MomentsOfInertia(const float& densityIn); // same as if rotated due to symetry in x, y, and z axis

        bool                                Contains(const Point& ptIn) const;
        bool                                Intersects(const Ray& rhs, float3 *ptOut) const;
        bool                                Intersects(const Line& rhs) const;
        bool                                Intersects(const Triangle& triIn, float3* intersectionOut) const;
        bool                                Intersects(const Plane& rhs) const;
        bool                                Intersects(const Sphere& rhs) const;
        bool                                Intersects(const Box& boxIn, const Quaternion& orientationIn) const;

        float3 __vectorcall                 FindNearestPointFromSphere(const float3& pt3In) const;
                
        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Plane const& planeIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Capsule const& capsuleIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        virtual bool                        Collision(Frustum const& frustumIn) const override;
        virtual bool                        Collision(Pyramid const& pyramidIn) const override { return false; }

        void                                Merge(const Sphere& sIn); // grow the sphere to match the min and max extents of each sphere
        // Accessors
        virtual float3                      GetCenter() const { return float3(v); }
        float                               GetRadius() const { return abs(GetW()); } // since it can be set directly, the only guarantee (+) is to abs each call
        inline float                        GetSurfaceArea() const { auto r = GetRadius(); return 4.0f * UnitOfMeasure::PI * r * r; }
        inline float                        GetVolume() const { auto r = GetRadius(); return 4.0f / 3.0f * UnitOfMeasure::PI * r * r * r; }
                                                                                      // Assignments
        void __vectorcall                   SetCenter(const float3 centerIn) { float r = GetRadius(); v = FloatPoint4(centerIn, r); }
        void                                SetCenter(const float x, const float y, const float z) { float r = GetRadius(); v = DirectX::XMVectorSet(x, y, z, r); }
        void                                SetRadius(const float radiusIn) { SetW(abs(radiusIn)); }
        // I/O
        friend std::ostream& operator<< (std::ostream& os, const King::Sphere& in);
        friend std::istream& operator>> (std::istream& is, Sphere& out);
        friend void to_json(json& j, const Sphere& from);
        friend void from_json(const json& j, Sphere& to);
    };
    std::ostream& operator<< (std::ostream& os, const King::Sphere& in);
    std::istream& operator>> (std::istream& is, King::Sphere& out);
    std::wostream& operator<<(std::wostream& os, const King::Sphere& in);
    void to_json(json& j, const Sphere& from);
    void from_json(const json& j, Sphere& to);
    /******************************************************************************
    *    Capsule
    * 
    *    /---------\
    *   (           )  = ((((((((((()((((((((((() = a line segment of infinite spheres
    *    \---------/
    *
    ******************************************************************************/
    class alignas(16) Capsule : public Collidable
    {
    public:
        float           radius = 0.f;
        Line            segment;

    public:
        // Construction/Destruction
        inline Capsule() = default;
        inline Capsule(const float3 startIn, const float3 endIn, const float radiusIn) : segment(startIn,endIn), radius(fabsf(radiusIn)) { ; }
        inline Capsule(const Line& lineIn, const float radiusIn) : segment(lineIn), radius(fabsf(radiusIn)) { ; }

        Capsule(const Capsule& in) : radius(in.radius), segment(in.segment) { ; } // copy 
        Capsule(Capsule&& in) noexcept { radius = std::move(in.radius); segment = std::move(in.segment); } // move
        virtual ~Capsule() = default;
        // Operators 
        inline Capsule& operator= (const Capsule& in) = default; // copy assignment
        inline Capsule& operator= (Capsule&& in) = default; // move assignment
        inline Capsule& operator*= (const DirectX::XMMATRIX& m) { segment *= m; return *this; }
        friend Capsule operator* (Capsule lhs, const DirectX::XMMATRIX& m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Comparators
        inline bool operator==  (const Capsule& rhs) { return (radius == rhs.radius && segment == rhs.segment); }
        inline bool operator!=  (const Capsule& rhs) { return (radius != rhs.radius || segment != rhs.segment); }
        // Functionality
        /* To test collision against a capsule, you must find the closest point to it's line segment to whatever it is you are testing, then construct a spehre at that point. Then it becomes a whatever / sphere test.*/
        bool                                Intersects(const Point& ptIn) const;
        bool                                Intersects(const Ray& rayIn, Ray* intersectAndNormal1Out, Ray* intersectAndNormal2Out) const;
        bool                                Intersects(const Line& lineIn) const;
        //bool                                Intersects(const Plane& planeIn) const { return false; }
        bool                                Intersects(const Sphere& sphereIn) const;
        bool                                Intersects(const Box& boxIn) const;

        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Plane const& planeIn) const override { return false; }
        virtual bool                        Collision(const Sphere& sphereIn) const override;
        virtual bool                        Collision(Capsule const& capsuleIn) const override { return false; }
        virtual bool                        Collision(Box const& boxIn) const override;
        virtual bool                        Collision(Frustum const& frustumIn) const override { return false; }
        virtual bool                        Collision(Pyramid const& pyramidIn) const override;

        // Accessors
        Line                                GetSegment() const { return segment; }
        float                               GetRadius() const { return radius; }
        // Assignments
        void __vectorcall                   SetSegment(const Line lineIn) { segment = lineIn; }
        void                                SetRadius(const float radiusIn) { radius = radiusIn; }
        // I/O
/*        friend std::ostream& operator<< (std::ostream& os, const King::Capsule& in);
        friend std::istream& operator>> (std::istream& is, Capsule& out);
        friend void to_json(json& j, const Capsule& from);
        friend void from_json(const json& j, Capsule& to)*/;
    };
    //std::ostream& operator<< (std::ostream& os, const King::Capsule& in);
    //std::istream& operator>> (std::istream& is, King::Capsule& out);
    //std::wostream& operator<<(std::wostream& os, const King::Capsule& in);
    //void to_json(json& j, const Capsule& from);
    //void from_json(const json& j, Capsule& to);
    /******************************************************************************
    *    Box
    *        An axis-aligned box with transforms for non-algined box.
    *        Must pass in transforms and returns orientated points out.
    *        Defaults to a unit box center on (0,0,0) with WHD (1,1,1)
    *
    *          4------7       x=width
    *         /      /|       y=height      +y
    *        /      /6 rtf    z=depth        ^   ^ (-z RHS)
    *       5------|.pt_max                  |  /
    *   lbb | 0.   | /3                      | /
    *pt_min |      |/                        |/
    *       1-------2                        ---------> +x
    *    Note: offsets are (x+,y+,z+) from lbb (min) to rtf (max) in RHS
    ******************************************************************************/
    class alignas(16) Box : public Collidable
    {
        /* variables */
    public:
        float3        pt_min = { -0.5f, -0.5f, -0.5f }; // left, back, bottom (min in RHS)
        float3        pt_max = { +0.5f, +0.5f, +0.5f }; // right, top, front (max in RHS)
        
        enum CornerDescription { lbb = 0, lbf, rbf, rbb, ltb, ltf, rtf, rtb, INVALID };
    protected:

    private:

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Box>    Create() { return std::make_shared<Box>(); }
        static std::unique_ptr<Box>    CreateUnique() { return std::make_unique<Box>(); }
        // Construction/Destruction
        Box() = default;
        Box(const float3& extents) { SetCenterAndExtents(float3(), extents); }
        // Box(const float3& pt) : pt_min(pt), pt_max(pt) { ; }
        Box(const float3& p1, const float3& p2) : pt_min(Min(p1, p2)), pt_max(Max(p1, p2)) { ; }
        Box(const Box& b1, const Box& b2) : pt_min(Min(b1.pt_min, b2.pt_min)), pt_max(Max(b1.pt_max, b2.pt_max)) { ; }
        Box(const Box &in) { *this = in; } // forward to copy assignment
        Box(Box &&in) noexcept { *this = std::move(in); } // forward to move assignment
        Box(const DirectX::BoundingBox &in) { Set(in); }

        virtual ~Box() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Box*>(p)); }
        Box & operator= (const Box &in) { Set(in.GetMin(), in.GetMax()); return *this; } // copy assignment
        Box & operator= (Box &&in) = default; // move assignment
        inline Box& operator*= (const DirectX::XMMATRIX& m) { assert(false); }/*this is broken, collapes boxes on rotation !!!!, must use full box  SetAABBfromThisTransformedBox(FXMMATRIX M);*/ //pt_min = DirectX::XMVector4Transform(FloatPoint4(pt_min, 1.0f), m); pt_max = DirectX::XMVector4Transform(FloatPoint4(pt_max, 1.0f), m); return *this; } // does not support scaling since non-uniformed
        friend Box operator* (Box lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        inline Box& operator+= (const float3 d) { MoveBy(d); return *this; }
        inline Box& operator-= (const float3 d) { MoveBy(-d); return *this; }
        inline Box& operator*= (const float3 s) { Grow(s); return *this; }
        inline Box operator+ (const float3 d) const { Box rtn(*this); rtn.MoveBy(d); return rtn; }
        inline Box operator- (const float3 d) const { Box rtn(*this); rtn.MoveBy(-d); return rtn; }
        inline Box operator* (const float3 s) const { Box rtn(*this); rtn.Grow(s); return rtn; }
        // Conversions
        inline explicit operator bool() const { return (!DirectX::XMVector3IsNaN(pt_min) && !DirectX::XMVector3IsNaN(pt_max)); } // valid
        inline bool operator !() const { return (DirectX::XMVector3IsNaN(pt_min) || DirectX::XMVector3IsNaN(pt_max)); } // invalid
        // Functionality
        bool                                Intersects(const Ray &rayIn, float &distOut) const;
        bool                                Intersects(const Line &lineIn, float3* intersectionOut) const;
        bool                                Intersects(const Triangle& rhs, float3* intersectionOut) const;
        bool                                Intersects(const Plane& planeIn) const;
        bool                                Intersects(const Box &boxIn) const;
        bool                                Intersects(const Sphere &sphereIn) const;
        bool                                Intersects(const Pyramid& pyramidIn, float3* intersectionOut) const;

        bool                                Contains(const Box &boxIn) const; // boxIn is wholely inside of this box
        bool __vectorcall                   Contains(const float3 ptIn) const;

        void                                Constrain(float3 *ptOut) const; // constrain point to our box

        float3 __vectorcall                 FindNearestPointOnBox(const float3 &pt3In) const;
        float3 __vectorcall                 FindNearestPointOnBox(const float3 &pt3In, const DirectX::FXMMATRIX &M) const;

        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Plane const& planeIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Capsule const& capsuleIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        virtual bool                        Collision(Frustum const& frustumIn) const override;
        virtual bool                        Collision(Pyramid const& pyramidIn) const override { return false; }

        Box                                 CollisionVolume(const Box& boxIn); // over lapping volume with boxIn

        DirectX::XMMATRIX                   MomentsOfInertia(const float& densityIn = 700.0f);
        DirectX::XMMATRIX                   MomentsOfInertiaRotated(const DirectX::XMMATRIX& Iin, const DirectX::XMMATRIX& rotationIn);

        inline void                         MoveTo(const float3 &pt2In) { float3 offset(pt_max - pt_min); pt_min = pt2In; pt_max = pt_min + offset; }
        inline bool                         MoveTo(const float &xIn, const float &yIn, const float &zIn) { MoveTo(float3(xIn, yIn, zIn)); }
        inline void                         MoveBy(const float3 &deltaIn) { pt_min += deltaIn; pt_max += deltaIn; }
        inline void                         MoveBy(const Distance& deltaIn) { MoveBy(float3(deltaIn)); }
        inline void                         MoveBy(const float &dxIn, const float &dyIn, const float &dzIn) { float3 delta(dxIn, dyIn, dzIn); MoveBy(delta); }
        inline void                         MoveByX(const float &dxIn) { pt_min.SetX(pt_min.GetX() + dxIn); pt_max.SetX(pt_max.GetX() + dxIn); }
        inline void                         MoveByY(const float &dyIn) { pt_min.SetY(pt_min.GetY() + dyIn); pt_max.SetY(pt_max.GetY() + dyIn); }
        inline void                         MoveByZ(const float &dzIn) { pt_min.SetZ(pt_min.GetZ() + dzIn); pt_max.SetZ(pt_max.GetZ() + dzIn); }

        inline void                         Merge(const Box& bIn); // grow the box to match the min and max extents of each box
        inline void __vectorcall            Merge(const float3 ptIn); // grow the box to contain the point
        inline void __vectorcall            Grow(const float3 scale3In) { auto s = scale3In * GetSize(); SetWHD(s); }
        // Accessors
        inline float3                       GetSize() const { return Abs(pt_max - pt_min); } // width, height & depth
        inline float3                       GetExtents() const { return (pt_max - pt_min) * 0.5f; }
        inline float3                       GetCenter() const { return (pt_max + pt_min) * 0.5f; }
        inline float                        GetRadius() const { auto r3 = Abs(pt_max - pt_min) * 0.5f; float r = max(r3.GetX(), r3.GetY()); return max(r,r3.GetZ()); }

        inline const float3 &               GetMin() const { return pt_min; }
        inline const float3 &               GetMax() const { return pt_max; }
        inline float                        GetWidth() const { return std::abs(pt_max.GetX() - pt_min.GetX()); }
        inline float                        GetHeight() const { return std::abs(pt_max.GetY() - pt_min.GetY()); }
        inline float                        GetDepth() const { return std::abs(pt_max.GetZ() - pt_min.GetZ()); }
        inline float3                       GetDiagonal() const { return pt_max - pt_min; }
        inline float                        GetSurfaceArea() const { auto d = GetDiagonal(); return 2.f * (d.f[0] * d.f[1] + d.f[0] * d.f[2] + d.f[1] * d.f[2]); }
        inline float                        GetVolume() const { auto d = GetDiagonal(); return 2.f * (d.f[0] * d.f[1] * d.f[2]); }

        std::vector<std::pair<float3, CornerDescription>>  IdentifyCorners(const vector<float3>& pointsIn, const Quaternion* quaternionIn = nullptr) const;          
        DirectX::XMFLOAT4*                  GetCorners8Transformed(const DirectX::FXMMATRIX& M); // scaled, rotate and translate a "unit" Box to a model bounding box
        std::vector<float3>                 GetCorners8ObjectSpace(const Quaternion* quaternionIn = nullptr) ; // return all 8 corners with enum CornerDescription indexing
        std::vector<float3>                 GetCorners8(const Quaternion * quaternionIn = nullptr) const; // return all 8 corners with enum CornerDescription indexing
        std::vector<float3>                 GetCornersTop4(const Quaternion * quaternionIn = nullptr) const; 
        std::vector<float3>                 GetCornersBottom4(const Quaternion * quaternionIn = nullptr) const;
        std::vector<float3>                 GetCornersLowest4(const Quaternion * quaternionIn = nullptr) const;
        void                                GetCorners8ObjectSpace(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr) ; // array size passed in is [8], axis aligned
        void                                GetCorners8(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr) const; // array size passed in is [8], axis aligned
        void                                GetCornersTop4(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr) const; // array size passed in is [4]
        void                                GetCornersBottom4(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr) const; // array size passed in is [4]
        void                                GetCornersLowest4(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr) const; // array size passed in is [4]

        inline Quad                         GetQuadFront(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[8]; GetCorners8(a, quaternionIn); return Quad(a[ltf], a[lbf], a[rbf], a[rtf] ); }
        inline Quad                         GetQuadRight(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[8]; GetCorners8(a, quaternionIn); return Quad(a[rtf], a[rbf], a[rbb], a[rtb]); }
        inline Quad                         GetQuadBack(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[8]; GetCorners8(a, quaternionIn); return Quad(a[rtb], a[rbb], a[lbb], a[ltb]); }
        inline Quad                         GetQuadLeft(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[8]; GetCorners8(a, quaternionIn); return Quad(a[ltb], a[lbb], a[lbf], a[ltf]); }
        inline Quad                         GetQuadTop(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[4]; GetCornersTop4(a, quaternionIn); return Quad(a); }
        inline Quad                         GetQuadBottom(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[4]; GetCornersBottom4(a, quaternionIn); return Quad(a); }
        //// Assignments                      
        inline void                         SetZero() { pt_min.SetZero(); pt_max.SetZero(); }
        inline void __vectorcall            Set(const float3 pt_minIn, const float3 pt_maxIn) { assert(pt_minIn); assert(pt_maxIn); pt_min = Min(pt_minIn, pt_maxIn); pt_max = Max(pt_minIn, pt_maxIn); } // defines the box with min (pt_min) and max (pt_max) points in RHS
        inline void                         Set(const DirectX::BoundingBox &in) { float3 c = DirectX::XMLoadFloat3(&in.Center); float3 e = DirectX::XMLoadFloat3(&in.Extents); pt_min = c - e; pt_max = c + e; }
        inline void __vectorcall            SetCenterAndExtents(const float3 cIn, const float3 eIn) { pt_min = cIn - eIn; pt_max = cIn + eIn; } // defines the box with min (pt_min) and max (pt_max) points in RHS
        inline void __vectorcall            SetWHD(const float3 whdIn) { float3 offset = Abs(whdIn) * 0.5f; float3 c = GetCenter(); pt_min = c - offset; pt_max = c + offset; }
        inline void __vectorcall            SetCenter(const float3 centerIn) { float3 delta = centerIn - GetCenter(); MoveBy(delta); }
        inline void __vectorcall            SetExtents(const float3 eIn) { const auto c = GetCenter(); const auto e = Abs(eIn); pt_min = c - e; pt_max = c + e; }
        void __vectorcall                   SetAABBfromThisTransformedBox(FXMMATRIX M);
        void __vectorcall                   SetAABBfromBoundingSphere(Sphere s); // orientation does not matter, will be the largest AABB, but quicker than SetAABBfromThisTransformedBox(FXMMATRIX M);
        // I/O
        friend std::ostream& operator<< (std::ostream &os, const King::Box &in);
        friend std::istream& operator>> (std::istream &is, Box &out);
        friend std::wostream& operator<< (std::wostream& os, const King::Box& in);
        friend std::wistream& operator>> (std::wistream& is, King::Box& out);
        friend void to_json(json& j, const Box& from);
        friend void from_json(const json& j, Box& to);
    };
    std::ostream& operator<< (std::ostream &os, const King::Box &in);
    std::istream& operator>> (std::istream &is, King::Box &out);
    std::wostream& operator<<(std::wostream& os, const King::Box& in);
    std::wistream& operator>>(std::wistream& is, King::Box& out);
    void to_json(json& j, const Box& from);
    void from_json(const json& j, Box& to);
    /******************************************************************************
    *    Fustrum
    *
    *    Reference: Patterned after Frustum class in Microsoft mini engine
    ******************************************************************************/
    class alignas(16) Frustum : public Collidable
    {
        /* variables */
    public:
        enum CornerID { lbb = 0, lbf, rbf, rbb, ltb, ltf, rtf, rtb, INVALID };
        enum PlaneID { kNearPlane = 0, kFarPlane, kLeftPlane, kRightPlane, kTopPlane, kBottomPlane, kINVALID };
    protected:
        float3                              _FrustumCorners[8];
        Plane                               _FrustumPlanes[6];
        /* methods */
    public:
        // Construction/Destruction
        Frustum() { ; }
        Frustum(const DirectX::XMMATRIX &projectionMatrixIn);
        Frustum(const Box& in, const quat* rQIn = nullptr) { ConstructOrthographicFrustum(in, rQIn); }

        Frustum(const Frustum &in) { *this = in; } // copy 
        Frustum(Frustum &&in) noexcept { *this = std::move(in); } // move
        virtual ~Frustum() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Frustum*>(p)); }
        Frustum & operator= (const Frustum &in) = default; // copy assignment
        Frustum & operator= (Frustum &&in) = default; // move assignment
        Frustum & operator*= (const DirectX::XMMATRIX &m);
        friend Frustum operator* (Frustum lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Functionality
        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override { return false; }
        virtual bool                        Collision(Ray const& rayIn) const override { return false; }
        virtual bool                        Collision(Line const& lineIn) const override { return false; }
        virtual bool                        Collision(Plane const& planeIn) const override { return false; }
        virtual bool                        Collision(Sphere const& sphereIn) const override { return Intersect(sphereIn); }
        virtual bool                        Collision(Capsule const& capsuleIn) const override { return false; }
        virtual bool                        Collision(Box const& boxIn) const override { return Intersect(boxIn); }
        virtual bool                        Collision(Frustum const& frustumIn) const override { return false; }
        virtual bool                        Collision(Pyramid const& pyramidIn) const override { return false; }
        // *** TO DO *** add a cone collision test, and implement a better frustum culling routine; reference https://www.flipcode.com/archives/Frustum_Culling.shtml *** TO DO ***

        inline bool                         Intersect(const Box& boxIn, const Quaternion* quaternionIn = nullptr) const;
        inline bool __vectorcall            Intersect(const Sphere sphereIn) const;
        // Accessors
        inline const float3&                GetFrustumCorner(const Frustum::CornerID id) const { assert(id >= 0 && id < Frustum::CornerID::INVALID); return _FrustumCorners[id]; }
        inline const Plane&                 GetFrustumPlane(const Frustum::PlaneID id) const { assert(id >= 0 && id < Frustum::PlaneID::kINVALID); return _FrustumPlanes[id]; }
        // Assignments
        inline void                         SetFrustumPlane(const Frustum::PlaneID id, const Plane p) { assert(id >= 0 && id < Frustum::PlaneID::kINVALID); _FrustumPlanes[id] = p; }
    private:
        void                                ConstructPerspectiveFrustum(float HTan, float VTan, float NearClip, float FarClip); // Perspective frustum constructor (for pyramid-shaped frusta)
        void                                ConstructOrthographicFrustum(float Left, float Right, float Top, float Bottom, float NearClip, float FarClip); // Orthographic frustum constructor (for box-shaped frusta from front plane and depth)
        void                                ConstructOrthographicFrustum(const Box& in, const quat *rQIn = nullptr); //  // Orthographic frustum constructor (for box-shaped frusta)
        // I/O
        friend std::ostream& operator<< (std::ostream& os, const King::Frustum& in);
        friend std::istream& operator>> (std::istream& is, Frustum& out);
        friend void to_json(json& j, const Frustum& from);
        friend void from_json(const json& j, Frustum& to);
    };
    std::ostream& operator<< (std::ostream& os, const King::Frustum& in);
    std::istream& operator>> (std::istream& is, King::Frustum& out);
    void to_json(json& j, const Frustum& from);
    void from_json(const json& j, Frustum& to);

    /******************************************************************************
    *    Pyramid
    *       FloatPoint3 is the center of the base of the Pyramid
    ******************************************************************************/
    class alignas(16) Pyramid : public FloatPoint3 //, public Collidable
    {
    protected:
        float baseLength;  // Length of the base of the pyramid
        float height;      // Height of the pyramid

    public:
        // Construction/Destruction
        inline Pyramid() : baseLength(1.0f), height(1.0f) { FloatPoint3::Set(float3()); } // Default: unit pyramid
        inline Pyramid(const float baseLengthIn, const float heightIn) : baseLength(abs(baseLengthIn)), height(abs(heightIn)) { FloatPoint3::Set(float3()); }
        inline Pyramid(const float3 center, const float baseLengthIn, const float heightIn = 1.0f) : baseLength(abs(baseLengthIn)), height(abs(heightIn)) { FloatPoint3::Set(center); }
        Pyramid(const float x, const float y, const float z, const float baseLengthIn, const float heightIn) : baseLength(abs(baseLengthIn)), height(abs(heightIn)) { FloatPoint3::Set(x, y, z); }

        inline Pyramid(const Pyramid& in) = default; // copy
        inline Pyramid(Pyramid&& in) noexcept = default; // move
        virtual ~Pyramid() = default;

        // Operators
        Pyramid& operator= (const Pyramid& in) = default; // copy assignment
        Pyramid& operator= (Pyramid&& in) = default; // move assignment
        inline Pyramid& operator= (const FXMVECTOR& in) { v = in; return *this; }

        inline Pyramid& operator*= (const DirectX::XMMATRIX& m) {
            v = DirectX::XMVector3Transform(v, m);
            baseLength *= DirectX::XMVectorGetX(m.r[0]);  // Apply scaling to base length (assuming scaling along x-axis)
            height *= DirectX::XMVectorGetY(m.r[1]);  // Apply scaling to height (assuming scaling along y-axis)
            return *this;
        }

        friend Pyramid operator* (Pyramid lhs, const DirectX::XMMATRIX& m) { lhs *= m; return lhs; } // invokes std::move(lhs)

        // Comparators
        inline bool operator==  (const Pyramid& rhs) const { return DirectX::XMVector3Equal(v, rhs.v) && baseLength == rhs.baseLength && height == rhs.height; }
        inline bool operator!=  (const Pyramid& rhs) const { return !(*this == rhs); }

        // Functionality
        DirectX::XMMATRIX                   MomentsOfInertia(const float& densityIn = 700.f); // Moments of inertia for a pyramid

        bool                                Contains(const Point& ptIn) const;

        bool                                Intersects(const Ray& rayIn, float3* ptOut) const;
        bool                                Intersects(const Line& lineIn, float3* intersectionOut) const;
        bool                                Intersects(const Triangle& rhs, float3* intersectionOut) const;
        bool                                Intersects(const Quad& rhs, float3* intersectionOut) const;
        bool                                Intersects(const Plane& rhs, float3* intersectionOut) const;
        bool                                Intersects(const Sphere& rhs, float3* intersectionOut) const;
        bool                                Intersects(const Box& boxIn, const Quaternion& orientationIn, float3* intersectionOut) const;
        bool                                Intersects(const Capsule& capsuleIn, float3* intersectionOut) const;
        bool                                Intersects(const Pyramid& rhs, float3* intersectionOut) const;

        float3 __vectorcall                 FindNearestPoint(const float3& pt3In) const;
        float  __vectorcall                 DistanceFromPoint(const float3& point) const;

        //virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        //virtual bool                        Collision(Point const& pointIn) const override;
        //virtual bool                        Collision(Ray const& rayIn) const override;
        //virtual bool                        Collision(Line const& lineIn) const override;
        //virtual bool                        Collision(Plane const& planeIn) const override;
        //virtual bool                        Collision(Sphere const& sphereIn) const override;
        //virtual bool                        Collision(Pyramid const& pyramidIn) const override;
        //virtual bool                        Collision(Capsule const& capsuleIn) const override;
        //virtual bool                        Collision(Box const& boxIn) const override;
        //virtual bool                        Collision(Frustum const& frustumIn) const override { return false; } /* TO DO */

        // Accessors
        virtual float3                      GetCenter() const { return float3(v); } // of base
        float3                              GetCentroid() const { auto c = GetCenter(); c.SetY(c.GetY() + height / 4.0f); }
        float3                              GetApex() const { auto c = GetCenter(); c.SetY(c.GetY() + height); return c; }
        float                               GetBaseLength() const { return baseLength; }
        float                               GetHeight() const { return height; }
        std::vector<float3>                 GetBaseVertices() const;
        Quad                                GetBaseQuad() const;
        std::vector<Triangle>               GetFaces() const;

        void                                SetBaseLength(const float baseLengthIn) { baseLength = abs(baseLengthIn); }
        void                                SetHeight(const float heightIn) { height = abs(heightIn); }

        inline float                        GetSurfaceArea() const { return CalculateSurfaceArea(); }
        inline float                        GetVolume() const { return (1.0f / 3.0f) * baseLength * baseLength * height; }

        // Assignments
        void __vectorcall                   SetCenter(const float3 centerIn) { v = FloatPoint3(centerIn); }
        void                                SetCenter(const float x, const float y, const float z) { v = DirectX::XMVectorSet(x, y, z, 1.0f); }

        // I/O
        //friend std::ostream& operator<< (std::ostream& os, const Pyramid& in);
        //friend std::istream& operator>> (std::istream& is, Pyramid& out);
        //friend void to_json(json& j, const Pyramid& from);
        //friend void from_json(const json& j, Pyramid& to);

    private:
        inline float CalculateSurfaceArea() const {
            float slantHeight = sqrtf((baseLength * 0.5f) * (baseLength * 0.5f) + height * height);
            return baseLength * baseLength + 2 * baseLength * slantHeight; // Base area + lateral surface area
        }
    };
    // Stream operators
    //std::ostream& operator<< (std::ostream& os, const Pyramid& in);
    //std::istream& operator>> (std::istream& is, Pyramid& out);
    //std::wostream& operator<<(std::wostream& os, const Pyramid& in);
    // JSON serialization
    //void to_json(json& j, const Pyramid& from);
    //void from_json(const json& j, Pyramid& to);

    /******************************************************************************
    *    Collided 
    *        Methods to find contacts and keep the information from them
    *        along with the respone impulses. CollisionBehavior is just a  
    *        stored value to be check by handling routines for the collision response.
    *        This must be set by external code that parses collisions into types for
    *        a response by other collision resolver routines.
    * 
    *        Definitions:
    *           none - do nothing
    *           stop - stop, leave the collision in place
    *           repel - adjust positions so not overlapping
    *           rickochet - use impulses to separate collisions which modifies velocities
    *           detonate - something bad to handle
    *           invalid - as it says, this collision type is not liked, could be reporting
    *               an event that should not occur
    ******************************************************************************/
    class alignas(16) Collided
    {
        /* definitions */
    public:
        enum class eCollisionBehavior { none, stop, repel, rickochet, detonate, invalid };
        /* variables */
    public:
        std::weak_ptr<Sphere>           collider1;
        std::weak_ptr<Sphere>           collider2;

        std::vector<DirectX::XMVECTOR>  contactPoints;

        DirectX::XMVECTOR               normal;
        float                           penetrationDepth = 0.f;
        enum class eCollisionBehavior   collisionType = eCollisionBehavior::invalid;

    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr< Collided> Create() { return std::make_shared<Collided>(); }
         Collided() : normal(DirectX::XMVectorZero()), penetrationDepth(0.0f) {};
         Collided(const Collided & in) { *this = in; } // forward to copy assignment
         Collided( Collided&& in) noexcept { *this = std::move(in); } // forward to move assignment
         Collided(const std::weak_ptr<Sphere> c1, const std::weak_ptr<Sphere> c2)
             : collider1(c1), collider2(c2), normal(DirectX::XMVectorZero()), penetrationDepth(0.0f) {}

        virtual ~Collided() { ; }

        // Conversions
        // Operators 
        void* operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void* p) { _aligned_free(static_cast< Collided*>(p)); }
         Collided& operator= (const  Collided& other) = default; // copy assign
         Collided& operator= ( Collided&& other) = default; // move assign
        explicit operator bool() const { return (bool)GetHasContact(); } // valid
        bool operator !() const { return !(bool)GetHasContact(); } // invalid
        // Math Operators
        // Init/Start/Stop/Destroy
        // Functionality
        // Accessors
        const size_t                        GetHasContact() const { return contactPoints.size(); }
        const auto&                         GetCollisionType() const { return collisionType; }
        // Assignments
        void                                SetObjects(std::weak_ptr<Sphere> obj1, std::weak_ptr<Sphere> obj2) { collider1 = obj1; collider2 = obj2; }
        void                                SetCollisionType(eCollisionBehavior type) { collisionType = type; }

        bool                                SphereOnSphereIntersection(const Sphere& A, const Sphere& B);
        bool                                OBBonOBBIntersection(const Box& A, const Quaternion qA, const Box& B, const Quaternion qB, const std::vector<float3>& cornersA, const std::vector<float3>& cornersB);
        // *** TO DO *** bool Set CollidedPointsFromSphereOnOBBIntersection() { ; } // *** TO DO *** https://gamedev.stackexchange.com/questions/163873/separating-axis-theorem-obb-vs-sphere

    protected:
        // Helps to Set CollidedPointsFromOBBonOBBIntersection
        inline std::vector<float3>          GetContactPoints(const std::vector<float3>& corners, const Distance& dist);
        inline std::vector<float3>          ClosestPointsToFaces(std::vector<float3>* contactVertsA, std::vector<float3>* contactVertsB);
        inline float3                       ClosestPointEdgeEdge(const std::vector<float3>& contactVertsA, const std::vector<float3>& contactVertsB); // collapse the edges to a point
        inline bool                         VertInsideFace(const std::vector<float3>& contactPoints, const float3& p0, const float& planeErr = 0.0f);
    };
    /******************************************************************************
    *    LineMesh
    *        Class object that references master data and does not keep data itself.
    *        Uses an index buffer to define lines from a vertex buffer to form
    *        a mesh (connected line segments).
    ******************************************************************************/
    class alignas(16) LineMesh
    {
        /* variables */
    private:
        std::string                         _name;
        VertexFormat                        _vertexFormat;
        uint8_t *                           _vb = nullptr; // reference, do not free
        uint32_t *                          _ib = nullptr; // reference, do not free
        uint32_t                            _vbStart = 0; // vertex buffer start offset (starting vertex #) to add to address
        uint32_t                            _ibStart = 0; // index buffer start offset (starting index #) to add to address
        uint32_t                            _numLines = 0; // assumes ib is a line list with 2 index per triangle

        Box                                 _boundingBox; // calculate and store after data definitions
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<LineMesh> Create() { return std::make_shared<LineMesh>(); }
        static std::shared_ptr<LineMesh> Create(uint32_t numLinesIn, const VertexFormat &vfIn, uint32_t vbStartIn, uint32_t ibStartIn, uint8_t *vbIn, uint32_t *ibIn) { return std::make_shared<LineMesh>(numLinesIn, vfIn, vbStartIn, ibStartIn, vbIn, ibIn); }
        static std::unique_ptr<LineMesh> CreateUnique() { return std::make_unique<LineMesh>(); }
        // Construction/Destruction
        LineMesh() = default;
        LineMesh(const LineMesh &in) { *this = in; } // forward to copy assignment
        LineMesh(LineMesh &&in) { *this = std::move(in); } // forward to move assignment
        LineMesh(uint32_t numLinesIn, const VertexFormat &vfIn, uint32_t vbStartIn, uint32_t ibStartIn, uint8_t *vbIn, uint32_t *ibIn) { Initialize(numLinesIn, vfIn, vbStartIn, ibStartIn, vbIn, ibIn); }

        virtual ~LineMesh() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<LineMesh*>(p)); }
        LineMesh & operator= (const LineMesh &in) = default; // copy assignment
        LineMesh & operator= (LineMesh &&in) = default; // move assignment
        inline LineIndexed operator[] (uint32_t indexIn) const { return GetLineIndexed(indexIn); } // accessor
        // Functionality
        virtual void                        Initialize(uint32_t numLinesIn, const VertexFormat &vfIn, uint32_t vbStartIn, uint32_t ibStartIn, uint8_t *vbIn, uint32_t *ibIn) { _numLines = numLinesIn; _vertexFormat = vfIn; _vbStart = vbStartIn; _ibStart = ibStartIn; _vb = vbIn; _ib = ibIn; }
        virtual Box                         CalculateBoundingBox(); // call after loading or changing position data
        // Accessors
        Box                                 GetBoundingBox() const { return _boundingBox; }
        const auto &                        Get_name() const { return _name; }
        auto                                GetIB() const { return _ib + _ibStart; }
        auto                                GetVB() const { return _vb + (uint32_t)_vbStart * (uint32_t)_vertexFormat.GetByteSize(); }
        const auto &                        GetIBStart() const { return _ibStart; } // # of index to skip for the start of buffer
        const auto &                        GetVBStart() const { return _vbStart; } // # of vertex to skip for the start of buffer
        const VertexFormat &                GetVertexFormat() const { return _vertexFormat; }
        const auto &                        GetNumLines() const { return _numLines; }
        auto                                GetNumIndicies() const { return _numLines * 2; }
        auto                                GetNumVerticies(); // calculates based on max index buffer value

        LineIndexed                         GetLineIndexed(const uint32_t &lineIndexIn) const { auto o = (GetIB() + lineIndexIn * 2); return LineIndexed(GetVB(), _vertexFormat, *o, *(o + 1)); }

        auto                                GetVertexAddr(const uint32_t vertNumIn) const { return GetVB() + vertNumIn * _vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        float3                              GetVertexPosition(const uint32_t vertNumIn) const;
        // Assignments
        void                                Set_name(const string _name_IN) { _name = _name_IN; }
        void                                SetIB(uint32_t *ibIn) { _ib = ibIn; }
        void                                SetVB(uint8_t *vbIn) { _vb = vbIn; }
        void                                SetIBStart(uint32_t ibStartIn) { _ibStart = ibStartIn; } // added to the address of all indicies
        void                                SetVBStart(uint32_t vbStartIn) { _vbStart = vbStartIn; } // added to the address of all verticies
        void                                SetNumLines(uint32_t numLinesIn) { _numLines = numLinesIn; }
        void                                SetNumIndicies(size_t numIndiciesIn) { _numLines = (uint32_t)(numIndiciesIn / 2); }
        void                                SetVertexFormat(const VertexFormat & vertexFormatIn) { _vertexFormat = vertexFormatIn; }
    };
    /******************************************************************************
    *    TriangleMesh
    *        CCW rotation for RHS face normals
    *        Class object that references master data and does not keep data itself.
    *        Uses an index buffer to define triangles from a vertex buffer to form
    *        a mesh.  These buffers are maintained in the model class
    ******************************************************************************/
    class alignas(16) TriangleMesh
    {
        /* variables */
    private:
        // begin file type v1
        std::string                         _name; // name the mesh
        std::string                         _materialName = "default"; // lookup name in the owning model class that keeps the material data
        uint32_t                            _vbStart = 0; // vertex buffer start offset (starting vertex #) to add to address
        uint32_t                            _ibStart = 0; // index buffer start offset (starting index #) to add to address
        uint32_t                            _numTriangles = 0; // assumes ib is a triangle list with 3 index per triangle
        Box                                 _boundingBox; // calculate and store after data definitions
        // end file type v1
        VertexFormat                        _vertexFormat; // same as model class that owns TriangleMesh instance
        uint8_t*                            _vb = nullptr; // reference, do not free, owned by model class that owns TriangleMesh instance
        uint32_t*                           _ib = nullptr; // reference, do not free, owned by model class that owns TriangleMesh instance

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<TriangleMesh> Create() { return std::make_shared<TriangleMesh>(); }
        static std::shared_ptr<TriangleMesh> Create(uint32_t numTrianglesIn, const VertexFormat& vfIn, uint32_t vbStartIn, uint32_t ibStartIn, uint8_t* vbIn, uint32_t* ibIn) { return std::make_shared<TriangleMesh>(numTrianglesIn, vfIn, vbStartIn, ibStartIn, vbIn, ibIn); }
        static std::unique_ptr<TriangleMesh> CreateUnique() { return std::make_unique<TriangleMesh>(); }
        // Construction/Destruction
        TriangleMesh() = default;
        TriangleMesh(const TriangleMesh &in) { *this = in; } // forward to copy assignment
        TriangleMesh(TriangleMesh &&in) noexcept { *this = std::move(in); } // forward to move assignment
        TriangleMesh(uint32_t numTrianglesIn, VertexFormat vfIn, uint32_t vbStartIn, uint32_t ibStartIn, uint8_t *vbIn, uint32_t *ibIn) { Initialize(numTrianglesIn, vfIn, vbStartIn, ibStartIn, vbIn, ibIn); }

        virtual ~TriangleMesh() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<TriangleMesh*>(p)); }
        TriangleMesh & operator= (const TriangleMesh &in) = default; // copy assignment
        TriangleMesh & operator= (TriangleMesh &&in) = default; // move assignment
        inline TriangleIndexed operator[] (uint32_t indexIn) const { return GetTriangleIndexed(indexIn); } // accessor
        // Functionality
        virtual void                        Initialize(uint32_t numTrianglesIn, VertexFormat vfIn, uint32_t vbStartIn, uint32_t ibStartIn, uint8_t *vbIn, uint32_t *ibIn) { _numTriangles = numTrianglesIn; _vertexFormat = vfIn; _vbStart = vbStartIn; _ibStart = ibStartIn; _vb = vbIn; _ib = ibIn; }
        virtual Box                         CalculateBoundingBox(); // call after loading or changing position data
        // I/O
        virtual bool                        Read_v1(ifstream& dataFile);
        virtual bool                        Write_v1(ofstream& dataFile);
        // Accessors
        Box                                 GetBoundingBox() const { return _boundingBox; }
        const auto &                        Get_name() const { return _name; }
        const auto &                        Get_materialName() const { return _materialName; }
        uint32_t *                          GetIB() const { return _ib + _ibStart; }
        uint8_t *                           GetVB() const { return _vb + (uint32_t)_vbStart * (uint32_t)_vertexFormat.GetByteSize(); }
        const auto &                        GetIBStart() const { return _ibStart; } // # of index to skip for the start of buffer
        const auto &                        GetVBStart() const { return _vbStart; } // # of vertex to skip for the start of buffer
        const VertexFormat &                GetVertexFormat() const { return _vertexFormat; }
        const auto &                        GetNumTriangles() const { return _numTriangles; }
        auto                                GetNumIndicies() const { return _numTriangles * 3; }
        uint32_t                            GetNumVerticies() const; // calculates based on max index buffer value

        TriangleIndexed                     GetTriangleIndexed(const uint32_t &triangleIndexIn) const { auto o = (GetIB() + (uint32_t)triangleIndexIn * 3); return TriangleIndexed(GetVB(), _vertexFormat, *o, *(o + 1), *(o + 2)); }
        
        auto                                GetVertexAddr(const uint32_t vertNumIn) const { return GetVB() + vertNumIn * _vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        //auto                                GetVertexAddrByIndexBuffer(const uint32_t indexIn) const { auto c = GetNumIndicies(); assert(indexIn < c); return GetVB() + *(GetIB() + indexIn) * (uint32_t)_vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        float3                              GetVertexPosition(const uint32_t vertNumIn) const;
        // Assignments
        void                                Set_name(const string _name_IN) { _name = _name_IN; }
        void                                Set_materialName(std::string materialNameIn) { _materialName = materialNameIn; }
        void                                SetIB(uint32_t *ibIn) { _ib = ibIn; }
        void                                SetVB(uint8_t *vbIn) { _vb = vbIn; }
        void                                SetIBStart(uint32_t ibStartIn) { _ibStart = ibStartIn; } // added to the address of all indicies
        void                                SetVBStart(uint32_t vbStartIn) { _vbStart = vbStartIn; } // added to the address of all verticies
        void                                SetNumTriangles(uint32_t numTrianglesIn) { _numTriangles = numTrianglesIn; }
        void                                SetNumIndicies(size_t numIndiciesIn) { _numTriangles = (uint32_t)(numIndiciesIn / 3); }
        void                                SetVertexFormat(const VertexFormat & vertexFormatIn) { _vertexFormat = vertexFormatIn; }

        void                                SetVertexPosition(const uint32_t indexIn, float3 posIn);
        // Helpers
        void                                HelperMoveVertexOnGrid(const float3 delta, const unsigned int vertexNumIn, const unsigned int verticiesIn_i, const unsigned int verticiesIn_j);
    };
    /******************************************************************************
    *    HeightGrid
    *    pt1 ------- pt2
    ******************************************************************************/
    class alignas(16) HeightGrid
    {
        /* variables */
    public:
        MemoryBlock<float>                   _gridMem;
        IntPoint2                            _xzSize;
        Box                                  _boundingBox;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<HeightGrid>    Create() { return std::make_shared<HeightGrid>(); }
        static std::unique_ptr<HeightGrid>    CreateUnique() { return std::make_unique<HeightGrid>(); }
        // Construction/Destruction
        HeightGrid() = default;
        HeightGrid(const HeightGrid &in) { *this = in; } // forward to copy assignment
        HeightGrid(HeightGrid &&in) noexcept { *this = std::move(in); } // forward to move assignment
        HeightGrid(const TriangleMesh & triMeshIn, long xIn=0l, long zIn=0l) { InitializeXZ(triMeshIn, xIn, zIn); }

        virtual ~HeightGrid() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<HeightGrid*>(p)); }
        HeightGrid & operator= (const HeightGrid &in) = default; // copy assignment
        HeightGrid & operator= (HeightGrid &&in) = default; // move assignment
        // Functionality
        IntPoint2                           InitializeXZ(const TriangleMesh & triMeshIn, long xIn=0l, long zIn=0l); // return IntPoint2(X,Z) size of height grid allocated
        float                               Height(const float2 & xzWorldIn);
        bool                                Intersects(const float3 & xzWorldIn, float *heightOut = nullptr); // ignores y component, returns true if intersects bounding box and heightOut containing the height at that position
        // Accessors
        // Assignments
        void                                SetXZSize(const IntPoint2 & in) { _xzSize.Set(in); }
        // IO
        bool                                Load(const string & nameIn);
        void                                Save(const string & nameIn);
    };
    /******************************************************************************
    *    Material - not geometry, but very important to its physical properties
    *        Reference: http://paulbourke.net/dataformats/mtl/ format document .mtl
    ******************************************************************************/
    class alignas(16) Material
    {
    public:
        /* structures */
        struct ColorProp
        {
            uint16_t ver = 1;
            float ambient[3] = { 1.f, 1.f, 1.f };
            float diffuse[3] = { 1.f, 1.f, 1.f };
            float specular[3] = { 0.1f, 0.1f, 0.1f };
            float emissive[3] = { 0.f, 0.f, 0.f };
            float transparent[3] = { 0.5f, 0.5f, 0.5f }; // light passing through a transparent surface is multiplied by this filter color
        };
        struct FileNames
        {
            uint16_t ver = 1;
            string light; // ambient occlusion / light map
            string diffuse; // texture map
            string specular; // specular color
            string specular_strength; // multiplier on top of specular color
            string normal; // bump map
            string emmissive;
            string transparency;
            string displacement; // geometry displacement map
            string stencil;
            string reflection;
        };
        struct Properties
        {
            uint16_t ver = 1;
            ColorProp color;
            float indexOfRefraction = 1.f; // 0.001 to 10; 1.0 means that light does not bend, glass has value 1.5
            float specularStrength = 1; // exponent 0 to 1000, shininess
            float dissolved = 1.f; // 0 transparent to 1 opaque (eg. alpha)
            float transmissionFilter = 1.f; // 0 to 1
            bool isCutOut = false; // color (diffuse) is always on, cutout specifies if material has alpha channel (for rendering order)
            bool isAmbient = true; // ambient on/off
            bool isShiny = true; // specular on/off
            bool isEmissive = false; // emissive on/off
        };

        /* variables */
    private:
        // begin file type v1
        uint8_t                             _shaderMethod = 0;
        std::string                         _name = "default"s;
        Properties                          _properties;      
        FileNames                           _fileNames;
        // end file type v1

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Material>    Create() { return std::make_shared<Material>(); }
        static std::shared_ptr<Material>    Create(const std::string &nameIn) { return std::make_shared<Material>(nameIn); }
        Material() { ; }
        Material(const std::string &nameIn) : _name(nameIn) { ; }
        Material(const Material &in) { *this = in; } // forward to copy assignment
        Material(Material &&in) noexcept { *this = std::move(in); } // forward to move assignment
        virtual ~Material() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Material*>(p)); }
        Material & operator= (const Material &in) = default; // copy assign
        Material & operator= (Material &&in) = default; // move assign
        // Conversions
        // Functionality
        // I/O
        bool                                Read_v1(ifstream& dataFile);
        bool                                Write_v1(ofstream& dataFile);
        // Accessors
        const auto &                        Get_name() const { return _name; }
        const auto &                        Get_properties() const { return _properties; }
        const auto &                        Get_fileNames() const { return _fileNames; }
        const auto &                        Get_shaderMethod() const { return _shaderMethod; }
        // Assignments
        void                                Set_name(const string _name_IN) { _name = _name_IN; }
        void                                Set_properties(const struct Properties &_properties_IN) { _properties = _properties_IN; }
        void                                Set_fileNames(const struct FileNames &_fileNames_IN) { _fileNames = _fileNames_IN; }
        void                                Set_shaderMethod(const uint8_t method_IN) { _shaderMethod = method_IN; }
    };
    /******************************************************************************
    *    ModelScaffold
    *   Base class for models such as King::LineModel and King::Model (TriangleModel)
    *   
    *   Remarks: This class keeps master data, sub-meshes in derived classes
    *   will reference the data kept in this master data.
    ******************************************************************************/
    class alignas(16) ModelScaffold : public std::enable_shared_from_this<King::ModelScaffold> // base class
    {
         /* variables */
    protected:
        // begin file type v1
        std::string                         _modelName; // should be unique for each model
        static size_t                       s_modelNameCounter; // enable unique before loads or programmer name change; defined in 3DGeometry.cpp

        VertexFormat                        _vertexFormat; // byte size of format is stride size for _vertexBufferMaster
        IndexFormat                         _indexFormat = IndexFormat::euint32;

        Box                                 _boundingBox;

        MemoryBlock<uint8_t>                _vertexBufferMaster; // set stride of buffer to vertexFormat size after format is initialized
        MemoryBlock<uint32_t>               _indexBufferMaster; // 4,294,967,295 maximum verticies with 32 bit indexing
        // end file type v1

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<ModelScaffold>    Create() { return std::make_shared<ModelScaffold>(); }
        // Construction/Destruction
        ModelScaffold() { SetModelName(std::to_string(s_modelNameCounter++)); /*_vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3); _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());*/ }
        ModelScaffold(const Line& lineIn) { *this = lineIn; } // forward to copy assignment
        ModelScaffold(const Box& boxIn) { *this = boxIn; } // forward to copy assignment
        ModelScaffold(const ModelScaffold &in) { *this = in; } // forward to copy assignment
        ModelScaffold(ModelScaffold &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~ModelScaffold() { Destroy(); }

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<ModelScaffold*>(p)); }
        ModelScaffold& operator= (const Line& lineIn);
        ModelScaffold& operator= (const Box& boxIn);
        ModelScaffold & operator= (const ModelScaffold &in) = default; // copy assignment
        ModelScaffold & operator= (ModelScaffold &&in) = default; // move assignment
        // Functionality
        virtual void                        Initalize(const VertexFormat &vertexFormatIn, size_t verticiesIn, size_t indiciesIn) { _vertexFormat = vertexFormatIn; _vertexBufferMaster.Initialize(verticiesIn * _vertexFormat.GetByteSize()); _indexBufferMaster.Initialize(indiciesIn); }
        virtual void                        Destroy() { _indexBufferMaster.Destroy(); _vertexBufferMaster.Destroy(); _vertexFormat.Destroy(); _boundingBox.SetZero(); }

        virtual bool                        Load(std::string fileNameIN) { return false; }
        virtual bool                        Save(std::string fileNameIN) { return false; }

        virtual bool                        Read_v1(ifstream& dataFile);
        virtual bool                        Write_v1(ofstream& dataFile);

        Position                            ConvertWorldPointToObjectSpace(const Position& worldPointIn, const DirectX::XMMATRIX worldMatrixIn);
        Ray                                 ConvertWorldRayToObjectSpace(const Ray& worldRayIn, const DirectX::XMMATRIX worldMatrixIn);

        virtual void                        Transform(const DirectX::XMMATRIX transform); // modifies the underlying data
        virtual void _vectorcall            Translate(const float3 In); 
        virtual void _vectorcall            Scale(const float3 scaleIn);
        virtual void _vectorcall            Rotate(const float3 orignIn, const quat rIn);
        
        virtual Box                         CalculateBoundingBox() { Box rtn; rtn.SetZero(); return rtn; } // define in derived class
        // Confirmations
        bool                                HasPositions() const { return _vertexFormat.Has(King::VertexAttrib::enumDesc::position); }
        bool                                HasColors() const { return _vertexFormat.Has(King::VertexAttrib::enumDesc::color); }
        bool                                HasTextureCoordinates() const { return _vertexFormat.Has(King::VertexAttrib::enumDesc::textureCoord); }
        bool                                HasNormals() const { return _vertexFormat.Has(King::VertexAttrib::enumDesc::normal); }
        bool                                HasTangents() const { return _vertexFormat.Has(King::VertexAttrib::enumDesc::tangent); }
        bool                                HasBiTangents() const { return _vertexFormat.Has(King::VertexAttrib::enumDesc::bitangent); }
        bool                                HasBoneIndicies() const { return _vertexFormat.Has(King::VertexAttrib::enumDesc::boneIndicies); }
        bool                                HasBoneWeights() const { return _vertexFormat.Has(King::VertexAttrib::enumDesc::boneWeights); }
        // Accessors
        const auto&                         GetModelName() const { return _modelName; }
        const auto&                         GetBoundingBox() const { return _boundingBox; }

        uint8_t*                            GetVertexAddr(const uint32_t vertexNumIn, uint32_t vbStartIn = 0); // no indirection
        uint8_t*                            GetVertexAddr(const uint32_t indexIndirectionIn, uint32_t vbStartIn, uint32_t ibStartIn); // byte pointer to start of vertex data, indirection through index
        uint32_t                            GetVertexCount() const { return (uint32_t)(_vertexBufferMaster.GetLength() / _vertexFormat.GetByteSize()); }
        auto &                              GetVertexFormat() const { return _vertexFormat; }
        auto                                GetVertexStride() const { return _vertexFormat.GetByteSize(); }
        auto &                              GetVertexBufferMaster() { return _vertexBufferMaster; }

        virtual const size_t                GetNumMeshes() { return 0; } // just our master data as one mesh

        auto                                GetIndexCount() const { return _indexBufferMaster.GetElements(); }
        uint32_t                            GetIndexStride() const { return IndexFormat::euint32 ? 4 : 2;} // in bytes
        auto &                              GetIndexBufferMaster() { return _indexBufferMaster; }

        // unique will return vertex in order (eg. 0, 1, 2, 3) and unique = false will return in index buffer order with duplicates (eg. indicies = {0,1,2, 1,3,2} return is 0, 1, 2, 1, 3, 2 and not 0, 1, 2, 3
        vector<vector<float>>               GetPositions(bool unique = true) { return GetDataAttribute(VertexAttrib::enumDesc::position, unique); }
        vector<vector<float>>               GetColors(bool unique = true) { return GetDataAttribute(VertexAttrib::enumDesc::color, unique); }
        vector<vector<float>>               GetTextureCoordinates(bool unique = true) { return GetDataAttribute(VertexAttrib::enumDesc::textureCoord, unique); }
        vector<vector<float>>               GetNormals(bool unique = true) { return GetDataAttribute(VertexAttrib::enumDesc::normal, unique); }
        vector<vector<float>>               GetTangents(bool unique = true) { return GetDataAttribute(VertexAttrib::enumDesc::tangent, unique); }
        vector<vector<float>>               GetBiTangents(bool unique = true) { return GetDataAttribute(VertexAttrib::enumDesc::bitangent, unique); }
        vector<vector<float>>               GetBoneIndicies(bool unique = true) { return GetDataAttribute(VertexAttrib::enumDesc::boneIndicies, unique); }
        vector<vector<float>>               GetBoneWeights(bool unique = true) { return GetDataAttribute(VertexAttrib::enumDesc::boneWeights, unique); }
        // Assignments
        virtual void                        AddMasterVertexData(const MemoryBlock<uint8_t>& vdIn) { _vertexBufferMaster.Append(vdIn); }
        virtual void                        AddMasterIndexData(const MemoryBlock<uint32_t> &idIn) { _indexBufferMaster.Append(idIn); }
        void                                AddVertexAttribute(const VertexAttrib::enumDesc & propertyIn, const VertexAttrib::enumFormat & formatIn);

        void                                SetModelName(const std::string modelNameIN) { _modelName = modelNameIN; }
        void                                SetIndex(const size_t& indexIn, const uint32_t& value);
        void                                SetVertex(const size_t& vertexIndexIn, uint8_t * dataIn);
        void                                SetVertexStream(const size_t& vertexIndexIn, uint8_t* dataIn, size_t numVertex);
        void                                SetIndexStream(const size_t& startIndexIn, uint8_t* dataIn, size_t numIndicies);
        void                                SetIndexStream(const size_t& startIndexIn, uint32_t* dataIn, size_t numIndicies);
        void                                SetVertexElement(const size_t & vertexIndexIn, const VertexAttrib::enumDesc & propertyIn, const uint8_t * dataIn);
        void                                SetVertexFormat(const VertexFormat & vfIn) { _vertexFormat = vfIn; _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize()); }
    protected:
        vector<vector<float>>               GetDataAttribute(VertexAttrib::enumDesc attributeEnum, uint32_t vbStartIn = 0, uint32_t ibStartIn = 0, bool unique = true);
        
        void                                SetDataAttribute(vector<vector<float>>& dataIn, VertexAttrib::enumDesc attributeEnum, uint32_t vbStartIn = 0, uint32_t ibStartIn = 0, bool unique = true);
    };
    /******************************************************************************
    *    LineModel
    *       of Line meshes
    *   Use:    Set the vertex format before calling 
    *           Initalize(size_t verticiesIn, size_t indiciesIn)
    ******************************************************************************/
    class alignas(16) LineModel : public ModelScaffold
    {
       // friend class LineModel_IO; // separate the loading/saving functionality so it can be overridded easily with a new class and operate on LineModel

        /* variables */
    protected:
        std::vector<std::shared_ptr<LineMesh>> _meshes;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<LineModel>    Create() { return std::make_shared<LineModel>(); }
        // Construction/Destruction
        LineModel() = default;
        LineModel(const Box& boxIn) { *this = boxIn; } // forward to copy assignment
        LineModel(const LineModel &in) { *this = in; } // forward to copy assignment
        LineModel(LineModel &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~LineModel() { Destroy(); }

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<LineModel*>(p)); }
        LineModel& operator= (const Line& lineIn) { CreateMeshFrom(lineIn); } // adds a mesh to model
        LineModel& operator= (const Box& boxIn); // adds a mesh to model
        LineModel& operator= (const LineModel &in); // copy assignment
        LineModel& operator= (LineModel &&in) = default; // move assignment
        // Functionality
        virtual void                        Destroy() override { _meshes.clear(); King::ModelScaffold::Destroy(); }

        virtual Box                         CalculateBoundingBox() override;

        void                                MergeMeshes(const size_t meshIndex1, const size_t meshIndex2);
        // Accessors
        virtual const size_t                GetNumMeshes() override { return _meshes.size(); }
        std::shared_ptr<LineMesh>           GetMesh(size_t index) { assert(index < _meshes.size()); return _meshes[index]; }
        // Assignments
        auto                                AddMesh(std::shared_ptr<LineMesh> meshIn) { _meshes.push_back(meshIn); return _meshes.size()-1; }
        std::shared_ptr<LineMesh>           CreateMesh(uint32_t numLinesIn, uint32_t vbStartIn = 0, uint32_t ibStartIn = 0);
        int                                 CreateMeshFrom(const King::Line& lineIn, float4 colorIn = float4(0.f, 1.0f, 1.0f, 1.0f));
    protected:
        void                                HelperSetAllMeshesVBandIBtoMaster() { for (auto& m : _meshes) { m->SetVB(_vertexBufferMaster.GetData()); m->SetIB(_indexBufferMaster.GetData()); } }
    };

    /******************************************************************************
    *    BoneHierarchy
    *       Modifier for geometry to allow skin transforms of a bone hierarchy.
    ******************************************************************************/
    class alignas(16) BoneHierarchy
    {
        /* structures */
        struct Bone
        {
            std::string                     _name; // unique so we can build the skeleton hiearchy after saves and loads
            DirectX::XMFLOAT4X4             _transform; // defines the bone orientation
        };
        struct Skeleton
        {
            std::vector<Bone>               _bones; // sort such that a child bone never comes before a parent bone
            std::vector<DirectX::XMFLOAT4X4> _toParentTransform; // 1 to 1 size with bones, contains the hiearchy of the bone structure

            // Note: toParentTransform[0] is alway identity as the top most node of the structure
        };
        /* variables */
    public:
        // begin file type v1
        Skeleton                            _skeleton;
        std::vector<uint8_t>                _boneInfluenceCount; // 1 to 1 size with # of verticies in mesh and contains the number of bones influencing each vertex

        std::vector<uint8_t>                _boneIndex; // size is boneInfluenceCount values summed and hold the indicies into skeleton.bones and is ordered in vertex order; ex: _bones.size()=2, _boneInfluenceCount[0]=1, _boneInfluenceCount[1]=4, then _boneIndex.size() = 5;
        std::vector<float>                  _boneWeight; // size matches boneIndex and gives the weight of each bones influence on this vertex
        // end file type v1

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<BoneHierarchy>   Create() { return std::make_shared<BoneHierarchy>(); }
        static std::unique_ptr<BoneHierarchy>   CreateUnique() { return std::make_unique<BoneHierarchy>(); }
        BoneHierarchy() = default;
        BoneHierarchy(const BoneHierarchy& in) { *this = in; } // forward to copy assignment
        BoneHierarchy(BoneHierarchy&& in) noexcept { *this = std::move(in); } // forward to move assignment
        virtual ~BoneHierarchy() = default;
        // Operators 
        void* operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void* p) { _aligned_free(static_cast<BoneHierarchy*>(p)); }

        BoneHierarchy& operator= (const BoneHierarchy& in) = default; // copy assign
        BoneHierarchy& operator= (BoneHierarchy&& in) = default; // move assign
        // Conversions
        // Functionality
        // I/O
        virtual bool                        Read_v1(ifstream& dataFile);
        virtual bool                        Write_v1(ofstream& dataFile);
        // Accessors
        auto                                GetBoneCount() const { return _skeleton._bones.size(); }
        // Assignments
    };

    /******************************************************************************
    *    Model
    *       of Triangle meshes
    ******************************************************************************/
    class alignas(16) Model : public ModelScaffold
    {
        friend class Model_IO; // separate the loading/saving functionality so it can be overridden easily with a new class and operate on Model

        /* variables */
    protected:
        // begin file type
        std::vector< std::shared_ptr<TriangleMesh>>      _meshes;
        std::map<std::string, std::shared_ptr<Material>> _materials;
        // end file type v1
        std::unique_ptr<BoneHierarchy>                   _boneHierarchy;
        // end file type v2

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Model>       Create() { return std::make_shared<Model>(); }
        static std::shared_ptr<Model>       Create(const VertexFormat& vfIn) { return std::make_shared<Model>(vfIn); }
        // Construction/Destruction
        // NOTE: if we have a default material for the sky model, it does not render, so commented out below as also not needed 7/15/2023 CHK
        Model() { /*_vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3); _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize()); AddMaterial(Material::Create("default"));*/ }
        Model(const VertexFormat& vfIn) { _vertexFormat = vfIn; _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize()); AddMaterial(Material::Create("default")); }
        
        Model(const Triangle& triIn) { *this = triIn; } // forward to copy assignment
        Model(const Quad& quadIn) { *this = quadIn; } // forward to copy assignment
        Model(const Path& closedPathIn) { *this = closedPathIn; } // forward to copy assignment
        Model(const Box& boxIn) { *this = boxIn; } // forward to copy assignment
        Model(const Sphere& sphereIn) { *this = sphereIn; } // forward to copy assignment
        Model(const Pyramid& pyramidIn) { *this = pyramidIn; } // forward to copy assignment
        Model(const Model &in) { *this = in; } // forward to copy assignment
        Model(Model &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~Model() { Destroy(); }

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Model*>(p)); }

        Model& operator= (const Triangle& triIn) { Destroy(); CreateMeshFrom(triIn); return *this; }
        Model& operator= (const Quad& quadIn) { Destroy(); CreateMeshFrom(quadIn); return *this; }
        Model& operator= (const Path& pathIn) { Destroy(); CreateMeshFrom(pathIn); return *this; }
        Model& operator= (const Box& boxIn) { Destroy(); CreateMeshFrom(boxIn); return *this; }
        Model& operator= (const Sphere& sphereIn) { Destroy(); CreateMeshFrom(sphereIn); return *this; }
        Model& operator= (const Pyramid& pyramidIn) { Destroy(); CreateMeshFrom(pyramidIn); return *this; }
        Model& operator= (const Model &in); // copy assignment
        Model& operator= (Model &&in) = default; // move assignment

        // Functionality
        virtual void                        Destroy() override { _meshes.clear(); _materials.clear(); delete _boneHierarchy.release(); King::ModelScaffold::Destroy(); }
        
        virtual bool                        Load(std::string fileNameIN) override;
        virtual bool                        Save(std::string fileNameIN) override;
        
        virtual bool                        Read_v1(ifstream& dataFile);
        virtual bool                        Write_v1(ofstream& dataFile);

        virtual bool                        Read_v2(ifstream& dataFile);
        virtual bool                        Write_v2(ofstream& dataFile);

        std::vector<float3>                 CalculateVertexSkinPositions(size_t meshIndex = 0);

        virtual Box                         CalculateBoundingBox() override;
        void                                CalculateNormals();
        void                                CalculateTextureCoordinates();
        void                                CalculateTangentsAndBiTangents();

        void                                ReverseNormals();
        void                                ReverseWindings();
        void                                ReverseWindingsToMatchNormals();

        size_t                              RemoveUnusedVerticies();
        size_t                              RemoveDuplicateVerticies();

        void                                OptimizeVertexBuffer() { for (auto & m : _meshes) { OptimizeMeshVertexBuffer(*m.get()); } }
        void                                OptimizeMeshVertexBuffer(const TriangleMesh & forMeshIn); // re-order to match index buffer ordering

        void                                MergeMeshes(const size_t meshIndex1, const size_t meshIndex2);

        void                                LogReport(const std::wstring fileNameIn);
        // Accessors
        virtual const size_t                GetNumMeshes() override { return _meshes.size(); }
        std::shared_ptr<TriangleMesh>       GetMesh(size_t index) { assert(index < _meshes.size()); return _meshes[index]; }
        const std::vector< std::shared_ptr<TriangleMesh>> & GetMeshes() const { return _meshes; }

        std::shared_ptr<Material>           GetMaterial(string name) { assert(_materials.count(name)); return _materials[name]; }
        auto&                               GetMaterials() { return _materials; }
        const auto                          GetNumMaterials() const { return _materials.size(); }
        
        virtual std::size_t                 GetBoneCount() const { if (_boneHierarchy) return _boneHierarchy->GetBoneCount(); else return 0; }
        // Assignments
        void                                AddMaterial(std::shared_ptr<Material> mtl_IN);

        virtual void                        AddMasterVertexData(const MemoryBlock<uint8_t>& vdIn) override { _vertexBufferMaster.Append(vdIn); for (auto& m : _meshes) m->SetVB(_vertexBufferMaster.GetData()); }
        virtual void                        AddMasterIndexData(const MemoryBlock<uint32_t>& idIn) override { _indexBufferMaster.Append(idIn); for (auto& m : _meshes) m->SetIB(_indexBufferMaster.GetData()); }

        void                                AddMesh(const TriangleMesh& meshIn) { auto m = TriangleMesh::Create(); *m = meshIn; _meshes.push_back(m); }
        void                                AddMesh(std::shared_ptr <TriangleMesh> meshIn) { _meshes.push_back(meshIn); }

        std::shared_ptr<TriangleMesh>       CreateMesh(const uint32_t numTrianglesIn, const uint32_t vbStartIn = 0, const uint32_t ibStartIn = 0); // empty mesh
        
        int                                 CreateMeshFrom(const Triangle& tri);
        int                                 CreateMeshFrom(const Quad& quad); 
        int                                 CreateMeshFrom(const Box& box); // presently, takes over all the data for the model, *** TO DO ** add mesh
        int                                 CreateMeshFrom(const Sphere& sphere, const uint32_t segmentsIn = 32); // quads
        int                                 CreateMeshFrom(const Pyramid& pyramidIn);
        int                                 CreateMeshFrom(const Path& closedPath); // closed path in ccw ordering, RHS as a triangle fan of path(first, second, last) then path(last, second, last-1), etc
        int                                 CreateMeshFrom(const Line& line, Distance d); // extrude d away from l as a quad
        int                                 CreateMeshFrom(const Path& path, Distance d); // extrude d away from p as a series of perpendicular quads
        int                                 CreateMeshFromExp(const Path& path, Distance d); // extrude d away from p as a series of perpendicular quads, d assumes "outward"
        int                                 CreateMeshFrom(const Path& pFrom, const Path& pTo); // extrude pFrom to pTo as a series of connected quads
    
        int                                 CreateMeshForGridXZ(const uint32_t numWidth, const uint32_t numHeight, const float2 cellSize);

        void                                SetColor(float3 cIn, int forMesh = 0) { // diffuse color
                                                auto mtl = GetMaterial("default");
                                                auto mtlProp = mtl->Get_properties();
                                                mtlProp.color.diffuse[0] = cIn.GetX();
                                                mtlProp.color.diffuse[1] = cIn.GetY();
                                                mtlProp.color.diffuse[2] = cIn.GetZ();
                                                mtl->Set_properties(mtlProp); 
                                                string n;
                                                n = "color " + std::to_string(mtlProp.color.diffuse[0]) + " " + std::to_string(mtlProp.color.diffuse[1]) + " " + std::to_string(mtlProp.color.diffuse[2]);
                                                mtl->Set_name(n);
                                                AddMaterial(mtl);
                                                assert(forMesh < _meshes.size());
                                                _meshes[forMesh]->Set_materialName(n); }
    protected:
        void                                HelperSetAllMeshesVBandIBtoMaster() { for (auto& m : _meshes) { m->SetVB(_vertexBufferMaster.GetData()); m->SetIB(_indexBufferMaster.GetData()); } }
        int                                 HelperCreateMeshFrom(const vector<float3>& positionsIn, const vector<uint32_t>& indiciesIn, const vector<float2>& uvIn, const vector<float3>& normalsIn);
        std::vector<King::float2>           HelperCreateTextureCoordinatesForTriOrQuad(const std::vector<King::float3>& pt, uint32_t ptsWide);
        void                                HelperCreateTextureCoordinatesForSphere();
    };                                                     

    /******************************************************************************
    *    SkinnedModel
    *      [[deprecated("Replaced by Model, which incorporates into the base class")]]
    ******************************************************************************/
    class alignas(16) SkinnedModel : public Model
    {
        friend class Model_IO;

        /* variables */
    public:
        // begin file type v1
        std::unique_ptr<BoneHierarchy>      _boneHierarchy;
        // end file type v1

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<SkinnedModel>Create() { return std::make_shared<SkinnedModel>(); }
        // Construction/Destruction
        SkinnedModel() = default;
        SkinnedModel(const VertexFormat& vfIn) : Model(vfIn) { ; }
        SkinnedModel(const Path& pathIn) { *this = pathIn; } // forward to copy assignment
        SkinnedModel(const Box& boxIn) { *this = boxIn; } // forward to copy assignment
        SkinnedModel(const Sphere& sphereIn) { *this = sphereIn; } // forward to copy assignment
        SkinnedModel(const Model& copyIn) : Model(copyIn) { ; }
        SkinnedModel(Model&& moveIn) noexcept : Model(std::move(moveIn)) { ; }
        SkinnedModel(const SkinnedModel &in) { *this = in; } // forward to copy assignment
        SkinnedModel(SkinnedModel &&in) noexcept { *this = std::move(in); } // forward to move assignment
        virtual ~SkinnedModel() { Destroy(); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<SkinnedModel*>(p)); }

        inline SkinnedModel& operator= (const Path& pathIn) { Model::operator=(pathIn); return *this; }
        inline SkinnedModel& operator= (const Box& boxIn) { Model::operator=(boxIn); return *this; }
        inline SkinnedModel& operator= (const Sphere& sphereIn) { Model::operator=(sphereIn); return *this; }
        inline SkinnedModel& operator= (const SkinnedModel& in) { Model::operator=(in); if (in._boneHierarchy) { _boneHierarchy = std::make_unique<BoneHierarchy>(); *_boneHierarchy = *in._boneHierarchy; } return *this; } // copy assignment
        inline SkinnedModel& operator= (const Model& in) { Model::operator=(in); return *this; } // copy assignment
        inline SkinnedModel& operator= (Model&& in) noexcept { Model::operator=(std::move(in)); return *this; } // move assignment
        inline SkinnedModel& operator= (SkinnedModel &&in) noexcept { std::swap(_boneHierarchy, in._boneHierarchy); Model::operator=(std::move(in)); return *this; } // move assignment
        // Functionality
        virtual void                        Destroy() override { delete _boneHierarchy.release(); Model::Destroy(); }

        std::vector<float3>                 CalculateVertexSkinPositions(size_t meshIndex = 0);
        // I/O
        virtual bool                        Read_v1(ifstream& dataFile);
        virtual bool                        Write_v1(ofstream& dataFile);
        // Accessors
        virtual std::size_t                 GetBoneCount() const { if (_boneHierarchy) return _boneHierarchy->GetBoneCount(); else return 0; }
        // Assignments
    };

} // King namespace
