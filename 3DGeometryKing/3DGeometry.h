/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          3DGeometryKing

Description:    3D Geometry with accelerated SIMD math

Usage:          3D Software development

References:     
                1) Book Title: Game Programming Gems II
                Chapter Title: Fast, Robust Intersection of 3D Line Segments
                Author: Graham Rhodes
                Revisions: 05-Apr-2001 - GSR. Original.
 
                2) Möller, Tomas; Trumbore, Ben (1997). "Fast, Minimum Storage Ray-Triangle Intersection". Journal of Graphics Tools. 2: 21–28.
                https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm


Contact:        https://chrisking340.github.io/GeometryKing/

Copyright (c) 2020 Christopher H. King

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

#define KING_3DGEOMETRY_VERSION_MAJOR 1
#define KING_3DGEOMETRY_VERSION_MINOR 0
#define KING_3DGEOMETRY_VERSION_PATCH 0

#include <DirectXCollision.h>
#include <vector>
#include <map>
#include <filesystem>
#include <time.h>

#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

#include "..\MathSIMD\MathSIMD.h"
#include "..\General\MemoryBlock.h"
#include "..\Physics\UnitOfMeasure.h"
#include "..\Physics\Position.h"
#include "..\Physics\Distance.h"

#include "CK_Cube.h"
#include "CK_CubeCollision.h"

// Compile with code files 3DGeometry.cpp and CK_CubeCollision.cpp

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
    class Box;  // SIMD two points; AABB and as a OBB by passing in a Quaternion to methods
    class Fustrum; // SIMD six planes and eight points
    // 3D Complex objects, working with vertex buffer references:
    class LineIndexed; // int[2] into a vertex buffer
    class TriangleIndexed; // int[3] into a vertex buffer
    // 3D Meshs, data is not owned and rather referenced from the model that stores it.
    class LineMesh;
    class TriangleMesh;
    // 3D Structure that matches verticies to an underlining connected hierarchy
    class BoneHierarchy; //skelton structure with transforms, index, and weights to influence verticies
    // 3D Models containing meshes.  Models own the master data that is shared by different meshes. 
    class HeightGrid; // initalized using a TriangleMesh, keeps a rectangular grid of float heights in the XZ domain and a world bounding box with min/max heights for collision detection to grid in the Y domain.
    class LineModel; // from lines
    class Model; // from triangles
    class SkinnedModel; // Model with a pointer to BoneHierarchy data (if it is not skinned then pointer is not initalized)
    // Helpers
    class VertexAttrib;
    class VertexFormat;
    //enum IndexFormat;

    // Functions
    class Contact;
    std::vector<Contact>    SAT_OBBonOBB(const Box& A, const King::Quaternion& qA, const Box& B, const King::Quaternion& qB);
    Contact                 SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(const std::vector<float3>& cornersA, const std::vector<float3>& cornersB, const float3& nonSeparatingAxis);

    // Classes
    /******************************************************************************
    *    VertexAttrib
    ******************************************************************************/
    class alignas(16) VertexAttrib
    {
    public:
        enum enumDesc
        {
            position = 0,
            color,
            textureCoord,
            normal,
            tangent,
            bitangent,
            boneWeights,
            boneIndicies,

            types
        };
        static std::vector<std::string>  description; // to describe enumDesc as strings.  Global initialized in 3DGeometry.cpp

        enum enumFormat
        {
            format_none = 0,
            format_byte8,
            format_byte8x2,
            format_byte8x3,
            format_byte8x4,
            format_int16,
            format_int16x2,
            format_int16x3,
            format_int32,
            format_float32,
            format_int32x3,
            format_float32x2,
            format_float32x3,
            format_int32x4,
            format_float32x4,

            formats
        };
        uint16_t    _offset = 0; // bytes
        enumFormat  _format = format_none; // data format
        enumDesc    _desc = types; // data descriptor

    public:
        VertexAttrib() = default;
        VertexAttrib(const VertexAttrib & in) : _offset(in._offset), _format(in._format), _desc(in._desc) {} // copy assign

        const auto &                        GetDescription() const { return _desc; }
        std::string                         GetDescriptionString() const { return description[_desc]; }
        const auto &                        GetFormat() const { return _format; }
        const auto &                        GetOffset() const { return _offset; }

        uint32_t GetByteSize() const
        {
            uint32_t bytes;
            switch (_format)
            {
            case VertexAttrib::enumFormat::format_none:
                bytes = 0;
                break;
            case VertexAttrib::enumFormat::format_byte8:
                bytes = 1;
                break;
            case VertexAttrib::enumFormat::format_byte8x2:
                bytes = 2;
                break;
            case VertexAttrib::enumFormat::format_byte8x3:
                bytes = 3;
                break;
            case VertexAttrib::enumFormat::format_byte8x4:
                bytes = 4;
                break;
            case VertexAttrib::enumFormat::format_int16:
                bytes = 2;
                break;
            case VertexAttrib::enumFormat::format_float32:
            case VertexAttrib::enumFormat::format_int16x2:
                bytes = 2 * 2;
                break;
            case VertexAttrib::enumFormat::format_int16x3:
                bytes = 2 * 3;
                break;
            case VertexAttrib::enumFormat::format_float32x2:
                bytes = 4 * 2;
                break;
            case VertexAttrib::enumFormat::format_int32x3:
            case VertexAttrib::enumFormat::format_float32x3:
                bytes = 4 * 3;
                break;
            case VertexAttrib::enumFormat::format_int32x4:
            case VertexAttrib::enumFormat::format_float32x4:
                bytes = 4 * 4;
                break;
            default:
                bytes = 8; // any other would be 64 bits
            }
            return bytes;
        }
    };
    /******************************************************************************
    *    VertexFormat
    ******************************************************************************/
    class alignas(16) VertexFormat
    {
    protected:
        VertexAttrib    attributes[8];
    private:
        uint16_t        nextAttribute = 0;
    public:
        VertexFormat() = default;
        VertexFormat(const VertexFormat & otherIn) { for (uint16_t i = 0; i < 8; ++i) attributes[i] = otherIn.attributes[i]; nextAttribute = otherIn.nextAttribute; } // copy assign

        bool Has(VertexAttrib::enumDesc descIn) const { for (uint16_t i = 0; i < nextAttribute; ++i) if (attributes[i]._desc == descIn) return true; return false; }

        uint16_t SetNext(VertexAttrib::enumDesc descIn, VertexAttrib::enumFormat formatIn)
        {
            assert(nextAttribute < 8);
            attributes[nextAttribute]._offset += GetByteSize();
            attributes[nextAttribute]._desc = descIn;
            attributes[nextAttribute]._format = formatIn;
            ++nextAttribute;

            return nextAttribute - 1;
        }
        uint32_t GetByteSize() const
        {
            uint32_t s = 0;
            int i = 0;

            while (i < nextAttribute) 
            { 
                s += attributes[i].GetByteSize(); 
                ++i; 
            }

            return s;
        }
        bool                    Has(VertexAttrib::enumDesc descIn);
        auto                    GetNumAttributes() const { return nextAttribute; }
        auto                    GetAttribute(uint16_t indexIn) const { return attributes[indexIn]; }
        uint16_t                GetAttributeIndexFromDescription(VertexAttrib::enumDesc descIn) const;
        auto                    GetAttributeDescriptionFromIndex(uint16_t indexIn) const { return attributes[indexIn]._desc; }
    };
    /******************************************************************************
    *    IndexFormat
    ******************************************************************************/
    enum IndexFormat
    {
        uint32 = 1
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
        float3                              _translation; // defaults to zero components
        /* methods */
    public:
        // Creation/Life cycle
        Pose() = default;
        Pose(const Pose &M) { *this = M; } // involke copy assignment operator
        Pose(Pose &&M) noexcept { *this = std::move(M); } // involke move assignment operator
        explicit Pose(const DirectX::XMMATRIX &M4x4); // involke conversion copy assignment operator
        explicit Pose(const DirectX::XMFLOAT4X4 &F4x4); // involke conversion copy assignment operator
        explicit Pose(const quat &rotation, const float3 &scale, const float3 &translation) { Set(rotation, scale, translation); }
        explicit Pose(quat rotate) : _rotation(rotate) {}
        explicit Pose(float3 translate) : _translation(translate) {}
        explicit Pose(quat rotate, float3 translate) : _rotation(rotate), _translation(translate) {}
        explicit Pose(const XMMATRIX &M3x3, float3 translate);
        ~Pose() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Pose*>(p)); }
        inline Pose & operator= (const Pose &in) // copy assign
        {
            _rotation = in._rotation;
            _scale = in._scale;
            _translation = in._translation;
            return *this;
        }
        inline Pose & operator= (Pose &&in) // move assign
        {
            if (this != &in)
            {
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
            rtn.SetScale(King::Lerp(_scale,in._scale, float3(0.5f))); // average
            rtn.SetRotation(in._rotation + _rotation); // non-communtative, order matters
            rtn.SetTranslation(_translation + in._translation);
            return rtn;
        }
        inline Pose operator- (const Pose &in) const
        {
            Pose rtn;
            rtn.SetScale(King::Lerp(_scale, in._scale, float3(0.5f))); // average
            rtn.SetRotation(in._rotation - _rotation); // non-communtative, order matters
            rtn.SetTranslation(_translation - in._translation);
            return rtn;
        }
        inline Pose operator* (const Pose &in) const
        {
            Pose rtn;
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
            rtn.SetRotation(_rotation*in);
            rtn.SetTranslation(_translation*in);
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
        Pose                                Interpolate(const Pose &in, const float fractionIn) const;
        // Accessors
        inline auto&                        GetRotation() { return _rotation; }
        inline auto&                        GetScale() { return _scale; }
        inline auto&                        GetTranslation() { return _translation; }
        // constant
        inline auto&                        GetRotation() const { return _rotation; }
        inline auto&                        GetScale() const { return _scale; }
        inline auto&                        GetTranslation() const { return _translation; }

        inline DirectX::XMMATRIX            Get_XMMATRIX() const { auto rtn = DirectX::XMMatrixAffineTransformation(_scale, float3(), _rotation, _translation); assert(!XMMatrixIsNaN(rtn)); return rtn; }
        inline DirectX::XMFLOAT4X4          Get_XMFLOAT4X4() const { DirectX::XMFLOAT4X4 rtn; DirectX::XMStoreFloat4x4(&rtn, DirectX::XMMatrixAffineTransformation(_scale, float3(), _rotation, _translation)); return rtn; }
        // Assignments
        inline void _vectorcall             Set(const quat &rotation, const float3 &scale, const float3 &translation) { _rotation = rotation; _scale = scale; _translation = translation; }
        inline void _vectorcall             SetRotation(const quat &rotation) { _rotation = rotation; _rotation.Validate(); }
        inline void _vectorcall             SetRotationAboutXYZAxis(const float &radiansX, const float &radiansY, const float &radiansZ) { _rotation = DirectX::XMQuaternionRotationRollPitchYaw(radiansX, radiansY, radiansZ); }
        inline void _vectorcall             SetRotationAboutXAxis(const float &radians) { _rotation = DirectX::XMQuaternionRotationRollPitchYaw(radians, 0.0f, 0.0f); }
        inline void _vectorcall             SetRotationAboutYAxis(const float &radians) { _rotation = DirectX::XMQuaternionRotationRollPitchYaw(0.0f, radians, 0.0f); }
        inline void _vectorcall             SetRotationAboutZAxis(const float &radians) { _rotation = DirectX::XMQuaternionRotationRollPitchYaw(0.0f, 0.0f, radians); }
        inline void _vectorcall             SetScale(const float3 &scale) { _scale = scale; }
        inline void _vectorcall             SetTranslation(const float3 &translation) { _translation = translation; }
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
    public:
        virtual bool Collision(Collidable const& in) const = 0;

        virtual bool Collision(Point const& pointIn) const = 0;
        virtual bool Collision(Ray const& pointIn) const = 0;
        virtual bool Collision(Line const& lineIn) const = 0;
        virtual bool Collision(Sphere const& sphereIn) const = 0;
        virtual bool Collision(Box const& boxIn) const = 0;
        // TO DO 
        //virtual bool Collision(Plane const& planeIn) const = 0;
        //virtual bool Collision(Frustum const& frustumIn) const = 0;
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
        Point(const FloatPoint3 &locationIn) : Position(locationIn) { ; }
        explicit inline Point(const Distance &distanceIn) : Position(distanceIn) { ; }
        explicit inline Point(const DirectX::XMVECTOR &vIn) : Position(vIn) { ; }
        explicit inline Point(const DirectX::XMVECTORF32 &vIn) : Position(vIn) { ; }
        explicit inline Point(const FloatPoint4 &ptIn) : Position(ptIn) { ; }
        explicit inline Point(const Point &in) { *this = in; } // forward to copy assignment
        explicit inline Point(Point &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~Point() = default;

        // Conversions
        inline explicit operator float() const { return King::FloatPoint3::Magnitude(_position); }
        inline explicit operator UnitOfMeasure::Length() const { return UnitOfMeasure::Length(King::FloatPoint3::Magnitude(_position)); }
        inline explicit operator DirectX::XMVECTOR() const { return _position.GetVecConst(); }
        inline operator FloatPoint3() const { return _position; } // allow implicit for a default behavior
        inline explicit operator FloatPoint4() const { return FloatPoint4(_position, 1.0f); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Point*>(p)); }
        inline Point & operator= (const XMVECTOR &in) { Position::operator=(in); return *this; } // copy assignment
        //inline Point & operator= (const XMVECTORF32 &in) { Position::operator=(in); return *this; } // copy assignment
        inline Point & operator= (const FloatPoint3 &in) { Position::operator=(in); return *this; } // copy assignment
        inline Point & operator= (const FloatPoint4 &in) { Position::operator=(in); return *this; } // copy assignment
        inline Point & operator= (const Point &in) { Position::operator=(in); return *this; } // copy assignment
        inline Point & operator= (Point &&in) { Position::operator=(std::move(in)); return *this; } // move assignment

        inline Point operator- (const Point &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) - static_cast<FloatPoint3>(rhs)); return ptOut; }
        inline Point operator+ (const Point &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) + static_cast<FloatPoint3>(rhs)); return ptOut; }
        inline Point operator* (const Point &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) * static_cast<FloatPoint3>(rhs)); return ptOut; }
        inline Point operator/ (const Point &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) / static_cast<FloatPoint3>(rhs)); return ptOut; }
        inline Point & operator-= (const Point &rhs) { *this = *this - rhs; return *this; }
        inline Point & operator+= (const Point &rhs) { *this = *this + rhs; return *this; }
        inline Point & operator*= (const Point &rhs) { *this = *this * rhs; return *this; }
        inline Point & operator/= (const Point &rhs) { *this = *this / rhs; return *this; }

        inline Point operator- (const Distance &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) - static_cast<FloatPoint3>(rhs)); return ptOut; }
        inline Point operator+ (const Distance &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) + static_cast<FloatPoint3>(rhs)); return ptOut; }
        inline Point operator* (const Distance &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) * static_cast<FloatPoint3>(rhs)); return ptOut; }
        inline Point operator/ (const Distance &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) / static_cast<FloatPoint3>(rhs)); return ptOut; }
        inline Point & operator-= (const Distance &rhs) { *this = *this - rhs; return *this; }
        inline Point & operator+= (const Distance &rhs) { *this = *this + rhs; return *this; }
        inline Point & operator*= (const Distance &rhs) { *this = *this * rhs; return *this; }
        inline Point & operator/= (const Distance &rhs) { *this = *this / rhs; return *this; }

        inline Point operator- (const FloatPoint3 &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) - (rhs)); return ptOut; }
        inline Point operator+ (const FloatPoint3 &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) + (rhs)); return ptOut; }
        inline Point operator* (const FloatPoint3 &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) * (rhs)); return ptOut; }
        inline Point operator/ (const FloatPoint3 &rhs) const { Point ptOut(static_cast<FloatPoint3>(*this) / (rhs)); return ptOut; }
        inline Point & operator-= (const FloatPoint3 &rhs) { *this = *this - rhs; return *this; }
        inline Point & operator+= (const FloatPoint3 &rhs) { *this = *this + rhs; return *this; }
        inline Point & operator*= (const FloatPoint3 &rhs) { *this = *this * rhs; return *this; }
        inline Point & operator/= (const FloatPoint3 &rhs) { *this = *this / rhs; return *this; }

        inline Point & operator*= (const DirectX::XMMATRIX &m) { *this = float3(DirectX::XMVector4Transform(static_cast<FloatPoint4>(*this), m)); return *this; }
        friend Point operator* (Point lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)

        // Functionality
        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch

        inline bool    __vectorcall         Intersects(const FloatPoint3 &pointIn) const;
        inline bool                         Intersects(const Point &pointIn) const;
    
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& pointIn) const override { return false; }
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        // *** TO DO  ***
        //virtual bool                        Collision(Plane const& planeIn) const override;
        //virtual bool                        Collision(Frustum const& frustumIn) const override;
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
        FloatPoint3 pt[2];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Line>    Create() { return std::make_shared<Line>(); }
        static std::unique_ptr<Line>    CreateUnique() { return std::make_unique<Line>(); }
        // Construction/Destruction
        Line() = default;
        Line(const Line &in) { *this = in; } // forward to copy assignment
        Line(Line &&in) { *this = std::move(in); } // forward to move assignment
        Line(const FloatPoint3 &pt1In, const FloatPoint3 &pt2In) { pt[0] = pt1In; pt[1] = pt2In; }

        virtual ~Line() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Line*>(p)); }
        inline Line & operator= (const Line &in) = default; // copy assignment
        inline Line & operator= (Line &&in) = default; // move assignment
        inline Line & operator*= (const DirectX::XMMATRIX &m) { pt[0] = DirectX::XMVector4Transform(FloatPoint4(pt[0], 1.0f), m); pt[1] = DirectX::XMVector4Transform(FloatPoint4(pt[1], 1.0f), m); return *this; }
        friend Line operator* (Line lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)

        // Functionality
        inline bool __vectorcall            Intersects(const Point &pointIn) const;
        inline bool __vectorcall            Intersects(const Line &lineIn, FloatPoint3 *intersectionOut = nullptr) const;
        FloatPoint3 __vectorcall            FindNearestPointOnLineSegment(const FloatPoint3 &pointIn) const;
        // Accessors
        const auto &                        GetVertex(const uint32_t vertexIndexIn) const { return pt[vertexIndexIn]; }
        FloatPoint3                         GetLength() { return FloatPoint3::Absolute(pt[1] - pt[0]); }
        FloatPoint3                         GetMidPoint() { return (pt[0] + pt[1]) * 0.5f; }
        // Assignments
        inline void __vectorcall            Set(const Line &in) { pt[0] = in.pt[0]; pt[1] = in.pt[1]; }
        inline void __vectorcall            SetVertex(const uint32_t vertexIndexIn, const FloatPoint3 &in) { pt[vertexIndexIn] = in; }

        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& pointIn) const override { return false; } // *** TO DO  ***
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        // *** TO DO  ***
//virtual bool                        Collision(Plane const& planeIn) const override;
//virtual bool                        Collision(Frustum const& frustumIn) const override;
    };
    /******************************************************************************
    *    LineIndexed - Vertex indicies representing a connected line segment
    *        references an external buffer as an array of byte sequences as described
    *        by vertexFormat.
    *    pt[0] ------- pt[1]
    ******************************************************************************/
    class alignas(16) LineIndexed
    {
        /* variables */
    public:
        uint32_t pt[2]; // index points to vertex buffer
    private:
        uint8_t *vb = nullptr; // vertex buffer
        VertexFormat vertexFormat;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<LineIndexed>    Create() { return std::make_shared<LineIndexed>(); }
        static std::unique_ptr<LineIndexed>    CreateUnique() { return std::make_unique<LineIndexed>(); }
        // Construction/Destruction
        LineIndexed() = default;
        LineIndexed(const LineIndexed &in) { *this = in; } // forward to copy assignment
        LineIndexed(LineIndexed &&in) { *this = std::move(in); } // forward to move assignment
        LineIndexed(const uint8_t *vbIn, const VertexFormat &vertexFormatIn, const uint32_t &pt1In, const uint32_t &pt2In) { pt[0] = pt1In; pt[1] = pt2In; }

        virtual ~LineIndexed() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<LineIndexed*>(p)); }
        inline LineIndexed & operator= (const LineIndexed &in) = default; // copy assignment
        inline LineIndexed & operator= (LineIndexed &&in) = default; // move assignment

        // Functionality
        inline bool    __vectorcall         Intersects(Line &lineIn, FloatPoint3 *intersectionOut = nullptr) { assert(vb != nullptr); Line(float3(GetVertex(0)),float3(GetVertex(1))).Intersects(lineIn, intersectionOut); }
        FloatPoint3 __vectorcall            FindNearestPointOnLineSegment(const FloatPoint3 &pointIn) { assert(vb != nullptr); return Line(float3(GetVertex(0)), float3(GetVertex(1))).FindNearestPointOnLineSegment(pointIn); }
        // Accessors
        uint8_t *                           GetVertex(const uint32_t indexIn) const { return vb + pt[indexIn] * vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        FloatPoint3                         GetVertexPosition(const uint32_t indexIn) const;
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
    *   Path - A 3D path made up of points
    *   Can transform all the points at once with a matrix
    *   front ------- pt2------- back
    ******************************************************************************/
    class alignas(16) Path : public std::list<Point>
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
        Path(const Path &in) { *this = in; } // forward to copy assignment
        Path(Path &&in) { *this = std::move(in); } // forward to move assignment

        virtual ~Path() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Path*>(p)); }
        inline Path & operator= (const Path &in) = default; // copy assignment
        inline Path & operator= (Path &&in) = default; // move assignment
        inline Path & operator*= (const DirectX::XMMATRIX &m) { for(Point & pt : *this) pt.Set( DirectX::XMVector4Transform(pt.GetVecConst(), m) ); return *this; }
        friend Path operator* (Path lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)

        // Functionality
        // Accessors
        // Assignments
   };
    /******************************************************************************
    *    Ray
    *    origin *---> direction (normalized)
    ******************************************************************************/
    class alignas(16) Ray : public Collidable
    {
        /* variables */
    public:
        FloatPoint3 origin;
        FloatPoint3 direction; // normalized on input

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Ray>    Create() { return std::make_shared<Ray>(); }
        static std::unique_ptr<Ray>    CreateUnique() { return std::make_unique<Ray>(); }
        // Construction/Destruction
        Ray() = default;
        Ray(const Ray &in) { *this = in; } // forward to copy assignment
        Ray(Ray &&in) { *this = std::move(in); } // forward to move assignment
        Ray(const FloatPoint3 &originIn, const FloatPoint3 &directionIn) { origin = originIn; direction = directionIn; direction.MakeNormalize(); }
        virtual ~Ray() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Ray*>(p)); }
        inline Ray & operator= (const Ray &in) = default; // copy assignment
        inline Ray & operator= (Ray &&in) = default; // move assignment
        inline Ray & operator*= (const DirectX::XMMATRIX &m) { origin = float3(DirectX::XMVector4Transform(static_cast<DirectX::XMVECTOR>(origin), m)); return *this; }
        friend Ray operator* (Ray lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Functionality
        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override { return false; } // *** TO DO  ***
        virtual bool                        Collision(Ray const& pointIn) const override { return false; } // *** TO DO  ***
        virtual bool                        Collision(Line const& lineIn) const override { return false; } // *** TO DO  ***
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        // *** TO DO  ***
        //virtual bool                        Collision(Plane const& planeIn) const override;
        //virtual bool                        Collision(Frustum const& frustumIn) const override;
        // Accessors
        FloatPoint3 const &                 GetDirection() const { return direction; }
        FloatPoint3 const &                 GetOrigin() const { return origin; }
        // Assignments
        void                                SetDirection(const FloatPoint3 & directionIn) { direction = directionIn; direction.MakeNormalize(); }
        void                                SetOrigin(const FloatPoint3& originIn) { origin = originIn; }
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
    class alignas(16) Plane : public FloatPoint4
    {
    public:
        // Construction/Destruction
        inline explicit Plane() { v = DirectX::XMVectorZero(); }
        inline explicit Plane(const FloatPoint3 normalToPlane, const float distFromOrigin) { v = FloatPoint4(normalToPlane, distFromOrigin); } // should distFromOrigin be negated?
        inline explicit Plane(const FloatPoint3 normalToPlane, const Point origin) { v = DirectX::XMPlaneFromPointNormal(static_cast<DirectX::XMVECTOR>(origin), normalToPlane); }
        inline explicit Plane(const Point &point1, const Point &point2, const Point &point3) { v = DirectX::XMPlaneFromPoints(static_cast<DirectX::XMVECTOR>(point1), static_cast<DirectX::XMVECTOR>(point2), static_cast<DirectX::XMVECTOR>(point3)); } // input points should be CCW
        Plane(float x, float y, float z, float s) { v = FloatPoint4(x,y,z,s); } // should distFromOrigin be negated?

        inline explicit Plane(const DirectX::XMVECTOR &vec) { v = vec; }
        inline explicit Plane(const FloatPoint4 in) { v = in; }
        inline Plane(const Plane &in) { v = in; } // copy 
        inline Plane(Plane &&in) { v = std::move(in); } // move
        virtual ~Plane() = default;
        // Operators 
        inline Plane & operator= (const Plane &in) = default; // copy assignment
        inline Plane & operator= (Plane &&in) = default; // move assignment
        inline Plane & operator= (const FXMVECTOR &in) { v = in; return *this; }
        inline Plane & operator*= (const DirectX::XMMATRIX &m) { v = DirectX::XMPlaneTransform(v, m); return *this; }
        inline Plane operator* (const DirectX::XMMATRIX &m) { return Plane(DirectX::XMPlaneTransform(v, m)); }
        friend Plane operator* (Plane lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Comparators
        inline bool operator==  (const Plane &rhs) { return DirectX::XMVector4Equal(v, rhs.v); }
        inline bool operator!=  (const Plane &rhs) { return DirectX::XMVector4NotEqual(v, rhs.v); }
        // Functionality
        inline bool __vectorcall            Intersects(const Point &pointIn) const;
        inline bool __vectorcall            Intersects(const Sphere &sphereIn) const;
        inline bool __vectorcall            Intersects(const Triangle& triangleIn) const;
        inline bool __vectorcall            Intersects(const Box& boxIn) const;
        FloatPoint3 __vectorcall            FindNearestPointOnPlane(const FloatPoint3 & pointIn) const;
        // Accessors
        FloatPoint3                         GetNormal(void) const { return DirectX::XMPlaneNormalize(v); }
        FloatPoint3                         GetOrigin(void) const { return -GetNormal() * GetW(); } // w keeps the scalar to translate unit vector to origin
        float                               DistanceFromPoint(FloatPoint3 point) const { return Dot(point, GetNormal()) + GetW(); } // projects the point onto the plane and adds the perpendicular distance of the origin
        float                               DistanceFromPoint(Point point) const { return Dot(static_cast<FloatPoint4>(point), v); } // homogeneous (ax,ay,az,a) if a=1
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
        inline TriangleIndexed & operator= (const TriangleIndexed &in) = default; // copy assignment
        inline TriangleIndexed& operator= (TriangleIndexed&& in) noexcept { *this = in; return *this; } // copy assign instead as we do not own the buffer
        // Functionality
        bool __vectorcall                   Intersects(const Ray &rayIn, FloatPoint3 *intersectPointOut) { return Intersects(rayIn.origin,rayIn.direction, intersectPointOut); }
        bool __vectorcall                   Intersects(const FloatPoint3 &rayOriginIn, const FloatPoint3 &rayDirectionIn, FloatPoint3 *intersectPointOut);
        // Accessors
        FloatPoint3                         GetFaceNormal() { assert(vb != nullptr); return FloatPoint3::CrossProduct(GetEdge1(), GetEdge2()); }
        FloatPoint3                         GetEdge1() { assert(vb != nullptr); return GetVertexPosition(1) - GetVertexPosition(0); }
        FloatPoint3                         GetEdge2() { assert(vb != nullptr); return GetVertexPosition(2) - GetVertexPosition(0); }
        FloatPoint3                         GetEdge3() { assert(vb != nullptr); return GetVertexPosition(2) - GetVertexPosition(1); }
        
        uint8_t *                           GetVertex(const uint32_t indexIn) const { return vb + pt[indexIn] * vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        FloatPoint3                         GetVertexPosition(const uint32_t indexIn) const;
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
        FloatPoint3    pt[3];

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
        explicit Triangle(const FloatPoint3 &pt1In, const FloatPoint3 &pt2In, const FloatPoint3 &pt3In) { pt[0] = pt1In; pt[1] = pt2In; pt[2] = pt3In; }
        // Construction Initializer
        Triangle(std::initializer_list<FloatPoint3> il) { assert(il.size() < 4); size_t count = 0; for (auto & each : il) { pt[count] = each; ++count; } }

        virtual ~Triangle() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Triangle*>(p)); }
        inline Triangle & operator= (const Triangle &in) = default; // copy assignment
        inline Triangle & operator= (Triangle &&in) = default; // move assignment
        // Functionality
        bool __vectorcall                   Intersects(const Ray &rayIn, FloatPoint3 *intersectPointOut) { Intersects(rayIn.origin, rayIn.direction, intersectPointOut); }
        bool __vectorcall                   Intersects(const FloatPoint3 &rayOriginIn, const FloatPoint3 &rayDirectionIn, FloatPoint3 *intersectPointOut);
        // Accessors
        FloatPoint3                         GetFaceNormal() { return FloatPoint3::CrossProduct(GetEdge1(), GetEdge2()); }
        FloatPoint3                         GetEdge1() const { return pt[1] - pt[0]; }
        FloatPoint3                         GetEdge2() const { return pt[2] - pt[0]; }
        FloatPoint3                         GetEdge3() const { return pt[2] - pt[1]; }

        auto &                              GetVertex(const int vertNumber_0_1_2) const { return pt[vertNumber_0_1_2]; }
        // Assignments
    };
    /******************************************************************************
    *    Quad
    *            edge4                   Alternate Triangles (Alt1, Alt2)
    *       pt0 -------- pt3              --------
    *           | T1 / |                  | \ T2 |
    *     edge1 |  /   | edge3            |   \  |
    *           |/  T2 |                  | T1  \|
    *       pt1 -------- pt2              --------
    *             edge2
    ******************************************************************************/
    class alignas(16) Quad
    {
        /* variables */
    public:
        FloatPoint3    pt[4];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Quad>    Create() { return std::make_shared<Quad>(); }
        static std::unique_ptr<Quad>    CreateUnique() { return std::make_unique<Quad>(); }
        // Construction/Destruction
        Quad() = default;
        Quad(const Quad &in) { *this = in; } // forward to copy assignment
        Quad(Quad &&in) { *this = std::move(in); } // forward to move assignment
        Quad(const FloatPoint3 &pt0In, const FloatPoint3 &pt1In, const FloatPoint3 &pt2In, const FloatPoint3 &pt3In) { pt[0] = pt0In; pt[1] = pt1In; pt[2] = pt2In; pt[3] = pt3In; }
        Quad(const DirectX::XMFLOAT3 &pt0In, const DirectX::XMFLOAT3 &pt1In, const DirectX::XMFLOAT3 &pt2In, const DirectX::XMFLOAT3 &pt3In) { pt[0] = pt0In; pt[1] = pt1In; pt[2] = pt2In; pt[3] = pt3In; }
        Quad(const FloatPoint3 *pArrayIn) { assert(pArrayIn); pt[0] = pArrayIn[0]; pt[1] = pArrayIn[1]; pt[2] = pArrayIn[2]; pt[3] = pArrayIn[3]; }
        Quad(const DirectX::XMFLOAT3 *pArrayIn) { assert(pArrayIn); pt[0] = pArrayIn[0]; pt[1] = pArrayIn[1]; pt[2] = pArrayIn[2]; pt[3] = pArrayIn[3]; }
        Quad(std::initializer_list<FloatPoint3> il) { assert(il.size() <= 4); size_t count = 0; for (auto & each : il) { pt[count] = each; ++count; } }

        virtual ~Quad() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Quad*>(p)); }
        inline Quad & operator= (const Quad &in) = default; // copy assignment
        inline Quad & operator= (Quad &&in) = default; // move assignment
        // Functionality
        bool                                Intersects(const Ray& ray, FloatPoint3* intersectPointOut = nullptr);
        bool                                Intersects(const FloatPoint3 &rayOriginIn, const FloatPoint3 &rayDirectionIn, FloatPoint3 *intersectPointOut);
        void                                MoveBy(const FloatPoint3 deltaIn) { for(int i=0;i<4;++i) pt[i]+=deltaIn; }
        std::vector<Quad>                   SubDivide();
        void                                Scale(const float scaleIn) { for (int i = 0; i<4; ++i) pt[i] *= scaleIn; }
        void                                Scale(const FloatPoint3 scaleIn) { for (int i = 0; i<4; ++i) pt[i] *= scaleIn; }
        // Accessors
        FloatPoint3                         GetCenter() const { FloatPoint3 r = GetSeam(); r *= 0.5f; return r + pt[1]; }
        FloatPoint3                         GetEdge1() const { return pt[1] - pt[0]; }
        FloatPoint3                         GetEdge2() const { return pt[2] - pt[1]; }
        FloatPoint3                         GetEdge3() const { return pt[3] - pt[2]; }
        FloatPoint3                         GetEdge4() const { return pt[0] - pt[3]; }
        FloatPoint3                         GetSeam() const { return pt[3] - pt[1]; }
        auto                                GetTriangle1() const { return Triangle(pt[0], pt[1], pt[3]); }
        auto                                GetTriangle2() const { return Triangle(pt[1], pt[2], pt[3]); }
        auto                                GetTriangleAlt1() const { return Triangle(pt[0], pt[1], pt[2]); }
        auto                                GetTriangleAlt2() const { return Triangle(pt[2], pt[3], pt[0]); }
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
        inline Sphere() { v = DirectX::XMVectorZero(); }
        inline Sphere(const FloatPoint3 center, const float radius) { FloatPoint4::Set(center, radius); }
        Sphere(const float x, const float y, const float z, const float radius) { FloatPoint4::Set(x, y, z, radius); }

        inline explicit Sphere(const DirectX::XMVECTOR &vec) { v = vec; }
        inline explicit Sphere(const FloatPoint4 in) { v = in; }
        inline Sphere(const Sphere &in) { v = in; } // copy 
        inline Sphere(Sphere &&in) noexcept { v = std::move(in); } // move
        virtual ~Sphere() = default;
        // Operators 
        inline Sphere & operator= (const Sphere &in) = default; // copy assignment
        inline Sphere & operator= (Sphere &&in) = default; // move assignment
        inline Sphere & operator= (const FXMVECTOR &in) { v = in; return *this; }
        inline Sphere & operator*= (const DirectX::XMMATRIX &m) { auto r = GetW(); v = DirectX::XMVector4Transform(FloatPoint4(v, 1.0f), m); SetW(r); return *this; } // does not support scaling since non-uniformed
        friend Sphere operator* (Sphere lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Comparators
        inline bool operator==  (const Sphere &rhs) { return DirectX::XMVector4Equal(v, rhs.v); }
        inline bool operator!=  (const Sphere &rhs) { return DirectX::XMVector4NotEqual(v, rhs.v); }
        // Functionality
        bool                                Contains(const Point & ptIn) const;
        bool                                Intersects(const Ray & rhs, float & distOut) const;
        bool                                Intersects(const Line &rhs) const;
        bool                                Intersects(const Sphere &rhs) const;
                
        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        // *** TO DO  ***
        //virtual bool                        Collision(Plane const& planeIn) const override;
        //virtual bool                        Collision(Frustum const& frustumIn) const override;
        // Accessors
        FloatPoint3                         GetCenter() const { return FloatPoint3(v); }
        float                               GetRadius() const { return GetW(); }
        // Assignments
        void __vectorcall                   SetCenter(const FloatPoint3 centerIn) { float r = GetRadius(); v = FloatPoint4(centerIn, r); }
        void                                SetCetner(const float x, const float y, const float z) { float r = GetRadius(); v = DirectX::XMVectorSet(x, y, z, r); }
        void                                SetRadius(const float radiusIn) { SetW(radiusIn); }
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
        FloatPoint3        pt_min = { -0.5f, -0.5f, -0.5f }; // left, back, bottom (min in RHS)
        FloatPoint3        pt_max = { +0.5f, +0.5f, +0.5f }; // right, top, front (max in RHS)
        
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
        Box(const float3& pt) : pt_min(pt), pt_max(pt) { ; }
        Box(const float3& p1, const float3& p2) : pt_min(Min(p1, p2)), pt_max(Max(p1, p2)) { ; }
        Box(const Box& b1, const Box& b2) : pt_min(Min(b1.pt_min, b2.pt_min)), pt_max(Max(b1.pt_max, b2.pt_max)) { ; }
        Box(const Box &in) { *this = in; } // forward to copy assignment
        Box(Box &&in) noexcept { *this = std::move(in); } // forward to move assignment
        Box(const DirectX::BoundingBox &in) { Set(in); }

        virtual ~Box() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Box*>(p)); }
        inline Box & operator= (const Box &in) { Set(in.GetMin(), in.GetMax()); return *this; } // copy assignment
        inline Box & operator= (Box &&in) = default; // move assignment
        inline Box & operator*= (const DirectX::XMMATRIX &m) { pt_min = DirectX::XMVector4Transform(FloatPoint4(pt_min, 1.0f), m); pt_max = DirectX::XMVector4Transform(FloatPoint4(pt_max, 1.0f), m); return *this; } // does not support scaling since non-uniformed
        friend Box operator* (Box lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        inline Box& operator+= (const float3& d) { MoveBy(d); return *this; }
        inline Box& operator-= (const float3& d) { MoveBy(-d); return *this; }
        inline Box& operator*= (const float3& s) { Grow(s); return *this; }
        // Functionality
        bool                                Intersects(const Ray &rayIn, float &distOut) const;
        bool                                Intersects(const Line &lineIn) const;
        bool                                Intersects(const Box &boxIn) const;
        bool                                Intersects(const Sphere &sphereIn) const;
        bool                                Contains(const Box &boxIn) const; // boxIn is wholely inside of this box
        bool __vectorcall                   Contains(const FloatPoint3 &ptIn) const;

        FloatPoint3 __vectorcall            FindNearestPointOnBox(const FloatPoint3 &pt3In) const;
        FloatPoint3 __vectorcall            FindNearestPointOnBox(const FloatPoint3 &pt3In, const DirectX::FXMMATRIX &M) const;

        virtual bool                        Collision(Collidable const& in) const override { return in.Collision(*this); } // double dispatch
        virtual bool                        Collision(Point const& pointIn) const override;
        virtual bool                        Collision(Ray const& rayIn) const override;
        virtual bool                        Collision(Line const& lineIn) const override;
        virtual bool                        Collision(Sphere const& sphereIn) const override;
        virtual bool                        Collision(Box const& boxIn) const override;
        // *** TO DO  ***
        //virtual bool                        Collision(Plane const& planeIn) const override;
        //virtual bool                        Collision(Frustum const& frustumIn) const override;

        Box                                 CollisionVolume(const Box& boxIn); // over lapping volume with boxIn

        DirectX::XMMATRIX                   MomentsOfInertia(const float& densityIn = 700.0f);
        DirectX::XMMATRIX                   MomentsOfInertiaRotated(const DirectX::XMMATRIX& Iin, const DirectX::XMMATRIX& rotationIn);

        inline void                         MoveTo(const FloatPoint3 &pt2In) { FloatPoint3 offset(pt_max - pt_min); pt_min = pt2In; pt_max = pt_min + offset; }
        inline bool                         MoveTo(const float &xIn, const float &yIn, const float &zIn) { MoveTo(FloatPoint3(xIn, yIn, zIn)); }
        inline void                         MoveBy(const FloatPoint3 &deltaIn) { pt_min += deltaIn; pt_max += deltaIn; }
        inline void                         MoveBy(const Distance& deltaIn) { MoveBy(FloatPoint3(deltaIn)); }
        inline void                         MoveBy(const float &dxIn, const float &dyIn, const float &dzIn) { FloatPoint3 delta(dxIn, dyIn, dzIn); MoveBy(delta); }
        inline void                         MoveByX(const float &dxIn) { pt_min.SetX(pt_min.GetX() + dxIn); pt_max.SetX(pt_max.GetX() + dxIn); }
        inline void                         MoveByY(const float &dyIn) { pt_min.SetY(pt_min.GetY() + dyIn); pt_max.SetY(pt_max.GetY() + dyIn); }
        inline void                         MoveByZ(const float &dzIn) { pt_min.SetZ(pt_min.GetZ() + dzIn); pt_max.SetZ(pt_max.GetZ() + dzIn); }

        inline void                         Merge(const Box& bIn); // grow the box to match the min and max extents of each box
        inline void __vectorcall            Merge(const FloatPoint3 ptIn); // grow the box to contain the point
        inline void __vectorcall            Grow(const FloatPoint3 scale3In) { auto s = scale3In * GetSize(); SetWHD(s); }
        // Accessors
        inline FloatPoint3                  GetSize() const { return Abs(pt_max - pt_min); } // width, height & depth
        inline FloatPoint3                  GetExtents() const { return (pt_max - pt_min) * 0.5f; }
        inline FloatPoint3                  GetCenter() const { return (pt_max + pt_min) * 0.5f; }

        inline const FloatPoint3 &          GetMin() const { return pt_min; }
        inline const FloatPoint3 &          GetMax() const { return pt_max; }
        inline float                        GetWidth() const { return std::abs(pt_max.GetX() - pt_min.GetX()); }
        inline float                        GetHeight() const { return std::abs(pt_max.GetY() - pt_min.GetY()); }
        inline float                        GetDepth() const { return std::abs(pt_max.GetZ() - pt_min.GetZ()); }
        inline float3                       GetDiagonal() const { return pt_max - pt_min; }
        inline float3                       GetSurfaceArea() const { auto d = GetDiagonal(); return 2.f * (d.f[0] * d.f[1] + d.f[0] * d.f[2] + d.f[1] * d.f[2]); }
        inline float3                       GetVolume() const { auto d = GetDiagonal(); return 2.f * (d.f[0] * d.f[1] * d.f[2]); }

        vector<pair<FloatPoint3, CornerDescription>> IdentifyCorners(const vector<FloatPoint3>& pointsIn, const Quaternion* quaternionIn = nullptr) const;
        std::vector<FloatPoint3>            GetCorners8Transformed(const DirectX::FXMMATRIX& M); // scaled, rotate and translate a "unit" Box to a model bounding box
        std::vector<FloatPoint3>            GetCorners8(const Quaternion * quaternionIn = nullptr) const; // return all 8 corners with enum CornerDescription indexing
        std::vector<FloatPoint3>            GetCornersTop4(const Quaternion * quaternionIn = nullptr); 
        std::vector<FloatPoint3>            GetCornersBottom4(const Quaternion * quaternionIn = nullptr);
        std::vector<FloatPoint3>            GetCornersLowest4(const Quaternion * quaternionIn = nullptr);
        void                                GetCorners8(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr) const; // array size passed in is [8], axis aligned
        void                                GetCornersTop4(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr); // array size passed in is [4]
        void                                GetCornersBottom4(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr); // array size passed in is [4]
        void                                GetCornersLowest4(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn = nullptr); // array size passed in is [4]

        inline Quad                         GetQuadFront(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[8]; GetCorners8(a, quaternionIn); return Quad(a[ltf], a[lbf], a[rbf], a[rtf] ); }
        inline Quad                         GetQuadRight(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[8]; GetCorners8(a, quaternionIn); return Quad(a[rtf], a[rbf], a[rbb], a[rtb]); }
        inline Quad                         GetQuadBack(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[8]; GetCorners8(a, quaternionIn); return Quad(a[rtb], a[rbb], a[lbb], a[ltb]); }
        inline Quad                         GetQuadLeft(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[8]; GetCorners8(a, quaternionIn); return Quad(a[ltb], a[lbb], a[lbf], a[ltf]); }
        inline Quad                         GetQuadTop(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[4]; GetCornersTop4(a, quaternionIn); return Quad(a); }
        inline Quad                         GetQuadBottom(const Quaternion * quaternionIn = nullptr) { DirectX::XMFLOAT3 a[4]; GetCornersBottom4(a, quaternionIn); return Quad(a); }
        // Assignments                      
        inline void                         SetZero() { pt_min.SetZero(); pt_max.SetZero(); }
        inline void    __vectorcall         Set(const FloatPoint3 &pt_minIn, const FloatPoint3 &pt_maxIn) { pt_min = pt_minIn; pt_max = pt_maxIn; } // defines the box with min (pt_min) and max (pt_max) points in RHS
        inline void                         Set(const DirectX::BoundingBox &in) { auto c = DirectX::XMLoadFloat3(&in.Center); auto e = DirectX::XMLoadFloat3(&in.Extents); pt_min = c - e; pt_max = c + e; }
        inline void    __vectorcall         SetCenterAndExtents(const FloatPoint3 &cIn, const FloatPoint3 &eIn) { pt_min = cIn - eIn; pt_max = cIn + eIn; } // defines the box with min (pt_min) and max (pt_max) points in RHS
        inline void    __vectorcall         Setpt_min(const FloatPoint3 &pt_minIn) { pt_min = pt_minIn; }
        inline void    __vectorcall         Setpt_max(const FloatPoint3 &pt_maxIn) { pt_max = pt_maxIn; }
        inline void    __vectorcall         SetWHD(const FloatPoint3 &whdIn) { FloatPoint3 offset = Abs(whdIn) * 0.5f; FloatPoint3 c = GetCenter(); pt_min = c - offset; pt_max = c + offset; }
        inline void    __vectorcall         SetCenter(const FloatPoint3 &centerIn) { FloatPoint3 delta = centerIn - GetCenter(); MoveBy(delta); }
        void __vectorcall                   SetAABBfromThisTransformedBox(FXMMATRIX M);
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
    class alignas(16) Frustum
    {
        /* variables */
    public:
        enum CornerID { lbb = 0, lbf, rbf, rbb, ltb, ltf, rtf, rtb, INVALID };
        enum PlaneID { kNearPlane = 0, kFarPlane, kLeftPlane, kRightPlane, kTopPlane, kBottomPlane, kINVALID };
    protected:
        FloatPoint3                         _FrustumCorners[8];
        Plane                               _FrustumPlanes[6];
        /* methods */
    public:
        // Construction/Destruction
        Frustum() { ; }
        Frustum(const DirectX::XMMATRIX &projectionMatrixIn);
        Frustum(const Box& in, const quat* rQIn = nullptr) { ConstructOrthographicFrustum(in, rQIn); }

        Frustum(const Frustum &in) { *this = in; } // copy 
        Frustum(Frustum &&in) { *this = std::move(in); } // move
        virtual ~Frustum() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Frustum*>(p)); }
        inline Frustum & operator= (const Frustum &in) = default; // copy assignment
        inline Frustum & operator= (Frustum &&in) = default; // move assignment
        inline Frustum & operator*= (const DirectX::XMMATRIX &m);
        friend Frustum operator* (Frustum lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)
        // Functionality
        bool __vectorcall                   Intersect(const Box &boxIn) const;
        bool __vectorcall                   Intersect(const Sphere &sphereIn) const;
        // Accessors
        inline FloatPoint3&                 GetFrustumCorner(Frustum::CornerID id) { assert(id < Frustum::CornerID::INVALID); return _FrustumCorners[id]; }
        inline Plane&                       GetFrustumPlane(Frustum::PlaneID id) { assert(id < Frustum::PlaneID::kINVALID); return _FrustumPlanes[id]; }
    private:
        void                                ConstructPerspectiveFrustum(float HTan, float VTan, float NearClip, float FarClip); // Perspective frustum constructor (for pyramid-shaped frusta)
        void                                ConstructOrthographicFrustum(float Left, float Right, float Top, float Bottom, float NearClip, float FarClip); // Orthographic frustum constructor (for box-shaped frusta)
        void                                ConstructOrthographicFrustum(const Box& in, const quat *rQIn = nullptr); // Frustrum that is orthographic and rotated by an arbitrary axis
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
    *    Contact
    *        Location of a contact point between two objects
    ******************************************************************************/
    class alignas(16) Contact
    {
        /* variables */
    public:
        std::vector<float3>                 _contactVerts;
        float3                              _Obj1Pt;
        float3                              _Obj2Pt;
    protected:
        bool                                _staticContact = false; // has not changed since last update *** TO DO ***
        Distance                            _directionContactObj1_to_ContactObj2; // normal & mag in world space
        float                               _momentumChangeFromPenetration = 0.0f; // *** TO DO ***

    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Contact> Create() { return std::make_shared<Contact>(); }
        Contact() = default;
        Contact(const Contact& in) { *this = in; } // forward to copy assignment
        Contact(Contact&& in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~Contact() { ; }

        // Conversions
        // Operators 
        void* operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void* p) { _aligned_free(static_cast<Contact*>(p)); }
        inline Contact& operator= (const Contact& other) = default; // copy assign
        inline Contact& operator= (Contact&& other) = default; // move assign
        explicit operator bool() const { return (bool)GetHasContact(); } // valid
        bool operator !() const { return !(bool)GetHasContact(); } // invalid
        // Math Operators
        // Init/Start/Stop/Destroy
        // Functionality
        // Accessors
        const size_t                        GetHasContact() const { return _contactVerts.size(); }
        bool                                GetHasPenetration() const;
        auto&                               GetPenetration() const { return _directionContactObj1_to_ContactObj2; }
        bool                                GetPenetrationDepth() const { return _directionContactObj1_to_ContactObj2.Get_magnitude(); }
        bool                                GetPenetrationDirection() const { return _directionContactObj1_to_ContactObj2.Get_unit_direction(); }
        // Assignments
        void                                SetContactPenetration(const Distance& Obj1_to_Obj2) { _directionContactObj1_to_ContactObj2 = Obj1_to_Obj2; _staticContact = false; }
        void                                SetObjects(const float3& obj1Pt, const float3& obj2Pt) { _Obj1Pt = obj1Pt; _Obj2Pt = obj2Pt; }
        inline void                         SetNoContact() { _contactVerts.clear(); }

        bool                                SAT_ContactPointsFromOBBonOBBIntersection(const Box& A, const Quaternion qA, const Box& B, const Quaternion qB, const std::vector<float3>& cornersA, const std::vector<float3>& cornersB);
    protected:
        inline std::vector<float3>          GetContactVerts(const std::vector<float3>& corners, const Distance& dist);
        inline std::vector<float3>          ClosestPointsToFaces(std::vector<float3>* contactVertsA, std::vector<float3>* contactVertsB);
        inline float3                       ClosestPointEdgeEdge(const std::vector<float3>& contactVertsA, const std::vector<float3>& contactVertsB);
        inline bool                         VertInsideFace(const std::vector<float3>& contactVerts, const float3& p0, const float& planeErr = 0.0f);
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
        inline LineMesh & operator= (const LineMesh &in) = default; // copy assignment
        inline LineMesh & operator= (LineMesh &&in) = default; // move assignment
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

        auto                                GetVertexAddr(const uint32_t indexIn) const { assert(indexIn < GetNumIndicies()); return GetVB() + *(GetIB() + indexIn) * _vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        FloatPoint3                         GetVertexPosition(const uint32_t indexIn) const;
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
    *        a mesh.
    ******************************************************************************/
    class alignas(16) TriangleMesh
    {
        /* variables */
    private:
        std::string                         _name;
        std::string                         _materialName = "default";
        VertexFormat                        _vertexFormat;
        uint8_t *                           _vb = nullptr; // reference, do not free
        uint32_t *                          _ib = nullptr; // reference, do not free
        uint32_t                            _vbStart = 0; // vertex buffer start offset (starting vertex #) to add to address
        uint32_t                            _ibStart = 0; // index buffer start offset (starting index #) to add to address
        uint32_t                            _numTriangles = 0; // assumes ib is a triangle list with 3 index per triangle
        
        Box                                 _boundingBox; // calculate and store after data definitions
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<TriangleMesh> Create() { return std::make_shared<TriangleMesh>(); }
        static std::unique_ptr<TriangleMesh> CreateUnique() { return std::make_unique<TriangleMesh>(); }
        // Construction/Destruction
        TriangleMesh() = default;
        TriangleMesh(const TriangleMesh &in) { *this = in; } // forward to copy assignment
        TriangleMesh(TriangleMesh &&in) { *this = std::move(in); } // forward to move assignment
        TriangleMesh(uint32_t numTrianglesIn, VertexFormat vfIn, uint32_t vbStartIn, uint32_t ibStartIn, uint8_t *vbIn, uint32_t *ibIn) { Initialize(numTrianglesIn, vfIn, vbStartIn, ibStartIn, vbIn, ibIn); }

        virtual ~TriangleMesh() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<TriangleMesh*>(p)); }
        inline TriangleMesh & operator= (const TriangleMesh &in) = default; // copy assignment
        inline TriangleMesh & operator= (TriangleMesh &&in) = default; // move assignment
        inline TriangleIndexed operator[] (uint32_t indexIn) const { return GetTriangleIndexed(indexIn); } // accessor
        // Functionality
        virtual void                        Initialize(uint32_t numTrianglesIn, VertexFormat vfIn, uint32_t vbStartIn, uint32_t ibStartIn, uint8_t *vbIn, uint32_t *ibIn) { _numTriangles = numTrianglesIn; _vertexFormat = vfIn; _vbStart = vbStartIn; _ibStart = ibStartIn; _vb = vbIn; _ib = ibIn; }
        virtual Box                         CalculateBoundingBox(); // call after loading or changing position data
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
        
        auto                                GetVertexAddr(const uint32_t indexIn) const { assert(indexIn < GetNumIndicies()); return GetVB() + *(GetIB() + indexIn) * (uint32_t)_vertexFormat.GetByteSize(); } // byte pointer to start of vertex data
        FloatPoint3                         GetVertexPosition(const uint32_t indexIn) const;
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
        HeightGrid(HeightGrid &&in) { *this = std::move(in); } // forward to move assignment
        HeightGrid(const TriangleMesh & triMeshIn, long xIn=0l, long zIn=0l) { InitializeXZ(triMeshIn, xIn, zIn); }

        virtual ~HeightGrid() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<HeightGrid*>(p)); }
        inline HeightGrid & operator= (const HeightGrid &in) = default; // copy assignment
        inline HeightGrid & operator= (HeightGrid &&in) = default; // move assignment
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
        /* structures */
        struct Properties
        {
            uint16_t ver = 0;
            struct ColorProp
            {
                float ambient[3] = { 0.4f, 0.4f, 0.4f };
                float diffuse[3] = { 0.5f, 0.5f, 0.5f };
                float specular[3] = { 0.1f, 0.1f, 0.1f };
                float emissive[3] = { 0.f, 0.f, 0.f };
                float transparent[3] = { 0.5f, 0.5f, 0.5f }; // light passing through a transparent surface is multiplied by this filter color
            } color;

            float indexOfRefraction = 1.f; // 0.001 to 10; 1.0 means that light does not bend, glass has value 1.5
            float specularStrength = 1; // exponent 0 to 1000, shininess
            float dissolved = 1.f; // 0 transparent to 1 opaque (eg. alpha)
            float transmissionFilter = 1.f; // 0 to 1
            bool isCutOut = false; // color (diffuse) is always on, cutout specifies if material has alpha channel (for rendering order)
            bool isAmbient = true; // ambient on/off
            bool isShiny = true; // specular on/off
            bool isEmissive = false; // emissive on/off
        };
        struct FileNames
        {
            uint16_t ver = 0;
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
        /* variables */
    private:
        std::string                         _name;
        Properties                          _properties;
        FileNames                           _fileNames;
        uint8_t                             _shaderMethod = 0;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Material>    Create() { return std::make_shared<Material>(); }
        Material() { ; }
        Material(const std::string nameIn) : _name(nameIn) { ; }
        Material(const Material &in) { *this = in; } // forward to copy assignment
        Material(Material &&in) { *this = std::move(in); } // forward to move assignment
        virtual ~Material() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Material*>(p)); }
        inline Material & operator= (const Material &in) = default; // copy assign
        inline Material & operator= (Material &&in) = default; // move assign
        // Conversions
        // Functionality
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
    *   Remarks: This class keeps master data but not the mesh indexing.  Mesh data
    *            will reference master data to build up the mesh and is dependent
    *            on the type of mesh.
    ******************************************************************************/
    class alignas(16) ModelScaffold : public std::enable_shared_from_this<King::ModelScaffold> // base class
    {
         /* variables */
    protected:
        std::string                         _modelName;

        MemoryBlock<uint8_t>                _vertexBufferMaster; // set stride to data size after vertexFormat is initialized
        MemoryBlock<uint32_t>               _indexBufferMaster; // 4,294,967,295 maximum verticies with 32 bit indexing

        VertexFormat                        _vertexFormat;
        IndexFormat                         _indexFormat = IndexFormat::uint32;

        Box                                 _boundingBox;
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<ModelScaffold>    Create() { return std::make_shared<ModelScaffold>(); }
        // Construction/Destruction
        ModelScaffold() { _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3); _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize()); }
        ModelScaffold(const ModelScaffold &in) { *this = in; } // forward to copy assignment
        ModelScaffold(ModelScaffold &&in) { *this = std::move(in); } // forward to move assignment

        virtual ~ModelScaffold() { Destroy(); }

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<ModelScaffold*>(p)); }
        inline ModelScaffold & operator= (const ModelScaffold &in) = default; // copy assignment
        inline ModelScaffold & operator= (ModelScaffold &&in) = default; // move assignment
        // Functionality
        virtual void                        Initalize(const VertexFormat &vertexFormatIn, size_t verticiesIn, size_t indiciesIn) { _vertexFormat = vertexFormatIn; _vertexBufferMaster.Initialize(verticiesIn * _vertexFormat.GetByteSize()); _indexBufferMaster.Initialize(indiciesIn); }
        void                                Destroy() { _indexBufferMaster.Destroy(); _vertexBufferMaster.Destroy(); _boundingBox.SetZero(); }

        virtual bool                        Load(std::string fileNameIN) { return false; }
        virtual bool                        Save(std::string fileNameIN) { return false; }

                                            // define in derived class
        virtual Box                         CalculateBoundingBox() { Box rtn; rtn.SetZero(); return rtn; } 
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
        const auto &                        GetModelName() const { return _modelName; }
        const auto &                        GetBoundingBox() const { return _boundingBox; }

        uint8_t*                            GetVertexAddr(const uint32_t indexIn, uint32_t vbStartIn = 0, uint32_t ibStartIn = 0); // byte pointer to start of vertex data
        uint32_t                            GetVertexCount() const { return (uint32_t)(_vertexBufferMaster.GetLength() / _vertexFormat.GetByteSize()); }
        auto &                              GetVertexFormat() const { return _vertexFormat; }
        auto                                GetVertexStride() const { return _vertexFormat.GetByteSize(); }
        auto &                              GetVertexBufferMaster() { return _vertexBufferMaster; }

        auto &                              GetIndexCount() const { return _indexBufferMaster.GetLength(); }
        uint32_t                            GetIndexStride() const { return IndexFormat::uint32 ? 4 : 2;} // in bytes
        auto &                              GetIndexBufferMaster() { return _indexBufferMaster; }

        vector<vector<float>>               GetPositions() { return GetDataAttribute(VertexAttrib::enumDesc::position); }
        vector<vector<float>>               GetColors() { return GetDataAttribute(VertexAttrib::enumDesc::color); }
        vector<vector<float>>               GetTextureCoordinates() { return GetDataAttribute(VertexAttrib::enumDesc::textureCoord); }
        vector<vector<float>>               GetNormals() { return GetDataAttribute(VertexAttrib::enumDesc::normal); }
        vector<vector<float>>               GetTangents() { return GetDataAttribute(VertexAttrib::enumDesc::tangent); }
        vector<vector<float>>               GetBiTangents() { return GetDataAttribute(VertexAttrib::enumDesc::bitangent); }
        vector<vector<float>>               GetBoneIndicies() { return GetDataAttribute(VertexAttrib::enumDesc::boneIndicies); }
        vector<vector<float>>               GetBoneWeights() { return GetDataAttribute(VertexAttrib::enumDesc::boneWeights); }
        // Assignments
        void                                AddMasterVertexData(const MemoryBlock<uint8_t> &vdIn) { _vertexBufferMaster.Append(vdIn); }
        void                                AddMasterIndexData(const MemoryBlock<uint32_t> &idIn) { _indexBufferMaster.Append(idIn); }
        void                                AddVertexAttribute(const VertexAttrib::enumDesc & propertyIn, const VertexAttrib::enumFormat & formatIn);

        void                                SetModelName(const std::string modelNameIN) { _modelName = modelNameIN; }
        void                                SetIndex(const size_t& indexIn, const uint32_t& value);
        void                                SetVertex(const size_t& vertexIndexIn, uint8_t * dataIn);
        void                                SetVertexStream(const size_t& vertexIndexIn, uint8_t* dataIn, size_t numVertex);
        void                                SetIndexStream(const size_t& startIndexIn, uint8_t* dataIn, size_t numIndicies);
        void                                SetIndexStream(const size_t& startIndexIn, uint32_t* dataIn, size_t numIndicies);
        void                                SetVertexElement(const size_t & vertexIndexIn, const VertexAttrib::enumDesc & propertyIn, uint8_t * dataIn);
        void                                SetVertexFormat(const VertexFormat & vfIn) { _vertexFormat = vfIn; _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize()); }
    private:
        vector<vector<float>>               GetDataAttribute(VertexAttrib::enumDesc attributeEnum, uint32_t vbStartIn = 0, uint32_t ibStartIn = 0);
    };
    /******************************************************************************
    *    LineModel
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
        LineModel() { ; }
        LineModel(const LineModel &in) { *this = in; } // forward to copy assignment
        LineModel(LineModel &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~LineModel() { Destroy(); }

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<LineModel*>(p)); }
        inline LineModel & operator= (const LineModel &in); // copy assignment
        inline LineModel & operator= (LineModel &&in) = default; // move assignment
        // Functionality
        void                                Destroy() { _meshes.clear(); King::ModelScaffold::Destroy(); }

        virtual Box                         CalculateBoundingBox() override;
        // Accessors
        const size_t                        GetNumMeshes() const { return _meshes.size(); }
        std::shared_ptr<LineMesh>           GetMesh(size_t index) { assert(index < _meshes.size()); return _meshes[index]; }
        // Assignments
        void                                AddMesh(std::shared_ptr<LineMesh> meshIn) { _meshes.push_back(meshIn); }
        std::shared_ptr<LineMesh>           CreateMesh(uint32_t numLinesIn, uint32_t vbStartIn=0, uint32_t ibStartIn=0)
        {
           assert((bool)GetVertexBufferMaster);
           assert((bool)GetIndexBufferMaster);
           auto ptr = LineMesh::Create(numLinesIn, GetVertexFormat(), vbStartIn, ibStartIn, &GetVertexBufferMaster().GetData(), &GetIndexBufferMaster().GetData());
           _meshes.push_back(ptr);
           return _meshes.back();
        }
    };
    /******************************************************************************
    *    Model
    ******************************************************************************/
    class alignas(16) Model : public ModelScaffold
    {
        friend class Model_IO; // separate the loading/saving functionality so it can be overridded easily with a new class and operate on Model

        /* variables */
    protected:
        std::vector<TriangleMesh>           _meshes;
        std::map<std::string, std::shared_ptr<Material>> _materials;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Model>       Create() { return std::make_shared<Model>(); }
        // Construction/Destruction
        Model() { _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3); _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize()); }
        Model(const Model &in) { *this = in; } // forward to copy assignment
        Model(Model &&in) { *this = std::move(in); } // forward to move assignment

        virtual ~Model() { Destroy(); }

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Model*>(p)); }
        inline Model & operator= (const Model &in); // copy assignment
        inline Model & operator= (Model &&in) = default; // move assignment

        // Functionality
        void                                Destroy() { _meshes.clear(); King::ModelScaffold::Destroy(); }
        
        virtual bool                        Load(std::string fileNameIN) override;
        virtual bool                        Save(std::string fileNameIN) override;
        
        virtual Box                         CalculateBoundingBox() override;
        void                                CalculateTangentsAndBiTangents();

        void                                OptimizeVertexBuffer() { for (auto & m : _meshes) { OptimizeMeshVertexBuffer(m); } }
        void                                OptimizeMeshVertexBuffer(const TriangleMesh & forMeshIn); // re-order to match index buffer ordering

        void                                LogReport(const std::wstring fileNameIn);
        // Accessors
        const auto                          GetNumMeshes() const { return _meshes.size(); }
        TriangleMesh &                      GetMesh(size_t index) { assert(index < _meshes.size()); return _meshes[index]; }
        std::vector<TriangleMesh> &         GetMeshes() { return _meshes; }

        const auto                          GetNumMaterials() const { return _materials.size(); }
        std::shared_ptr<Material>           GetMaterial(string name) { assert(_materials.count(name)); return _materials[name]; }
        auto &                              GetMaterials() { return _materials; }

        virtual std::size_t                 GetBoneCount() const { return 0; } // base class has no bones
        // Assignments
        void                                AddMesh(const TriangleMesh &meshIn) { _meshes.push_back(meshIn); }
        void                                AddMaterial(std::shared_ptr<Material> mtl_IN);
    };

    /******************************************************************************
    *    BoneHierarchy - modifier for geometry to allow skin transforms of a 
    *    bone hierarchy.
    ******************************************************************************/
    class alignas(16) BoneHierarchy
    {
        /* structures */
        struct Bone
        {
            std::string                     _name; // unique so we can build the skelton hiearchy after saves and loads
            DirectX::XMFLOAT4X4             _transform; // defines the bone orientation
        };
        struct Skeleton
        {
            std::vector<Bone>                _bones; // sorted such that a child bone never comes before a parent bone
            std::vector<DirectX::XMFLOAT4X4> _toParentTransform; // size of bones, contains the hiearchy of the bone structure
            // also note that toParentTransform[0] is alway identity as the top most node of the structure
        };
        /* variables */
    public:
        Skeleton                            _skeleton;
        std::vector<uint8_t>                _boneInfluenceCount; // size is equal to # of verticies in mesh and contains the number of bones influencing each vertex
        std::vector<uint8_t>                _boneIndex; // size is sum of boneInfluenceCount values, indexies into skeleton.bones and is ordered in vertex order
        std::vector<float>                  _boneWeight; // size matches boneIndex and gives the weight of each

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<BoneHierarchy>   Create() { return std::make_shared<BoneHierarchy>(); }
        BoneHierarchy() { ; }
        BoneHierarchy(const BoneHierarchy &in) { *this = in; } // forward to copy assignment
        BoneHierarchy(BoneHierarchy &&in) { *this = std::move(in); } // forward to move assignment
        virtual ~BoneHierarchy() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<BoneHierarchy*>(p)); }
        inline BoneHierarchy & operator= (const BoneHierarchy &in) = default; // copy assign
        inline BoneHierarchy & operator= (BoneHierarchy &&in) = default; // move assign
        // Conversions
        // Functionality
        // Accessors
        auto                                GetBoneCount() const { return _skeleton._bones.size(); }
        // Assignments
    };

    /******************************************************************************
    *    SkinnedModel
    ******************************************************************************/
    class alignas(16) SkinnedModel : public Model
    {
        friend class Model_IO;

        /* variables */
    public:
        std::unique_ptr<BoneHierarchy>      _boneHierarchy;
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<SkinnedModel>Create() { return std::make_shared<SkinnedModel>(); }
        // Construction/Destruction
        SkinnedModel() { ; }
        SkinnedModel(const SkinnedModel &in) { *this = in; } // forward to copy assignment
        SkinnedModel(SkinnedModel &&in) { *this = std::move(in); } // forward to move assignment
        virtual ~SkinnedModel() { Destroy(); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<SkinnedModel*>(p)); }
        inline SkinnedModel & operator= (const SkinnedModel &in) { Destroy(); Model::operator=(in); if (in._boneHierarchy) { _boneHierarchy = std::make_unique<BoneHierarchy>(); *_boneHierarchy = *in._boneHierarchy; } return *this; } // copy assignment
        inline SkinnedModel & operator= (const Model &in) { Destroy(); Model::operator=(in); return *this; } // copy assignment
        inline SkinnedModel & operator= (Model &in) { Destroy(); Model::operator=(std::move(in)); return *this; } // move assignment
        inline SkinnedModel & operator= (SkinnedModel &&in) { std::swap(_boneHierarchy, in._boneHierarchy); Model::operator=(std::move(in)); return *this; } // move assignment
        // Functionality
        virtual void                        Destroy() { delete _boneHierarchy.release(); Model::Destroy(); }
    
        std::vector<float3>                 CalculateVertexSkinPositions(size_t meshIndex = 0);
        // Accessors
        virtual std::size_t                 GetBoneCount() const { if (_boneHierarchy) return _boneHierarchy->GetBoneCount(); else return 0; }
        // Assignments
    };
}
