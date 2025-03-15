/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          2DGeometryKing

Description:    CPU side 2D geometry utilizing our SIMD math library for 
                accelerated math operations.  Includes json support so that
                these data types may be serialized easily for network 
                communication, file storage, or inspection during debugging.
                Uses our generic MemoryBlock class shared with 3DGeometryKing
                to draw primitives. The MemoryBlock class is inherited to keep
                width and height dimensions

Usage:          Intersection and collission detection for physics and games
                Simple 2D CAD applications
                Simple 3D Paint applications
                Visualization of memory through image exports in .TGA format
                                
Dependicies:    DirectXMath https://learn.microsoft.com/en-us/windows/win32/dxmath/directxmath-portal
                JSON for Modern C++ https://github.com/nlohmann/json

Contact:        https://chrisking340.github.io/GeometryKing/

Copyright (c) 2019 - 2023 Christopher H. King

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
// needed for MS compiler for had breaking change for std::make_shared
#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif

#ifndef __cplusplus
#error 3DGeometry requires C++
#endif

#if defined(_MSC_VER) && (_MSC_VER < 1920)
#error requires Visual C++ 2019 or later.
#endif

#define KING_2DGEOMETRY_VERSION_MAJOR 2
#define KING_2DGEOMETRY_VERSION_MINOR 4
#define KING_2DGEOMETRY_VERSION_PATCH 0
/*
    History:
        KING_2DGEOMETRY_VERSION_MINOR 1 Release:
            Authored circle 2D class
        KING_2DGEOMETRY_VERSION_MINOR 2 Release:
            Authored std:cout functionality with ostream and istream to all 2D classes
        KING_2DGEOMETRY_VERSION_MINOR 3 Release 12/24/2021:
            Authored polygon 2D class Polygon2DF
        KING_2DGEOMETRY_VERSION_MINOR 4 Release 5/8/2022:
            Authored clip function to Rectangle2DF for Rectangle2DF, Line2DF, Triangle2DF and Polygon2DF that returns a copy with object(s) clipped to the boundaries of the Rectangle2DF
            Authored bounding geometry of Rectangle2DF and Circle2DF from polygons as Set(Polygon2DF) and constructors.
        KING_2DGEOMETRY_VERSION_MINOR 5 Release 5/21/2022:
            Authored Offset function to Polygon2DF
            Authored Offset function to Line2DF
            Authored Join function to Line2DF
            Authored IsParallel function to Line2DF
            Authored IsPerpendicular function to Line2DF

        KING_2DGEOMETRY_VERSION_MAJOR 2 Release 9/25/2022:
            Authored Class ImageBlock for raw texture storage and methods to draw primitives to a buffer that can be used for texture uploads by a graphics API.
            Authored Class ImageTGA for saving and loading ImageBlock data in an exportable file format. TGA was chosen for it simplicity and age, meaning it is widely supported. 
        KING_2DGEOMETRY_VERSION_MINOR 1 Release 10/13/2022:
            Authored Rectangle2DF::FindNearestPoint(float2, Matrix) to Rectangle2DF to transform the rectangle given a rotation transform
        KING_2DGEOMETRY_VERSION_MINOR 2 Release 12/14/2022:
            Comment: Now that we can use ImageTGA im; im.WriteTGA("fileName.tga"); to export graphics to test the library efficiently, implementing collision tests for basic geometry beyound just a point.
            Authored Triangle2DF::Intersects(Triangle2DF) using the separating axis theorem
            Authored Triangle2DF Conversion operator to Polygon2DF to allow for use of Polygon2DF::Intersects(Polygon2DF polyIn)
            Authored Circle2DF::Transform(DirectX::XMMATRIX m) to modify data by an external transformation
            Authored Circle2DF::Intersects(Line2DF lIn) specialized tests unique to a circle and a line
            Authored Polygon2DF::Intersects(Polygon2DF polyIn) using the separating axis theorem generalized for two different arbitrary size polygons
            Authored Polygon2DF::Intersects(Line2DF other) to test end points and then arbitrary number of edges on line intersection
            Authored Polygon2DF::Intersects(Circle2DF other) specialized tests unique to a circle and arbitrary number of lines
            Authored Rectangle2DF Conversion operator to Polygon2DF to allow for use of Polygon2DF::Intersects(Polygon2DF polyIn)
            Authored Rectangle2DF::Transformation(DirectX::XMMATRIX m) to return a non-axis aligned rectangle from a matrix that was updated externally when transform to world space needed updating
            Authored Rectangle2DF::Transformation(FloatPoint2 scaleIn, FloatPoint2 rotOrigin, DirectX::XMVECTOR rotQuat, FloatPoint2 trans) for use by a class that keeps a transform to world space
            Authored Rectangle2DF::Rotate(FloatPoint2 originIn, float radians) to return a non-axis aligned rectangle rotated about an origin. Can pass in Rectangle2DF::Center();
            Authored Rectangle2DF::Intersects(const FloatPoint2 pt2In, const float rectRotationInRadians) for rotated rectangles. Useful for UI implementations of rotated hit boxes
            Authored Rectangle2DF::Transformation(DirectX::XMMATRIX m) to return a Polygon2DF to a non-axis aligned rectangle.
        KING_2DGEOMETRY_VERSION_MINOR 3 Release 7/28/2023:
            Authored ImageBlock::FilterGaussianBlur() a gaussian blur of the source image 
            Authored ImageBlock::FloodFill(unsigned int row, unsigned int col, const MemoryBlock<unsigned char> &newColor, const MemoryBlock<unsigned char>& oldColor) to fill the image tracing through an oldColor path
            To use code with 2D platform games added rectangle collision detection for a moving rectangle and a static rectangle (environment tiles). Onelonecoder had a nice video on how to do this
            Authored Rectangle2DF::CollisionWithRay(...)
            Authored Rectangle2DF::CollisionWithDynamicRect(...)
            Authored Rectangle2DF::ResolveDynamicRectVsRect(...)
        KING_2DGEOMETRY_VERSION_MINOR 4 Release 10/10/2023: ImageBlock benefits from base class MemoryBlock gaining WriteCppHeader(const std::string& outputFileName, const std::string& variableName) to easily
            export MemoryBlock to imbed in C++ applications. Very useful to imbed images in code and eliminate distributing image files with executables.
*/
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <memory>
#include <utility>
#include <limits>
#include "..\..\MathSIMD\MathSIMD.h"
#include "..\General\MemoryBlock.h"

#include "..\..\json\single_include\nlohmann\json.hpp"
//#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

namespace King {
    // 2D objects
    class Line2DF; // SIMD
    class Triangle2DF; // SIMD
    class Rectangle2DF; // SIMD
    class Rectangle2D; // not accelerated, replaces Windows RECT class
    class Circle2DF; // SIMD
    class Polygon2DF; // SIMD

    // alias
    typedef Line2DF         lineF; // float data types
    typedef Triangle2DF     triF; // float data types
    typedef Rectangle2DF    rectF; // float data types
    typedef Rectangle2D     rect; // integer data types
    typedef Circle2DF       circleF; // float data types
    typedef Polygon2DF      polygonF; // float data types

    /******************************************************************************
    *   Line2DF
    *   pt1 ------- pt2
    ******************************************************************************/
    class alignas(16) Line2DF
    {
        /* variables */
    public:
        FloatPoint2 pt[2];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Line2DF> Create() { return std::make_shared<Line2DF>(); }
        static std::unique_ptr<Line2DF> CreateUnique() { return std::make_unique<Line2DF>(); }
        // Construction/Destruction
        Line2DF() = default;
        Line2DF(const Line2DF &in) { *this = in; } // copy, involk operator=(&int)
        Line2DF(Line2DF &&in) noexcept { *this = std::move(in); } // move, involk operator=(&&in)
        Line2DF(const FloatPoint2 &pt1In, const FloatPoint2 &pt2In) { pt[0] = pt1In; pt[1] = pt2In; }
        Line2DF(std::initializer_list<FloatPoint2> il) { assert(il.size() < 3); std::size_t count = 0; for (auto & each : il) { pt[count] = each; ++count; } }

        virtual ~Line2DF() = default;

        // Operators 
        void * operator new (std::size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Line2DF*>(p)); }
        inline Line2DF & operator= (const Line2DF &in) { Set(in); return *this; } // copy assignment
        inline Line2DF & operator= (Line2DF &&in) = default; // move assignment
        // Conversions
        inline explicit operator bool() const { return pt[0] != pt[1]; } // valid
        inline bool operator !() const { return pt[0] == pt[1]; } // invalid
        // Functionality
        bool __vectorcall                   Intersects(const Line2DF &lineIn, FloatPoint2 *intersectPointOut = nullptr);
        FloatPoint2 __vectorcall            FindNearestPointOnLineSegment(const FloatPoint2 &pointIn);

        void __vectorcall                   Transform(DirectX::XMMATRIX m);

        void                                Traverse(std::function<void(IntPoint2 ptOut)> callBack) const; // rasterize the line and callback for each point along the line
        
        void                                Offset(const float distIn); // translate this line by this amount, using the normal to the outside of a CCW rotation as the direction
        void                                Offset(const float2 distIn); // translate this line by this amount
        bool                                Join(Line2DF& otherlineIn); // modifies both lines to join the ends, returns false if parrallel and no modifications
        
        bool                                IsParallel(const Line2DF& lineIn) const;
        bool                                IsPerpendicular(const Line2DF& lineIn) const;
        // Accessors
        const auto &                        GetVertex(const uint32_t vertexIndexIn) const { return pt[vertexIndexIn]; }
        FloatPoint2                         GetLength() const { auto rtn = pt[1] - pt[0]; rtn.Absolute(); return rtn; }
        FloatPoint2                         GetDirectionFrom0to1() const { auto rtn = pt[1] - pt[0]; rtn.Normalize(); return rtn; }
        FloatPoint2                         GetDirectionFrom1to0() const { auto rtn = pt[0] - pt[1]; rtn.Normalize(); return rtn; }
        // Assignments
        inline void __vectorcall            Set(const Line2DF &in) { pt[0] = in.pt[0]; pt[1] = in.pt[1]; }
        inline void __vectorcall            SetVertex(const uint32_t vertexIndexIn, const FloatPoint2 &in) { pt[vertexIndexIn] = in; }
    };
    /******************************************************************************
    *   Triangle2DF
    *           edge3
    *      pt0 ------- pt2
    *          \     /
    *    edge1  \   / edge2
    *            \ /
    *            pt1
    *   CCW rotation for RHS face normals
    ******************************************************************************/
    class alignas(16) Triangle2DF
    {
        /* variables */
    public:
        FloatPoint2 pt[3];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Triangle2DF> Create() { return std::make_shared<Triangle2DF>(); }
        static std::unique_ptr<Triangle2DF> CreateUnique() { return std::make_unique<Triangle2DF>(); }
        // Construction/Destruction
        Triangle2DF() = default;
        Triangle2DF(const Triangle2DF &in) { *this = in; } // copy, involk operator=(&int)
        Triangle2DF(Triangle2DF &&in) noexcept { *this = std::move(in); } // move, involk operator=(&&in)
        Triangle2DF(const FloatPoint2 &pt1In, const FloatPoint2 &pt2In, const FloatPoint2 &pt3In) { Set(pt1In,pt2In,pt3In); }
        // Construction Initializer
        Triangle2DF(std::initializer_list<FloatPoint2> il) { assert(il.size() < 4); size_t count = 0; for (auto & each : il) { pt[count] = each; ++count; } }

        virtual ~Triangle2DF() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Triangle2DF*>(p)); }
        inline Triangle2DF& operator= (const Triangle2DF& in) { Set(in); return *this; } // copy assignment
        inline Triangle2DF& operator= (Triangle2DF &&in) = default; // move assignment
        // Comparators
        bool operator==(const Triangle2DF& other) { return (pt[0] == other.pt[0]) && (pt[1] == other.pt[1]) && (pt[2] == other.pt[2]); }
        bool operator!=(const Triangle2DF& other) { return (pt[0] != other.pt[0]) || (pt[1] != other.pt[1]) || (pt[2] != other.pt[2]); }
        // Conversions
        inline explicit operator bool() const { return pt[0] != pt[1] && pt[0] != pt[2] && pt[1] != pt[2]; } // valid
        inline bool operator !() const { return pt[0] == pt[1] || pt[0] == pt[2] || pt[1] == pt[2]; } // invalid
        // Functionality
        Triangle2DF __vectorcall            Rotate(FloatPoint2 originIn, float radians) const;
        void __vectorcall                   Transform(DirectX::XMMATRIX m);

        bool __vectorcall                   Intersects(const Triangle2DF triIn) const;
        bool __vectorcall                   Intersects(const FloatPoint2 ptIn) const;
        // Accessors
        auto                                GetFaceNormal() const { return Cross(GetEdge2(), GetEdge1()); } // CCW right handed
        inline FloatPoint2                  GetEdge1() const { return pt[1] - pt[0]; }
        inline FloatPoint2                  GetEdge2() const { return pt[2] - pt[1]; }
        inline FloatPoint2                  GetEdge3() const { return pt[0] - pt[2]; }
        inline const FloatPoint2 &          GetVertex(const uint32_t vertexIn012) const { return pt[vertexIn012]; }
        // Assignments
        inline void __vectorcall            Set(const Triangle2DF &in) { Set(in.pt[0], in.pt[1], in.pt[2]); }
        inline void __vectorcall            Set(const FloatPoint2 &pt1In, const FloatPoint2 &pt2In, const FloatPoint2 &pt3In) { pt[0] = pt1In; pt[1] = pt2In; pt[2] = pt3In; }
        inline void __vectorcall            SetVertex(const uint32_t vertexIn012, const FloatPoint2 &ptIn) { pt[vertexIn012] = ptIn; }
    };
    /******************************************************************************
    *   Rectangle2DF        min
    *   lt -------          |----> (+)x
    *      |     |          |
    *      |     |          V (+)y  max
    *      ------- rb
    ******************************************************************************/
    class alignas(16) Rectangle2DF
    {
        /* variables */
    public:
        FloatPoint2     lt; // min
        FloatPoint2     rb; // max
    protected:

    private:

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Rectangle2DF>    Create() { return std::make_shared<Rectangle2DF>(); }
        static std::unique_ptr<Rectangle2DF>    CreateUnique() { return std::make_unique<Rectangle2DF>(); }

        Rectangle2DF() = default;
        Rectangle2DF(const Rectangle2DF& in) = default; // copy 
        Rectangle2DF(Rectangle2DF&& in) = default; // move
        Rectangle2DF(const RECT& rIn) { Set(rIn); } // copy
        Rectangle2DF(const FloatPoint2& ptIn) : lt(0.f, 0.f), rb(ptIn) { Set(lt, rb); }
        Rectangle2DF(const FloatPoint2& ltIn, const FloatPoint2& rbIn) { Set(ltIn, rbIn); }
        Rectangle2DF(const float& w, const float& h) : lt(0.f, 0.f), rb(w, h) { Set(lt, rb); }
        Rectangle2DF(const uint16_t& w, const uint16_t& h) : lt(0.f, 0.f), rb(static_cast<float>(w), static_cast<float>(h)) { ; }
        Rectangle2DF(const uint32_t& w, const uint32_t& h) : lt(0.f, 0.f), rb(static_cast<float>(w), static_cast<float>(h)) { ; }
        Rectangle2DF(const long& w, const long& h) : lt(0.f, 0.f), rb(static_cast<float>(w), static_cast<float>(h)) { ; }
        Rectangle2DF(const int& w, const int& h) : lt(0.f, 0.f), rb(static_cast<float>(w), static_cast<float>(h)) { ; }
        Rectangle2DF(const float& x1, const float& y1, const float& x2, const float& y2) : lt(x1, y1), rb(x2, y2) { ; }
        Rectangle2DF(const Triangle2DF& triIn) { Set(triIn); } // bounding rectangle from triangle
        Rectangle2DF(const Polygon2DF& polyIn) { Set(polyIn); } // bounding rectangle from polygon
        // Construction Initializer
        Rectangle2DF(std::initializer_list<FloatPoint2> il) { assert(il.size() < 3); auto it = il.begin(); lt = *it; rb = *(++it); };
        
        virtual ~Rectangle2DF() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Rectangle2DF*>(p)); }
        inline Rectangle2DF & operator= (const Rectangle2DF &in) = default; // copy assignment
        inline Rectangle2DF & operator= (const RECT &rIn) { Set(rIn); return *this; }
        inline Rectangle2DF & operator= (const FloatPoint2 in) { lt = in; rb = in; return *this; }
        inline Rectangle2DF & operator= (Rectangle2DF &&in) = default; // move assignment
        inline Rectangle2DF operator+ (const FloatPoint2 in) const { Rectangle2DF rtn(*this); rtn.MoveBy(in); return rtn; }
        inline Rectangle2DF operator- (const FloatPoint2 in) const { Rectangle2DF rtn(*this); rtn.MoveBy(-in); return rtn; }
        inline Rectangle2DF operator* (const FloatPoint2 in) const { Rectangle2DF rtn(*this); rtn.Grow(in); return rtn; }
        inline Rectangle2DF operator/ (const FloatPoint2 in) const { Rectangle2DF rtn(*this); rtn.Grow(FloatPoint2(1.0f) / in); return rtn; }
        inline Rectangle2DF & operator-= (const FloatPoint2 deltaIn) { MoveBy(-deltaIn); return *this; }
        inline Rectangle2DF & operator+= (const FloatPoint2 deltaIn) { MoveBy(deltaIn); return *this; }
        inline Rectangle2DF & operator*= (const FloatPoint2 in) { Grow(in); return *this; }
        inline Rectangle2DF & operator/= (const FloatPoint2 in) { Grow(FloatPoint2(1.0f) / in); return *this; }
        inline Rectangle2DF & operator*= (const float &scaleIn) { rb *= scaleIn; return *this; }
        inline Rectangle2DF & operator/= (const float &scaleIn) { rb /= scaleIn; return *this; }
        // Conversions
        inline operator RECT() const { return Get_RECT(); }
        inline operator const RECT() const { return Get_RECT(); }
        inline operator const Polygon2DF();
        // Functionality
        inline void __vectorcall            MoveBy(const FloatPoint2 deltaIn) { lt += deltaIn; rb += deltaIn; }
        inline void                         MoveBy(const long &dxIn, const long &dyIn) { lt.Set(lt.GetX() + dxIn, lt.GetY() + dyIn); rb.Set(rb.GetX() + dxIn, rb.GetY() + dyIn); }
        inline void                         MoveDX(const long &dxIn) { lt.SetX(lt.GetX() + dxIn); rb.SetX(rb.GetX() + dxIn); }
        inline void                         MoveDY(const long &dyIn) { lt.SetY(lt.GetY() + dyIn); rb.SetY(rb.GetY() + dyIn); }
        inline void __vectorcall            MoveTo(const FloatPoint2 ptLTIn) { auto wh = GetSize(); lt = ptLTIn; rb = lt + wh; }
        inline bool                         MoveTo(const long &xIn, const long &yIn) { MoveTo(FloatPoint2(xIn, yIn)); }
        
        Polygon2DF __vectorcall             Transformation(DirectX::XMMATRIX m);
        Polygon2DF __vectorcall             Transformation(FloatPoint2 scaleIn, FloatPoint2 rotOrigin, DirectX::XMVECTOR rotQuat, FloatPoint2 trans);
        Polygon2DF __vectorcall             Rotate(FloatPoint2 originIn, float radians) const; // Ex: auto poly = rect.Rotate(rect.Center(), 3.14159f);
        
        inline void __vectorcall            Grow(const FloatPoint2 scale3In) { auto s = scale3In * GetSize(); SetSize(s); }

        bool                                Intersects(const Rectangle2DF &rectIn) const;
        bool                                Intersects(const RECT &rectIn) const;
        bool __vectorcall                   Intersects(const FloatPoint2 pt2In) const { return (lt.GetX() <= pt2In.GetX()) && (pt2In.GetX() <= rb.GetX()) && (lt.GetY() <= pt2In.GetY()) && (pt2In.GetY() <= rb.GetY()); };
        //bool __vectorcall                   Intersects(const FloatPoint2 pt2In, const float rectRotationInRadians) const; // rotated about center
        bool                                Intersects(const float &xIn, const float &yIn) const;
        bool                                Intersects(const Circle2DF & circleIn) const;

        bool                                CollisionWithRay(const float2& ray_originIn, const float2& ray_dirIn, float2* contact_pointOut, float2* contact_normalOut, float* t_hit_nearOut);
        bool                                CollisionWithDynamicRect(const Rectangle2DF& movingRectIn, const float2& velIn, const float fTimeStepIn, float2* contact_pointOut, float2* contact_normalOut, float* contact_timeOut);
        bool                                ResolveDynamicRectVsRect(const Rectangle2DF& movingRectIn, float2* velInOut, const float fTimeStepIn);

        inline void                         ClipTo(const Rectangle2DF& rectIn); // clip this
        Rectangle2DF                        Clip(const King::Rectangle2DF& rectIn);
        Line2DF                             Clip(const Line2DF &src);
        Polygon2DF                          Clip(const Triangle2DF& src);
        Polygon2DF                          Clip(const Polygon2DF& pts);

        FloatPoint2 __vectorcall            FindNearestPoint(const FloatPoint2 pt2In) const;
        FloatPoint2 __vectorcall            FindNearestPoint(const FloatPoint2 pt2In, const DirectX::XMMATRIX M) const; // rectangle transformed by M
        // Accessors
        inline FloatPoint2                  GetSize() const { FloatPoint2 a(rb - lt); a.Absolute(); return a; } // width & height
        inline FloatPoint2                  GetCenter() const { return FloatPoint2(lt + (rb - lt) / 2.f); }
        inline float                        GetWidth() const { return std::abs(rb.GetX() - lt.GetX()); }
        inline float                        GetHeight() const { return std::abs(rb.GetY() - lt.GetY()); }
        inline float                        GetLeft() const { return lt.GetX(); }
        inline float                        GetRight() const { return rb.GetX(); }
        inline float                        GetTop() const { return lt.GetY(); }
        inline float                        GetBottom() const { return rb.GetY(); }
        inline const FloatPoint2 &          GetLT() const { return lt; }
        inline const FloatPoint2 &          GetRB() const { return rb; }
        inline const auto                   GetLB() const { return FloatPoint2(lt.GetX(), rb.GetY()); }
        inline const auto                   GetRT() const { return FloatPoint2(rb.GetX(), lt.GetY()); }
        inline RECT                         Get_RECT() const { RECT r; r.left = (long)GetLeft(); r.top = (long)GetTop(); r.right = (long)GetRight(); r.bottom = (long)GetBottom(); return r; }
        Polygon2DF                          Get_Polygon2DF() const;
        inline auto                         Get_Triangle2DF_LB_CCW() const { return Triangle2DF(GetLT(), GetLB(), GetRB()); } // CCW winding
        inline auto                         Get_Triangle2DF_RT_CCW() const { return Triangle2DF(GetRB(), GetRT(), GetLT()); } // CCW winding
        inline auto                         Get_Triangle2DF_LT_CW() const { return Triangle2DF(GetLB(), GetLT(), GetRT()); } // CW winding
        inline auto                         Get_Triangle2DF_RB_CW() const { return Triangle2DF(GetRT(), GetRB(), GetLB()); } // CW winding
        // Assignments
        inline void __vectorcall            Set(const Rectangle2DF rIn) { *this = rIn; }
        inline void __vectorcall            Set(const Triangle2DF triIn) { Set(Min(Min(triIn.GetVertex(0), triIn.GetVertex(1)), triIn.GetVertex(2)), Max(Max(triIn.GetVertex(0), triIn.GetVertex(1)), triIn.GetVertex(2))); }
        void                                Set(const Polygon2DF& polyIn);
        inline void                         Set(const RECT &rIn) { lt = FloatPoint2(rIn.left, rIn.top); rb = FloatPoint2(rIn.right, rIn.bottom); }
        inline void __vectorcall            Set(const FloatPoint2 ltIn, const FloatPoint2& rbIn) { lt = Min(ltIn, rbIn); rb = Max(ltIn, rbIn); }
        inline void __vectorcall            SetLT(const FloatPoint2 ltIn) { Set(ltIn, rb); }
        inline void __vectorcall            SetRB(const FloatPoint2 rbIn) { Set(lt, rbIn); }
        inline void __vectorcall            SetCenter(const FloatPoint2 cIn) { FloatPoint2 offset = GetSize() * 0.5f; FloatPoint2 c = GetCenter(); lt = c - offset; rb = c + offset; }
        inline void __vectorcall            SetSize(const FloatPoint2 whIn) { FloatPoint2 offset = Abs(whIn) * 0.5f; FloatPoint2 c = GetCenter(); lt = c - offset; rb = c + offset; }
        inline void                         SetWidth(const float &w) { rb.SetX(lt.GetX() + w); }
        inline void                         SetHeight(const float &h) { rb.SetY(lt.GetY() + h); }
        inline void                         SetLeft(const float &x) { lt.SetX(x); }
        inline void                         SetTop(const float &y) { lt.SetY(y); }
        inline void                         SetRight(const float &x) { rb.SetX(x); }
        inline void                         SetBottom(const float &y) { rb.SetY(y); }
    };
    /******************************************************************************
    *   Rectangle2D
    *   lt -------
    *      |     |
    *      |     |
    *      ------- rb
    ******************************************************************************/
    class alignas(16) Rectangle2D
    {
        /* variables */
    public:
        IntPoint2       lt;
        IntPoint2       rb;
    protected:

    private:

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Rectangle2D> Create() { return std::make_shared<Rectangle2D>(); }
        static std::unique_ptr<Rectangle2D> CreateUnique() { return std::make_unique<Rectangle2D>(); }

        Rectangle2D() = default;
        Rectangle2D(const Rectangle2D &in) { *this = in; } // copy, involk operator=(&int)
        Rectangle2D(Rectangle2D &&in) noexcept { *this = std::move(in); } // move, involk operator=(&&in)
        Rectangle2D(const RECT &rIn) { Set(rIn); } // copy
        Rectangle2D(const IntPoint2 &ptIn) : lt(0l, 0l), rb(ptIn) { ; }
        Rectangle2D(const IntPoint2 &ltIn, const IntPoint2 &rbIn) : lt(ltIn), rb(rbIn) { ; }
        Rectangle2D(const IntPoint2 &ltIn, const UIntPoint2 &whIn) : lt(ltIn), rb(ltIn + whIn) { ; }
        Rectangle2D(const long &x, const long &y) : lt(0l, 0l), rb(x, y) { ; }
        Rectangle2D(const int &x, const int &y) : lt(0l, 0l), rb(static_cast<long>(x), static_cast<long>(y)) { ; }
        Rectangle2D(const unsigned int &x, const unsigned int &y) : lt(0l, 0l), rb(static_cast<long>(x), static_cast<long>(y)) { ; }
        Rectangle2D(const unsigned long &x, const unsigned long &y) : lt(0l, 0l), rb(static_cast<long>(x), static_cast<long>(y)) { ; }
        Rectangle2D(const long &x1, const long &y1, const long &x2, const long &y2) : lt(x1, y1), rb(x2, y2) { ; }

        virtual ~Rectangle2D() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Rectangle2D*>(p)); }
        inline Rectangle2D & operator= (const Rectangle2D &in) { Set(in); return *this; } // copy assignment
        inline Rectangle2D & operator= (const RECT &rIn) { Set(rIn); return *this; }
        inline Rectangle2D & operator= (const IntPoint2 &in) { lt = in; rb = in; return *this; }
        inline Rectangle2D & operator= (Rectangle2D &&in) = default; // move assignment
        inline Rectangle2D operator- () const { Rectangle2D rtn(*this); rtn.SetLT(-rtn.GetLT()); rtn.SetRB(-rtn.GetRB()); return rtn; }
        inline Rectangle2D operator+ (const IntPoint2 in) const { Rectangle2D rtn(*this); rtn.MoveBy(in); return rtn; }
        inline Rectangle2D operator- (const IntPoint2 in) const { Rectangle2D rtn(*this); rtn.MoveBy(-in); return rtn; }
        inline Rectangle2D operator* (const IntPoint2 in) const { Rectangle2D rtn(*this); rtn.Grow(FloatPoint2(in)); return rtn; }
        inline Rectangle2D operator/ (const IntPoint2 in) const { Rectangle2D rtn(*this); rtn.Grow(FloatPoint2(1.0f) / FloatPoint2(in)); return rtn; }
        inline Rectangle2D & operator*= (const float scaleIn) { rb *= scaleIn; return *this; }
        inline Rectangle2D & operator/= (const float scaleIn) { rb /= scaleIn; return *this; }

        inline Rectangle2D & operator-= (const IntPoint2 deltaIn) { MoveBy(-deltaIn); return *this; }
        inline Rectangle2D & operator+= (const IntPoint2 deltaIn) { MoveBy(deltaIn); return *this; }
        inline Rectangle2D & operator*= (const IntPoint2 in) { Grow(FloatPoint2(in)); return *this; }
        inline Rectangle2D & operator/= (const IntPoint2 in) { Grow(FloatPoint2(1.0f) / FloatPoint2(in)); return *this; }

        inline bool operator== (const Rectangle2D& in) { return lt == in.lt ? (rb == in.rb ? true : false) : false; }
        inline bool operator!= (const Rectangle2D &in) { return lt != in.lt ? true : (rb != in.rb ? true : false); }
        inline bool operator< (const Rectangle2D& in) { auto l = GetSize(); auto r = in.GetSize(); return l < r ? true : false; }
        inline bool operator> (const Rectangle2D& in) { auto l = GetSize(); auto r = in.GetSize(); return l > r ? true : false; }
        inline bool operator<= (const Rectangle2D& in) { auto l = GetSize(); auto r = in.GetSize(); return l <= r ? true : false; }
        inline bool operator>= (const Rectangle2D& in) { auto l = GetSize(); auto r = in.GetSize(); return l >= r ? true : false; }

        // Conversions
        inline operator RECT() const { return Get_RECT(); }
        inline operator Rectangle2DF() const { return Get_Rectangle2DF(); }
        // Functionality
        inline void                         MoveBy(const IntPoint2 deltaIn) { lt += deltaIn; rb += deltaIn; }
        inline void                         MoveBy(const long &dxIn, const long &dyIn) { lt.Set(lt.GetX() + dxIn, lt.GetY() + dyIn); rb.Set(rb.GetX() + dxIn, rb.GetY() + dyIn); }
        inline void                         MoveDX(const long &dxIn) { lt.SetX(lt.GetX() + dxIn); rb.SetX(rb.GetX() + dxIn); }
        inline void                         MoveDY(const long &dyIn) { lt.SetY(lt.GetY() + dyIn); rb.SetY(rb.GetY() + dyIn); }
        inline void                         MoveTo(const IntPoint2 &pt2In) { auto wh = GetSize(); lt = pt2In; rb = lt + wh; }
        inline bool                         MoveTo(const long &xIn, const long &yIn) { MoveTo(IntPoint2(xIn, yIn)); }

        inline void __vectorcall            Grow(const FloatPoint2 scale3In) { auto s = scale3In * GetSize().Get_XMFLOAT2(); SetWH(s); }

        bool                                Intersects(const Rectangle2D &rectIn) const { return (rectIn.lt.GetX() < rb.GetX()) && (lt.GetX() < rectIn.rb.GetX()) && (rectIn.lt.GetY() < rb.GetY()) && (lt.GetY() < rectIn.rb.GetY()); }
        inline bool                         Contains(const IntPoint2 pt2In) const { return (lt.GetX() <= pt2In.GetX()) && (pt2In.GetX() < rb.GetX()) && (lt.GetY() <= pt2In.GetY()) && (pt2In.GetY() < rb.GetY()); }
        
        inline void                         ClipTo(const Rectangle2D& rectIn) { SetLT(Max(GetLT(), rectIn.GetLT())); SetRB(Min(GetRB(), rectIn.GetRB()));}

        inline IntPoint2                    FindNearestPoint(const IntPoint2& pt2In) const;
        // Accessors
        inline UIntPoint2                   GetSize() const { IntPoint2 a(rb - lt); a.MakeAbsolute(); return UIntPoint2(a); } // width & height
        inline IntPoint2                    GetCenter() const { return IntPoint2(lt + (rb - lt) / 2l); }
        inline long                         GetWidth() const { return std::abs(rb.GetX() - lt.GetX()); }
        inline long                         GetHeight() const { return std::abs(rb.GetY() - lt.GetY()); }
        inline long                         GetLeft() const { return lt.GetX(); }
        inline long                         GetRight() const { return rb.GetX(); }
        inline long                         GetTop() const { return lt.GetY(); }
        inline long                         GetBottom() const { return rb.GetY(); }
        inline const IntPoint2 &            GetLT() const { return lt; }
        inline const IntPoint2 &            GetRB() const { return rb; }
        inline IntPoint2                    GetLB() const { return IntPoint2(lt.GetX(), rb.GetY()); }
        inline IntPoint2                    GetRT() const { return IntPoint2(rb.GetX(), lt.GetY()); }
        inline RECT                         Get_RECT() const { RECT r; r.left = GetLeft(); r.top = GetTop(); r.right = GetRight(); r.bottom = GetBottom(); return r; }
        inline Rectangle2DF                 Get_Rectangle2DF() const { return Rectangle2DF(Get_RECT()); }
        // Assignments
        inline void                         Set(const Rectangle2D &rIn) { lt = rIn.lt; rb = rIn.rb; }
        inline void                         Set(const RECT &rIn) { lt = IntPoint2(rIn.left, rIn.top); rb = IntPoint2(rIn.right, rIn.bottom); }
        inline void                         Set(const IntPoint2 &ltIn, const IntPoint2 &rbIn) { lt = ltIn; rb = rbIn; }
        inline void                         SetLT(const IntPoint2 &ltIn) { lt = ltIn; }
        inline void                         SetRB(const IntPoint2 &rbIn) { rb = rbIn; }
        inline void                         SetSize(const UIntPoint2 &sz) { rb = lt + sz; }
        inline void                         SetWH(const UIntPoint2 &whIn) { rb = lt + whIn; }
        inline void                         SetWidth(const long &w) { rb.SetX(lt.GetX() + w); }
        inline void                         SetHeight(const long &h) { rb.SetY(lt.GetY() + h); }
        inline void                         SetLeft(const long &x) { lt.SetX(x); }
        inline void                         SetTop(const long &y) { lt.SetY(y); }
        inline void                         SetRight(const long &x) { rb.SetX(x); }
        inline void                         SetBottom(const long &y) { rb.SetY(y); }
    };
    /******************************************************************************
    *   Circle2DF
    *   pt1 ------- pt2
    ******************************************************************************/
    class alignas(16) Circle2DF
    {
        /* variables */
    public:
        FloatPoint3 _centerXYradiusZ;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Circle2DF> Create() { return std::make_shared<Circle2DF>(); }
        static std::unique_ptr<Circle2DF> CreateUnique() { return std::make_unique<Circle2DF>(); }
        // Construction/Destruction
        Circle2DF() = default;
        Circle2DF(const Circle2DF &in) { *this = in; } // copy, involk operator=(&int)
        Circle2DF(Circle2DF &&in) noexcept { *this = std::move(in); } // move, involk operator=(&&in)
        Circle2DF(const FloatPoint2 &centerIn, const float &radiusIn) { _centerXYradiusZ = DirectX::XMVectorSet(centerIn.GetX(), centerIn.GetY(), radiusIn, 0.0f); }
        Circle2DF(const Polygon2DF& polyIn) { Set(polyIn); } // bounding circle from polygon
        Circle2DF(std::initializer_list<float> il) { assert(il.size() < 4); auto it = il.begin(); _centerXYradiusZ = DirectX::XMVectorSet(*it, *(++it), *(++it), 0.0f); };

        virtual ~Circle2DF() = default;

        // Operators 
        void * operator new (std::size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Circle2DF*>(p)); }
        inline Circle2DF & operator= (const Circle2DF &in) { Set(in); } // copy assignment
        inline Circle2DF & operator= (Circle2DF &&in) = default; // move assignment
        // Functionality
        void __vectorcall                   Transform(DirectX::XMMATRIX m);

        bool __vectorcall                   Intersects(const FloatPoint2 pt2In) const;
        bool                                Intersects(const float &xIn, const float &yIn) const;
        bool __vectorcall                   Intersects(const Line2DF lIn) const;
        bool                                Intersects(const Rectangle2DF& rectIn) const;
        
        inline FloatPoint2                  FindNearestPoint(const FloatPoint2 &pt2In) const;
        // Accessors
        const FloatPoint2                   GetCenter() const { return FloatPoint2(_centerXYradiusZ.GetVecConst()); }
        const float                         GetRadius() const { return _centerXYradiusZ.GetZ(); }
        const FloatPoint3 &                 GetXYradiusZ() const { return _centerXYradiusZ; }
        // Assignments
        inline void __vectorcall            Set(const Circle2DF in) { _centerXYradiusZ = in._centerXYradiusZ; }
        void                                Set(const Polygon2DF& polyIn);
        inline void __vectorcall            SetCenter(const FloatPoint2 c) { _centerXYradiusZ = float3(c,GetRadius()); }
        inline void __vectorcall            SetCenter(const FloatPoint3 c) { _centerXYradiusZ.SetX(c.GetX()); _centerXYradiusZ.SetY(c.GetY()); }
        inline void __vectorcall            SetRadius(const float r) { _centerXYradiusZ.SetZ(r); }
        inline void __vectorcall            Set_centerXYradiusZ(const FloatPoint3 in) { _centerXYradiusZ = in; }
    };
    /******************************************************************************
    *   Polygon2DF
    *   pt1 ------- pt2
    *       \       \
    *    pt4 -------- pt3       
    ******************************************************************************/
    class alignas(16) Polygon2DF
    {
        /* variables */
    public:
        std::vector<FloatPoint2> _pt;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Polygon2DF> Create() { return std::make_shared<Polygon2DF>(); }
        static std::unique_ptr<Polygon2DF> CreateUnique() { return std::make_unique<Polygon2DF>(); }
        // Construction/Destruction
        Polygon2DF() = default;
        Polygon2DF(const Polygon2DF& in) { *this = in; } // copy, involk operator=(&int)
        Polygon2DF(Polygon2DF&& in) noexcept { *this = std::move(in); } // move, involk operator=(&&in)
        explicit Polygon2DF(std::initializer_list<float2> il) { for (const auto& ea : il) { _pt.clear(); _pt.reserve(il.size()); _pt.push_back(ea); } }
        Polygon2DF(const std::vector<FloatPoint2>& pts) { _pt.clear(); _pt.reserve(pts.size()); for (const auto& ea : pts) _pt.push_back(ea); }
        Polygon2DF(std::vector<FloatPoint2>&& pts) noexcept { std::swap(pts, _pt); }

        virtual ~Polygon2DF() = default;

        // Operators 
        void* operator new (std::size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void* p) { _aligned_free(static_cast<Polygon2DF*>(p)); }
        inline Polygon2DF& operator= (const Polygon2DF& in) { Set(in); } // copy assignment
        inline Polygon2DF& operator= (Polygon2DF&& in) = default; // move assignment
        // Functionality
        void __vectorcall                   Transform(DirectX::XMMATRIX m);

        // Rectangle2DF and Triangle2DF convert to Polygon2DF
        bool __vectorcall                   Intersects(const Polygon2DF polyIn) const;
        bool __vectorcall                   Intersects(const FloatPoint2 ptIn) const;
        bool __vectorcall                   Intersects(const Line2DF other) const;
        bool __vectorcall                   Intersects(const Circle2DF other) const;

        void                                Offset(const float dist);

        inline void __vectorcall            Add_pt(const FloatPoint2 val) { _pt.push_back(val); }
        inline void __vectorcall            Remove_pt(const FloatPoint2 val) { auto it = std::find(std::begin(_pt), std::end(_pt), val); if (it != std::end(_pt)) _pt.erase(it); }
        inline void                         Remove_pt(const size_t index) { assert(index < _pt.size()); _pt.erase(std::next(_pt.begin(), index)); }
        // Accessors
         // Assignments
        inline void __vectorcall            Set(const Polygon2DF in) { _pt = in._pt; }
        inline void __vectorcall            Set_pt(const size_t index, const FloatPoint3 def) { assert(index < _pt.size()); _pt[index] = def; }
    };
    /******************************************************************************
    *   ImageBlock
    *   0,0 -------
    *       |     |
    *       ------- (w,h)
    ******************************************************************************/
    class alignas(16) ImageBlock : public MemoryBlock<uint8_t>
    {
        /* variables */
    protected:
        // The row pitch, or width
        uint32_t _w = 0;
        // The depth pitch, or height
        uint32_t _h = 0;

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<ImageBlock> Create() { return std::make_shared<ImageBlock>(); }
        static std::shared_ptr<ImageBlock> Create(const uint32_t& w, const uint32_t& h, const uint32_t& texelSizeInBytes = 4) { return std::make_shared<ImageBlock>(w, h, texelSizeInBytes); }
        static std::unique_ptr<ImageBlock> CreateUnique() { return std::make_unique<ImageBlock>(); }
        static std::unique_ptr<ImageBlock> CreateUnique(const uint32_t& w, const uint32_t& h, const uint32_t texelSizeInBytes) { return std::make_unique<ImageBlock>(w, h, texelSizeInBytes); }
        // Construction/Destruction
        ImageBlock() = default;
        ImageBlock(const uint32_t& w, const uint32_t& h, const uint32_t texelSizeInBytes = 4) { Initialize(w, h, texelSizeInBytes); }
        ImageBlock(const ImageBlock& in) { *this = in; } // copy, involk operator=(&int)
        ImageBlock(ImageBlock&& in) noexcept { *this = std::move(in); } // move, involk operator=(&&in)

        virtual ~ImageBlock() = default;

        // Operators 
        void* operator new (std::size_t size) noexcept(false) { auto ptr = _aligned_malloc(size, 16); if (!ptr) throw std::bad_alloc(); return ptr; }
        void  operator delete (void* p) noexcept { _aligned_free(static_cast<ImageBlock*>(p)); }
        inline ImageBlock& operator= (const ImageBlock& in) { Set(in); } // copy assignment
        inline ImageBlock& operator= (ImageBlock&& in) = default; // move assignment
        // Functionality
        void                            Initialize(const uint32_t& w, const uint32_t& h, const uint32_t& texelInBytes = 4);
        bool                            Read(std::ifstream& dataFileIn); // custom binary file, very simple
        bool                            Write(std::ofstream& outfileIn);

        void __vectorcall               Draw(const float2 lineIn, float4 colorIn);
        void                            Draw(const Line2DF& lineIn, float4 colorIn, const float fractionIn = 1.0f);
        void                            Draw(const Triangle2DF& triIn, float4 colorIn, const float fractionIn = 1.0f);
        void                            Draw(const Polygon2DF& polyIn, float4 colorIn, const float fractionIn = 1.0f);
        void                            Draw(const Rectangle2DF& rectIn, float4 colorIn, const float fractionIn = 1.0f);
        void                            Draw(const Circle2DF& cirIn, float4 colorIn, const float fractionIn = 1.0f);

        void                            DrawFilled(const Triangle2DF& triIn, float4 colorIn);
        void                            DrawFilled(const Rectangle2DF& rectIn, float4 colorIn);
        void                            DrawFilled(const Circle2DF& cirIn, float4 colorIn);
        void                            DrawFilledScanLine(const uint32_t& xFrom, const uint32_t& xTo, const uint32_t& y_HorzScanLine, unsigned char* colorBufferIn, const bool guardIn = false);

        void                            CopyRectOut(const Rectangle2DF& srcRect, ImageBlock* destOut);
        void                            CopyImageBlockIn(const ImageBlock& srcIn, const float& x, const float& y);

        void __vectorcall               FillColor(float4 colorIn);
        void                            FloodFill(unsigned int row, unsigned int col, const MemoryBlock<unsigned char>& newColor, const MemoryBlock<unsigned char>& oldColor);
        void __vectorcall               FloodFill(unsigned int row, unsigned int col, float4 newColor, float4 oldColor);

        void                            FlipVertically();

        void                            FilterGaussianBlur(const uint32_t radius, Rectangle2DF srcRect = Rectangle2DF());
        // Accessors
        const auto&                     GetWidth() const { return _w; }
        const auto&                     GetHeight() const { return _h; }
        void                            GetPixel(const uint32_t& x, const uint32_t& y, unsigned char* out);
         // Assignments
        void                            Set(const ImageBlock& in) { MemoryBlock<unsigned char>::operator=(in); _w = in._w; _h = in._h; }
        void                            SetWidth(const uint32_t wIn, uint32_t* heightOut) { _w = wIn; if (_length > 0) _h = (uint32_t)(_length / _w); *heightOut = _h; }
        void                            SetHeight(const uint32_t hIn, uint32_t* widthOut) { _h = hIn; if (_length > 0) _w = (uint32_t)(_length / _h); *widthOut = _w; }
        void                            SetPixel(const uint32_t& x, const uint32_t& y, const unsigned char* colorBufferIn, const bool guardIn = false);
        // Helpers
        MemoryBlock<unsigned char> _vectorcall ConvertColorFrom4FloatTo32Bit(const float4 colorIn);
    };
    /******************************************************************************
    *   ImageTGA
    *   The Targa format, which stands for Truevision Advanced Raster Graphics Adapter, 
    *   was designed by Truevision (now Avid Technology) in 1984 for use with its first 
    *   video software programs. The original format consisted of 2 areas:
    *       TGA File Header
    *       Image/Color Map Data
    *
    *   Header data type code values:
    *       0  -  No image data included.
    *       1  -  Uncompressed, color-mapped images.
    *       2  -  Uncompressed, RGB images.
    *       3  -  Uncompressed, black and white images.
    *       9  -  Runlength encoded color-mapped images.
    *      10  -  Runlength encoded RGB images.
    *      11  -  Compressed, black and white images.
    ******************************************************************************/
    class alignas(16) ImageTGA : public ImageBlock
    {
        /* variables */
    public:
        // TGA header
        struct TGAHeader
        {
            char  idlength = 0; // string length after the header
            char  colormaptype = 0; // 0 is none, 1 is yes
            char  datatypecode = 2; // 2 and 10 supported for reads, 2 for writes
            short int colormaporigin = 0; // (not supported yet)
            short int colormaplength = 0; // Total length in bytes
            char  colormapdepth = 0; // each color map entry is 2, 3, or 4 bytes.  
            short int x_origin = 0; // used with imagedescriptor = 40
            short int y_origin = 0; // 
            short int width = 0; // 2 bytes in low-high order
            short int height = 0; // 2 bytes in low-high order
            char  bitsperpixel = 32;
            char  imagedescriptor = 1 << 5; // Bit 4 of the image descriptor byte indicates right-to-left pixel ordering if set. Bit 5 indicates an ordering of top-to-bottom. Otherwise, pixels are stored in bottom-to-top, left-to-right order.
        } header;

        std::string IdentificationFieldString; // read after header of idlength bytes

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<ImageTGA> Create() { return std::make_shared<ImageTGA>(); }
        static std::shared_ptr<ImageTGA> Create(const uint32_t& w, const uint32_t& h, const uint32_t& texelSizeInBytes = 4) { return std::make_shared<ImageTGA>(w, h, texelSizeInBytes); }
        static std::unique_ptr<ImageTGA> CreateUnique() { return std::make_unique<ImageTGA>(); }
        static std::unique_ptr<ImageTGA> CreateUnique(const uint32_t& w, const uint32_t& h, const uint32_t texelSizeInBytes) { return std::make_unique<ImageTGA>(w, h, texelSizeInBytes); }
        // Construction/Destruction
        ImageTGA() = default;
        ImageTGA(const uint32_t& w, const uint32_t& h, const uint32_t texelSizeInBytes = 4) { Initialize(w, h, texelSizeInBytes); }
        ImageTGA(const ImageTGA& in) { *this = in; } // copy, involk operator=(&int)
        ImageTGA(ImageTGA&& in) noexcept { *this = std::move(in); } // move, involk operator=(&&in)

        virtual ~ImageTGA() = default;

        // Operators 
        void* operator new (std::size_t size) noexcept(false) { auto ptr = _aligned_malloc(size, 16); if (!ptr) throw std::bad_alloc(); return ptr; }
        void  operator delete (void* p) noexcept { _aligned_free(static_cast<ImageTGA*>(p)); }
        inline ImageTGA& operator= (const ImageTGA& in) { Set(in); } // copy assignment
        inline ImageTGA& operator= (ImageTGA&& in) = default; // move assignment
        // Functionality
        bool                            ReadTGA(std::ifstream& dataFileIn);
        bool                            WriteTGA(std::ofstream& outfileIn);
        // Accessors
        // Assignments
        void                            Set(const ImageTGA& in) { MemoryBlock<unsigned char>::operator=(in); _w = in._w; _h = in._h; }
    private:
        uint32_t                        Read4ByteBGRAasRGBA(std::ifstream& dataFileIn);
        uint32_t                        Read3ByteBGRasRGBA(std::ifstream& dataFileIn);
    };


    /******************************************************************************
    *   streams to enable std::cout and std::cin
    ******************************************************************************/
    std::ostream& operator<< (std::ostream& os, const King::Line2DF& in);
    std::ostream& operator<< (std::ostream& os, const King::Triangle2DF& in);
    std::ostream& operator<< (std::ostream& os, const King::Rectangle2DF& in);
    std::ostream& operator<< (std::ostream& os, const King::Rectangle2D& in);
    std::ostream& operator<< (std::ostream& os, const King::Circle2DF& in);
    std::ostream& operator<< (std::ostream& os, const King::Polygon2DF& in);

    std::istream& operator>> (std::istream& is, King::Line2DF& out);
    std::istream& operator>> (std::istream& is, King::Triangle2DF& out);
    std::istream& operator>> (std::istream& is, King::Rectangle2DF& out);
    std::istream& operator>> (std::istream& is, King::Rectangle2D& out);
    std::istream& operator>> (std::istream& is, King::Circle2DF& out);
    /******************************************************************************
    *   json
    ******************************************************************************/
    void to_json(json & j, const Line2DF & from);
    void to_json(json & j, const Triangle2DF & from);
    void to_json(json & j, const Rectangle2DF & from);
    void to_json(json & j, const Rectangle2D & from);
    void to_json(json& j, const Circle2DF& from);
    
    void from_json(const json & j, Line2DF & to);
    void from_json(const json & j, Triangle2DF & to);
    void from_json(const json & j, Rectangle2DF & to);
    void from_json(const json & j, Rectangle2D & to);
    void from_json(const json& j, Circle2DF& to);

} // King namespace

/*
Example use of Geometry class for a game object of an arbitrary rectangle(not axis aligned) in world space optimized to store the transformations to world space.
Rectangle2DF keeps "object space" data and Rectangle keeps "world space" data. When rendering, call GetPolygon() and draw lines between each point. Animate each frame
by calling MoveBy(float2 deltaIn) and RotateBy(float deltaRadiansIn).For more pre - built code to perform transforms, refer to class Pose in the Geometry3D namespace
which can replace _scale, _originRot, _position, and _rotation and properly handle compound transforms through math overloads.

class Rectangle : public Rectangle2DF
{
protected:
    float2 _scale = { 1.f, 1.f };
    float2 _originRot = { 0.f, 0.f };
    float2 _position = { 0.f, 0.f };
    float _rotation = 0;
    bool _dirty = true;
    DirectX::XMMATRIX _worldMatrix;
    Polygon2DF _worldPolygon;

public:
    Rectangle() { _worldMatrix = GetWorldMatrix(); }
    void Set(const float2 scaleIn) { _scale = scaleIn; _dirty = true; }
    void Set(const float2 originIn) { _originRot = originIn; _dirty = true; }
    void Set(const float rotationIn) { _rotation = rotationIn; _dirty = true; }
    void Set(const float2 positionIn) { _position = positionIn; _dirty = true; }

    void MoveBy(float2 deltaIn) { _position += deltaIn; _dirty = true; }
    void RotateBy(float deltaRadiansIn) { _rotation += deltaRadiansIn; _dirty = true; }

    DirectX::XMMATRIX GetWorldMatrix() { if (_dirty) _worldMatrix = XMMatrixAffineTransformation(_scale, _originRot, Quaternion(0.f, 0.f, _rotation), _position); _dirty = false; return _worldMatrix; }
    Polygon2DF __vectorcall Transformation(FloatPoint2 scaleIn, FloatPoint2 rotOrigin, DirectX::XMVECTOR rotQuat, FloatPoint2 trans); // overload base class
    Polygon2DF GetPolygon() { if (_dirty) _worldPolygon = Transformation(_scale, _originRot, Quaternion(0.f, 0.f, _rotation), _position); _dirty = false; return _worldPolygon; }
};
Polygon2DF __vectorcall King::Rectangle::Transformation(FloatPoint2 scaleIn, FloatPoint2 rotOrigin, DirectX::XMVECTOR rotQuat, FloatPoint2 trans)
{
    _worldMatrix = DirectX::XMMatrixAffineTransformation(scaleIn, rotOrigin, rotQuat, trans);
    _dirty = false;
    return Rectangle2DF::Transformation(_worldMatrix);
}
*/