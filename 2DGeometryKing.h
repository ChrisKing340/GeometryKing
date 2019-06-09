/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          2DGeometryKing

Description:    CPU side 2D geometry utilizing our SIMD math library.

Usage:          Use the typedef keywords in your applications as a generic
                use of the library.  All code is inline and won't be included
                in your code if not referenced.  This code is intended to
                be complied for 64 bit operating systems. 
                
Contact:        ChrisKing340@gmail.com

Copyright (c) 2019 Christopher H. King

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
#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif

#include "MathSIMD.h"

using namespace std;
using namespace DirectX;

namespace King {
    // 2D objects
    class Line2DF; // SIMD
    class Triangle2DF; // SIMD
    class Rectangle2DF; // SIMD
    class Rectangle2D; // not accelerated, integers

    // alias
    typedef Line2DF         lineF; // float data types
    typedef Triangle2DF     triF; // float data types
    typedef Rectangle2DF    rectF; // float data types
    typedef Rectangle2D     rect; // integer data types

    /******************************************************************************
    *   Line2DF
    *   pt1 ------- pt2
    ******************************************************************************/
    __declspec(align(16)) class Line2DF
    {
        /* variables */
    private:
        FloatPoint2 pt[2];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Line2DF> Create() { return std::make_shared<Line2DF>(); }
        static std::unique_ptr<Line2DF> CreateUnique() { return std::make_unique<Line2DF>(); }
        // Construction/Destruction
        Line2DF() = default;
        Line2DF(const Line2DF &in) { *this = in; } // copy, involk operator=(&int)
        Line2DF(Line2DF &&in) { *this = std::move(in); } // move, involk operator=(&&in)
        Line2DF(const FloatPoint2 &pt1In, const FloatPoint2 &pt2In) { pt[0] = pt1In; pt[1] = pt2In; }
        explicit Line2DF(std::initializer_list<FloatPoint2> il) { assert(il.size() < 3); size_t count = 0; for (auto & each : il) { pt[count] = each; ++count; } }

        virtual ~Line2DF() = default;

        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Line2DF*>(p)); }
        inline Line2DF & operator= (const Line2DF &in) { Set(in); } // copy assignment
        inline Line2DF & operator= (Line2DF &&in) = default; // move assignment

        // Functionality
        bool                                Intersects(const Line2DF &lineIn, FloatPoint2 *intersectPointOut);
        void                                LineTraverse(std::function<void(IntPoint2 ptOut)> callBack); // rasterize the line and callback for each point along the line
        // Accessors

        // Assignments
        inline void __vectorcall            Set(const Line2DF &in) { pt[0] = in.pt[0]; pt[1] = in.pt[1]; }
        inline void __vectorcall            SetPoint(const uint32_t vertexIn01, const FloatPoint2 &in) { pt[vertexIn01] = in; }
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
    __declspec(align(16)) class Triangle2DF
    {
        /* variables */
    private:
        FloatPoint2 pt[3];

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Triangle2DF> Create() { return std::make_shared<Triangle2DF>(); }
        static std::unique_ptr<Triangle2DF> CreateUnique() { return std::make_unique<Triangle2DF>(); }
        // Construction/Destruction
        Triangle2DF() = default;
        Triangle2DF(const Triangle2DF &in) { *this = in; } // copy, involk operator=(&int)
        Triangle2DF(Triangle2DF &&in) { *this = std::move(in); } // move, involk operator=(&&in)
        Triangle2DF(const FloatPoint2 &pt1In, const FloatPoint2 &pt2In, const FloatPoint2 &pt3In) { pt[0] = pt1In; pt[1] = pt2In; pt[2] = pt3In; }
        // Construction Initializer
        Triangle2DF(std::initializer_list<FloatPoint3> il) { assert(il.size() < 4); size_t count = 0; for (auto & each : il) { pt[count] = each; ++count; } }

        virtual ~Triangle2DF() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Triangle2DF*>(p)); }
        inline Triangle2DF & operator= (const Triangle2DF &in) = default; // copy assignment
        inline Triangle2DF & operator= (Triangle2DF &&in) = default; // move assignment
        // Functionality
        bool                                Intersects(Triangle2DF &triIn);
        bool                                Contains(const FloatPoint2 &ptIn);
        // Accessors
        auto                                GetFaceNormal() const { return Cross(GetEdge2(), GetEdge1()); } // CCW right handed
        inline FloatPoint2                  GetEdge1() const { return pt[1] - pt[0]; }
        inline FloatPoint2                  GetEdge2() const { return pt[2] - pt[1]; }
        inline FloatPoint2                  GetEdge3() const { return pt[0] - pt[2]; }
        inline const FloatPoint2 &          GetVertex(const uint32_t vertexIn012) const { return pt[vertexIn012]; }
        // Assignments
        inline void __vectorcall            SetVertex(const uint32_t vertexIn012, const FloatPoint2 &vertexIn) { pt[vertexIn012] = vertexIn; }
    };
    /******************************************************************************
    *   Rectangle2D
    *   lt -------
    *      |     |
    *      |     |
    *      ------- rb
    ******************************************************************************/
    __declspec(align(16)) class Rectangle2D
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
        Rectangle2D(Rectangle2D &&in) { *this = std::move(in); } // move, involk operator=(&&in)
        Rectangle2D(const RECT &rIn) { Set(rIn); } // copy
        Rectangle2D(const IntPoint2 &ptIn) : lt(0l,0l), rb(ptIn) { ; }
        Rectangle2D(const IntPoint2 &ltIn, const IntPoint2 &rbIn) : lt(ltIn), rb(rbIn) { ; }
        Rectangle2D(const IntPoint2 &ltIn, const UIntPoint2 &whIn) : lt(ltIn), rb(ltIn+whIn) { ; }
        Rectangle2D(const long &x, const long &y) : lt(0l, 0l), rb(x, y) { ; }
        Rectangle2D(const int &x, const int &y) : lt(0l, 0l), rb(static_cast<long>(x), static_cast<long>(y)) { ; }
        Rectangle2D(const unsigned int &x, const unsigned int &y) : lt(0l, 0l), rb(static_cast<long>(x), static_cast<long>(y)) { ; }
        Rectangle2D(const unsigned long &x, const unsigned long &y) : lt(0l, 0l), rb(static_cast<long>(x), static_cast<long>(y)) { ; }
        Rectangle2D(const long &x1, const long &y1, const long &x2, const long &y2 ) : lt(x1, y1), rb(x2, y2) { ; }
        
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
        // Conversions
        inline operator RECT() const { return Get_RECT(); }
        inline operator const RECT () const { return Get_RECT(); }
        // Functionality
        inline void                         MoveBy(const IntPoint2 deltaIn) { lt += deltaIn; rb += deltaIn; }
        inline void                         MoveBy(const long &dxIn, const long &dyIn) { lt.Set( lt.GetX() + dxIn, lt.GetY() + dyIn); rb.Set(rb.GetX() + dxIn, rb.GetY() + dyIn); }
        inline void                         MoveDX(const long &dxIn) { lt.SetX(lt.GetX() + dxIn); rb.SetX(rb.GetX() + dxIn); }
        inline void                         MoveDY(const long &dyIn) { lt.SetY(lt.GetY() + dyIn); rb.SetY(rb.GetY() + dyIn); }
        inline void                         MoveTo(const IntPoint2 &pt2In) { auto wh = GetSize(); lt = pt2In; rb = lt + wh; }
        inline bool                         MoveTo(const long &xIn, const long &yIn) { MoveTo(IntPoint2(xIn, yIn)); }
        
        inline void __vectorcall            Grow(const FloatPoint2 &scale3In) { auto s = scale3In * GetSize(); SetWH(s); }
        
        inline bool                         Intersects(const Rectangle2D &rectIn) const { return (rectIn.lt.GetX() < rb.GetX()) && (lt.GetX() < rectIn.rb.GetX()) && (rectIn.lt.GetY() < rb.GetY()) && (lt.GetY() < rectIn.rb.GetY()); }
        inline bool                         Intersects(const RECT &rectIn) const { return (rectIn.left < rb.GetX()) && (lt.GetX() < rectIn.right) && (rectIn.top < rb.GetY()) && (lt.GetY() < rectIn.bottom); }
        inline bool                         Contains(const IntPoint2 pt2In) const { return (lt.GetX() <= pt2In.GetX()) && (pt2In.GetX() < rb.GetX()) && (lt.GetY() <= pt2In.GetY()) && (pt2In.GetY() < rb.GetY()); }
        inline bool                         Contains(const long &xIn, const long &yIn) const { return (lt.GetX() <= xIn) && (xIn < rb.GetX()) && (lt.GetY() <= yIn) && (yIn < rb.GetY()); }
        // Accessors
        inline IntPoint2                    GetSize() const { IntPoint2 a(rb - lt); a.MakeAbsolute(); return a; } // width & height
        inline IntPoint2                    GetCenter() const { return IntPoint2(lt + (rb - lt) / 2l); }
        inline long                         GetWidth() const { return std::abs( rb.GetX() - lt.GetX() ); }
        inline long                         GetHeight() const { return std::abs( rb.GetY() - lt.GetY() ); }
        inline long                         GetLeft() const { return lt.GetX(); }
        inline long                         GetRight() const { return rb.GetX(); }
        inline long                         GetTop() const { return lt.GetY(); }
        inline long                         GetBottom() const { return rb.GetY(); }
        inline const IntPoint2 &            GetLT() const { return lt; }
        inline const IntPoint2 &            GetRB() const { return rb; }
        inline RECT                         Get_RECT() const { RECT r; r.left = GetLeft(); r.top = GetTop(); r.right = GetRight(); r.bottom = GetBottom(); return r; }
        // Assignments
        inline void                         Set(const Rectangle2D &rIn) { lt = rIn.lt; rb = rIn.rb; }
        inline void                         Set(const RECT &rIn) { lt = IntPoint2(rIn.left, rIn.top); rb = IntPoint2(rIn.right, rIn.bottom); }
        inline void                         Set(const IntPoint2 &ltIn, const IntPoint2 &rbIn) { lt = ltIn; rb = rbIn; }
        inline void                         SetLT(const IntPoint2 &ltIn) { lt = ltIn; }
        inline void                         SetRB(const IntPoint2 &rbIn) { rb = rbIn; }
        inline void                         SetSize(const UIntPoint2 &sz) { rb = lt + sz; }
        inline void                         SetWH(const UIntPoint2 &whIn) { UIntPoint2 offset = Abs(whIn) / 2ul; FloatPoint3 c = GetCenter(); lt = c - offset; rb = c + offset; }
        inline void                         SetWidth(const long &w) { rb.SetX(lt.GetX() + w); }
        inline void                         SetHeight(const long &h) { rb.SetY(lt.GetY() + h); }
        inline void                         SetLeft(const long &x) { lt.SetX(x); }
        inline void                         SetTop(const long &y) { lt.SetY(y); }
        inline void                         SetRight(const long &x) { rb.SetX(x); }
        inline void                         SetBottom(const long &y) { rb.SetY(y); }
    };
    /******************************************************************************
    *   Rectangle2DF
    *   lt -------
    *      |     |
    *      |     |
    *      ------- rb
    ******************************************************************************/
    __declspec(align(16)) class Rectangle2DF
    {
        /* variables */
    public:
        FloatPoint2     lt;
        FloatPoint2     rb;
    protected:

    private:

        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Rectangle2DF>    Create() { return std::make_shared<Rectangle2DF>(); }
        static std::unique_ptr<Rectangle2DF>    CreateUnique() { return std::make_unique<Rectangle2DF>(); }

        Rectangle2DF() = default;
        Rectangle2DF(const Rectangle2DF &in) = default; // copy 
        Rectangle2DF(Rectangle2DF &&in) = default; // move
        inline virtual Rectangle2DF * Clone() const { return new Rectangle2DF(*this); } // Copy polymorphically
        Rectangle2DF(const RECT &rIn) { Set(rIn); } // copy
        Rectangle2DF(const FloatPoint2 &ptIn) : lt(0.f, 0.f), rb(ptIn) { ; }
        Rectangle2DF(const FloatPoint2 &ltIn, const FloatPoint2 &rbIn) : lt(ltIn), rb(rbIn) { ; }
        Rectangle2DF(const float &w, const float &h) : lt(0.f, 0.f), rb(w, h) { ; }
        Rectangle2DF(const long &w, const long &h) : lt(0.f, 0.f), rb(static_cast<float>(w), static_cast<float>(h)) { ; }
        Rectangle2DF(const int &w, const int &h) : lt(0.f, 0.f), rb(static_cast<float>(w), static_cast<float>(h)) { ; }
        Rectangle2DF(const float &x1, const float &y1, const float &x2, const float &y2) : lt(x1, y1), rb(x2, y2) { ; }
        Rectangle2DF(const Triangle2DF &triIn) { Set(triIn); }
        virtual ~Rectangle2DF() = default;
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Rectangle2DF*>(p)); }
        inline Rectangle2DF & operator= (const Rectangle2DF &in) = default; // copy assignment
        inline Rectangle2DF & operator= (const RECT &rIn) { Set(rIn); return *this; }
        inline Rectangle2DF & operator= (const FloatPoint2 &in) { lt = in; rb = in; return *this; }
        inline Rectangle2DF & operator= (Rectangle2DF &&in) = default; // move assignment
        inline Rectangle2DF operator+ (const FloatPoint2 &in) const { Rectangle2DF rtn(*this); rtn.MoveBy(in); return rtn; }
        inline Rectangle2DF operator- (const FloatPoint2 &in) const { Rectangle2DF rtn(*this); rtn.MoveBy(-in); return rtn; }
        inline Rectangle2DF operator* (const FloatPoint2 &in) const { Rectangle2DF rtn(*this); rtn.Grow(in); return rtn; }
        inline Rectangle2DF operator/ (const FloatPoint2 &in) const { Rectangle2DF rtn(*this); rtn.Grow(FloatPoint2(1.0f) / in); return rtn; }
        inline Rectangle2DF & operator-= (const FloatPoint2 &deltaIn) { MoveBy(-deltaIn); return *this; }
        inline Rectangle2DF & operator+= (const FloatPoint2 &deltaIn) { MoveBy(deltaIn); return *this; }
        inline Rectangle2DF & operator*= (const FloatPoint2 &in) { Grow(in); return *this; }
        inline Rectangle2DF & operator/= (const FloatPoint2 &in) { Grow(FloatPoint2(1.0f) / in); return *this; }
        inline Rectangle2DF & operator*= (const float &scaleIn) { rb *= scaleIn; return *this; }
        inline Rectangle2DF & operator/= (const float &scaleIn) { rb /= scaleIn; return *this; }
        // Conversions
        inline operator RECT() const { return Get_RECT(); }
        inline operator const RECT() const { return Get_RECT(); }
        // Functionality
        inline void __vectorcall            MoveBy(const FloatPoint2 &deltaIn) { lt += deltaIn; rb += deltaIn; }
        inline void                         MoveBy(const long &dxIn, const long &dyIn) { lt.Set(lt.GetX() + dxIn, lt.GetY() + dyIn); rb.Set(rb.GetX() + dxIn, rb.GetY() + dyIn); }
        inline void                         MoveDX(const long &dxIn) { lt.SetX(lt.GetX() + dxIn); rb.SetX(rb.GetX() + dxIn); }
        inline void                         MoveDY(const long &dyIn) { lt.SetY(lt.GetY() + dyIn); rb.SetY(rb.GetY() + dyIn); }
        inline void __vectorcall            MoveTo(const FloatPoint2 &ptLTIn) { auto wh = GetSize(); lt = ptLTIn; rb = lt + wh; }
        inline bool                         MoveTo(const long &xIn, const long &yIn) { MoveTo(FloatPoint2(xIn, yIn)); }

        inline void __vectorcall            Grow(const FloatPoint2 &scale3In) { auto s = scale3In * GetSize(); SetWH(s); }

        inline bool __vectorcall            Intersects(const Rectangle2DF &rectIn) const { return (rectIn.lt.GetX() < rb.GetX()) && (lt.GetX() < rectIn.rb.GetX()) && (rectIn.lt.GetY() < rb.GetY()) && (lt.GetY() < rectIn.rb.GetY()); }
        inline bool                         Intersects(const RECT &rectIn) const { return (rectIn.left < rb.GetX()) && (lt.GetX() < rectIn.right) && (rectIn.top < rb.GetY()) && (lt.GetY() < rectIn.bottom); }
        inline bool __vectorcall            Intersects(const FloatPoint2 &pt2In) const { return (lt.GetX() <= pt2In.GetX()) && (pt2In.GetX() < rb.GetX()) && (lt.GetY() <= pt2In.GetY()) && (pt2In.GetY() < rb.GetY()); }
        inline bool                         Intersects(const float &xIn, const float &yIn) const { return (lt.GetX() <= xIn) && (xIn < rb.GetX()) && (lt.GetY() <= yIn) && (yIn < rb.GetY()); }
        // Accessors
        inline FloatPoint2                  GetSize() const { FloatPoint2 a(rb - lt); a.MakeAbsolute(); return a; } // width & height
        inline FloatPoint2                  GetCenter() const { return FloatPoint2(lt + (rb - lt) / 2.f); }
        inline float                        GetWidth() const { return std::abs(rb.GetX() - lt.GetX()); }
        inline float                        GetHeight() const { return std::abs(rb.GetY() - lt.GetY()); }
        inline float                        GetLeft() const { return lt.GetX(); }
        inline float                        GetRight() const { return rb.GetX(); }
        inline float                        GetTop() const { return lt.GetY(); }
        inline float                        GetBottom() const { return rb.GetY(); }
        inline const FloatPoint2 &          GetLT() const { return lt; }
        inline const FloatPoint2 &          GetRB() const { return rb; }
        inline RECT                         Get_RECT() const { RECT r; r.left = (long)GetLeft(); r.top = (long)GetTop(); r.right = (long)GetRight(); r.bottom = (long)GetBottom(); return r; }
        // Assignments
        inline void __vectorcall            Set(const Rectangle2DF &rIn) { *this = rIn; }
        inline void __vectorcall            Set(const Triangle2DF &triIn) { Set(Min(Min(triIn.GetVertex(0), triIn.GetVertex(1)), triIn.GetVertex(2)), Max(Max(triIn.GetVertex(0), triIn.GetVertex(1)), triIn.GetVertex(2))); }
        inline void                         Set(const RECT &rIn) { lt = FloatPoint2(rIn.left, rIn.top); rb = FloatPoint2(rIn.right, rIn.bottom); }
        inline void __vectorcall            Set(const FloatPoint2 &ltIn, const FloatPoint2 &rbIn) { lt = ltIn; rb = rbIn; }
        inline void __vectorcall            SetLT(const FloatPoint2 &ltIn) { lt = ltIn; }
        inline void __vectorcall            SetRB(const FloatPoint2 &rbIn) { rb = rbIn; }
        inline void __vectorcall            SetSize(const FloatPoint2 &sz) { rb = lt + sz; }
        inline void __vectorcall            SetWH(const FloatPoint2 &whIn) { FloatPoint3 offset = Abs(whIn) * 0.5f; FloatPoint3 c = GetCenter(); lt = c - offset; rb = c + offset; }
        inline void                         SetWidth(const float &w) { rb.SetX(lt.GetX() + w); }
        inline void                         SetHeight(const float &h) { rb.SetY(lt.GetY() + h); }
        inline void                         SetLeft(const float &x) { lt.SetX(x); }
        inline void                         SetTop(const float &y) { lt.SetY(y); }
        inline void                         SetRight(const float &x) { rb.SetX(x); }
        inline void                         SetBottom(const float &y) { rb.SetY(y); }
    };

}