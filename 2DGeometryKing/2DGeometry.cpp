#include "..\2DGeometryKing\2DGeometry.h"

using namespace King;
using namespace std;


// Output streams
std::ostream& King::operator<<(std::ostream& os, const King::Line2DF& in) { return os << "{ pt0" << in.pt[0] << " pt1" << in.pt[1] << " }"; }
std::ostream& King::operator<<(std::ostream& os, const King::Triangle2DF& in) { return os << "{ pt0" << in.pt[0] << " pt1" << in.pt[1] << " pt2" << in.pt[2] << " }"; }
std::ostream& King::operator<<(std::ostream& os, const King::Rectangle2DF& in) { return os << "{ LT" << in.lt << " RB" << in.rb << " }"; }
std::ostream& King::operator<<(std::ostream& os, const King::Rectangle2D& in) { return os << "{ LT" << in.lt << " RB" << in.rb << " }"; }
std::ostream& King::operator<<(std::ostream& os, const King::Circle2DF& in) { return os << "{ Center" << in.GetCenter() << " Radius" << in.GetRadius() << " }"; }
std::ostream& King::operator<<(std::ostream& os, const King::Polygon2DF& in) { os << "{ "; for (const auto& pt : in._pt) { os << pt << " "; } os << " }"; return os; }
// Input streams
std::istream& King::operator>>(std::istream& is, King::Line2DF& out) { return is >> out.pt[0] >> out.pt[1]; } // binary in
std::istream& King::operator>>(std::istream& is, King::Triangle2DF& out) { return is >> out.pt[0] >> out.pt[1] >> out.pt[2]; }// binary in
std::istream& King::operator>>(std::istream& is, King::Rectangle2DF& out) { return is >> out.lt >> out.rb; } // binary in
std::istream& King::operator>>(std::istream& is, King::Rectangle2D& out) { return is >> out.lt >> out.rb; } // binary in
std::istream& King::operator>>(std::istream& is, King::Circle2DF& out) 
{
    float2 c;
    float r;
    is >> c >> r; // binary in
    out.SetCenter(c);
    out.SetRadius(r);
    return is;
}
/******************************************************************************
*   json
******************************************************************************/
void King::to_json(json& j, const Line2DF & from) { j = json{ {"pt0", from.GetVertex(0)}, {"pt1", from.GetVertex(1)} }; }
void King::to_json(json& j, const Triangle2DF & from) { j = json{ {"pt0", from.GetVertex(0)}, {"pt1", from.GetVertex(1)}, {"pt2", from.GetVertex(2)} }; }
void King::to_json(json& j, const Rectangle2DF & from) { j = json{ {"lt", from.GetLT()}, {"rb", from.GetRB()} }; }
void King::to_json(json& j, const Rectangle2D & from) { j = json{ {"lt", from.GetLT()}, {"rb", from.GetRB()} }; }
void King::to_json(json& j, const Circle2DF& from) { j = json{ "centerXYRadiusZ", from.GetXYradiusZ()}; }

void King::from_json(const json& j, Line2DF & to) { j.at("pt0").get_to(to.pt[0]); j.at("pt1").get_to(to.pt[1]); }
void King::from_json(const json& j, Triangle2DF & to) { j.at("pt0").get_to(to.pt[0]); j.at("pt1").get_to(to.pt[1]); j.at("pt2").get_to(to.pt[2]); }
void King::from_json(const json& j, Rectangle2DF & to) { j.at("lt").get_to(to.lt); j.at("rb").get_to(to.rb); }
void King::from_json(const json& j, Rectangle2D & to) { j.at("lt").get_to(to.lt); j.at("rb").get_to(to.rb); }
void King::from_json(const json& j, Circle2DF& to) { j.at("centerXYRadiusZ").get_to(to._centerXYradiusZ); }

/******************************************************************************
*   Line2DF::Intersects
*       Desc:       line intersection test with another line
*       Input:      line to test against
*       Output:     FloatPoint2 containing the point of intersection
*       Returns:    true if intersection occurred, otherwise false
*       Remarks:    based on Andre LeMothe's "Tricks of the Windows Game Programming Gurus"
******************************************************************************/
// *** TO DO *** evaluate XMVector2IntersectLine vs function below
bool King::Line2DF::Intersects(const Line2DF &lineIn, FloatPoint2 *intersectPointOut)
{
    // uses parallelism SIMD math
    FloatPoint2 m1(pt[1] - pt[0]);
    FloatPoint2 m2(lineIn.pt[1] - lineIn.pt[0]);

    float denominator = m1.GetX() * m2.GetY() - m1.GetY() * m2.GetX();
    if (denominator == 0)
        return false; // Colinear

    bool denomPositive = denominator > 0;
    FloatPoint2 s3 = pt[0] - lineIn.pt[0];

    float s_numerator = m1.GetX() * s3.GetY() - m1.GetY() * s3.GetX();
    if ((s_numerator < 0) == denomPositive)
        return false; // no intersection

    float t_numerator = m2.GetX() * s3.GetY() - m2.GetY() * s3.GetX();
    if ((t_numerator < 0) == denomPositive)
        return false; // no intersection

    if (((s_numerator > denominator) == denomPositive) || ((t_numerator > denominator) == denomPositive))
        return false; // no intersection

    // Collision detected
    float t = t_numerator / denominator;

    if (intersectPointOut != nullptr)
        *intersectPointOut = m1 * t + pt[0];

    return true;
}
/******************************************************************************
*    Line2DF::FindNearestPointOnLineSegment
*        Desc:        Given a line (segment) and a point in 2-dimensional space,
*                    find the point on the line (segment) that is closest to the
*                    point.
*        Input:        a point in 2-dimensional space
*        Output:        nearest point on line to input point
*        Returns:    none
*        Remarks:    Adapted from:
|                         Book Title: Game Programming Gems II
|                         Chapter Title: Fast, Robust Intersection of 3D Line Segments
|                         Author: Graham Rhodes
|                         Revisions: 05-Apr-2001 - GSR. Original.
******************************************************************************/
King::FloatPoint2 King::Line2DF::FindNearestPointOnLineSegment(const FloatPoint2 & pointIn)
{
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    FloatPoint2 ray = pt[1] - pt[0];
    const float maxSizeNotToOverFlow = sqrtf(FLT_MAX - 1.0f);
    assert(float2::Magnitude(ray) <= maxSizeNotToOverFlow);

    // Line/Segment is degenerate (distance between them is near zero)
    float d = FloatPoint2::SumComponents(DirectX::XMVectorPow(ray, { 2.f,2.f ,2.f ,2.f }));
    if (d < epsilon_squared)
        return pt[0];

    FloatPoint2 ab = pointIn - pt[0];

    // parametric coordinate of the nearest point along the line (infinite in direction of ray)
    float parameter = FloatPoint2::SumComponents(ray * ab) / d;
    parameter = std::fmaxf(0.0f, std::fminf(1.0f, parameter)); // limit to the line segment (with range 0.0 to 1.0)

    FloatPoint2 nearestPoint = parameter * ray + pt[0];

    return nearestPoint;
}
/******************************************************************************
*   Line2DF::LineTraverse
*       Desc:       raterizes the line and traverses it point by point
*       Input:      function pointer (or lamda) to callback for each point of the line
*       Output:     none
*       Returns:    none
*       Remarks:    rasterizes the line with integers utilizing the Bresenham
*                    method.
******************************************************************************/
void King::Line2DF::LineTraverse(std::function<void(IntPoint2 ptOut)> callBack)
{
    // TO DO ********* make a 3D version for Line class, reference https://gist.github.com/yamamushi/5823518 and need an int3

    // points from float to int
    const IntPoint2 ptBegin(Round(pt[0]));
    const IntPoint2 ptEnd(Round(pt[1]));
    // point to increment through
    IntPoint2 pt = ptBegin;
    // setup the initial increments
    IntPoint2 delta(ptEnd - ptBegin);

    const int ix = (delta.GetX() < 0) ? -1 : 1;
    delta.SetX( std::abs(delta.GetX()) << 1 ); // multiply by 2

    const int iy = (delta.GetY() < 0) ? -1 : 1;
    delta.SetY( std::abs(delta.GetY()) << 1 ); // multiply by 2

    callBack(pt);

    const IntPoint2 incBoth(ix, iy);
    const IntPoint2 incX(ix, 0);
    const IntPoint2 incY(0, iy);

    if (delta.GetX() >= delta.GetY())
    {
        int error(delta.GetY() - (delta.GetX() >> 1));

        while (pt.GetX() != ptEnd.GetX())
        {
            if ((error >= 0) && (error || (ix > 0)))
            {
                error -= delta.GetX();
                pt += incBoth;
            }
            else
            {
                pt += incX;
            }
            error += delta.GetY();
            
            callBack(pt);
        }
    }
    else
    {
        int error(delta.GetX() - (delta.GetY() >> 1));

        while (pt.GetY() != ptEnd.GetY())
        {
            if ((error >= 0) && (error || (iy > 0)))
            {
                error -= delta.GetY();
                pt += incBoth;
            }
            else
            {
                pt += incY;
            }
            error += delta.GetX();

            callBack(pt);
        }
    }
}

void King::Line2DF::Offset(const float distIn)
{
    float2 dir;

    // CCW inside would rotate about Z with (+) PI/2
    // we offset outside, so rotate about Z with (-) PI/2
    quat q(0.f, 0.f, -3.141592f * 0.5f);
    float2 normalOffset = GetDirectionFrom0to1();
    normalOffset = q * normalOffset;

    auto dist = distIn * normalOffset;

    SetVertex(0, GetVertex(0) + dist);
    SetVertex(1, GetVertex(1) + dist);

    // note floating point error
}

void King::Line2DF::Offset(const float2 distIn)
{
    SetVertex(0, GetVertex(0) + distIn);
    SetVertex(1, GetVertex(1) + distIn);
}

bool King::Line2DF::Join(Line2DF& otherlineIn)
{
    float2 ptIntersect;
    const float MAXFLT = sqrtf(FLT_MAX - 2.0f); // guard against overflow but extend to infinity

    if (IsParallel(otherlineIn))
    {
        // are these segments of the same line already?
        Line2DF l0to0(GetVertex(0), otherlineIn.GetVertex(0));
        Line2DF l0to1(GetVertex(0), otherlineIn.GetVertex(1));

        if (l0to0.IsParallel(l0to1))
        {
            // yes, colinear, join the nearst end points
            auto npt0 = FindNearestPointOnLineSegment(otherlineIn.GetVertex(0));
            auto nptOther0 = otherlineIn.FindNearestPointOnLineSegment(GetVertex(0));
            // merge
            if (npt0 == GetVertex(0))
            {
                if (nptOther0 == otherlineIn.GetVertex(0))
                    SetVertex(0, otherlineIn.GetVertex(0));
                else
                    SetVertex(0, otherlineIn.GetVertex(1));
            }
            else if (npt0 == GetVertex(1))
            {
                if (nptOther0 == otherlineIn.GetVertex(0))
                    SetVertex(1, otherlineIn.GetVertex(0));
                else
                    SetVertex(1, otherlineIn.GetVertex(1));
            }
            return true;
        }
        else       
            return false;
    }
    else if (Intersects(otherlineIn, &ptIntersect))
    {
        // trim the lines 
        SetVertex(1, ptIntersect);
        otherlineIn.SetVertex(0, ptIntersect);
        return true;
    }
    else
    {
        // lines must be extended
        // find the nearst end point to our line
        /* Case 1:      Case 2:     
           L1---------  L2---------
             0       1    0       1
              L2| 0        L1|1
                | 1          |0
        */
        // find the nearest end point on both lines
        float2 npL1;
        auto npL1to0 = FindNearestPointOnLineSegment(otherlineIn.GetVertex(0));
        auto npL1to1 = FindNearestPointOnLineSegment(otherlineIn.GetVertex(1));
        // is either my end point?
        if (npL1to0 == GetVertex(0))
        {
            // yes, "outside" must extend vertex 0
            auto newPt0 = float2(MAXFLT, MAXFLT);
            auto dir = GetDirectionFrom1to0();
            dir.MakeNormalize();
            newPt0 = dir * newPt0;
            SetVertex(0, newPt0);
            // now, repeat
            npL1to0 = FindNearestPointOnLineSegment(otherlineIn.GetVertex(0));
            SetVertex(0, npL1to0);
            otherlineIn.SetVertex(0, npL1to0);
        }
        else if (npL1to0 == GetVertex(1))
        {
            // yes, "outside" must extend vertex 1
            auto newPt1 = float2(MAXFLT, MAXFLT);
            auto dir = GetDirectionFrom0to1();
            dir.MakeNormalize();
            newPt1 = dir * newPt1;
            SetVertex(1, newPt1);
            // now, repeat
            npL1to0 = FindNearestPointOnLineSegment(otherlineIn.GetVertex(0));
            SetVertex(1, npL1to0);
            otherlineIn.SetVertex(0, npL1to0);
        }
        else
        {
            // no, "inside" must trim one and extend one
            // let trim 
            // that is, segment one 0->1, 1 is trimmed and for segment two 0->1, 0 is trimmed
            // npL1to0 and npL1to1 lay on L1 between 0 and 1, which is the closest to L1 vertex 0?
            auto d00 = (npL1to0 - GetVertex(0)).GetMagnitude();
            auto d01 = (npL1to1 - GetVertex(0)).GetMagnitude();
            if (d00 < d01)
            {
                SetVertex(1, npL1to0);
                otherlineIn.SetVertex(0, npL1to0);
            }
            else
            {
                SetVertex(1, npL1to1);
                otherlineIn.SetVertex(1, npL1to1);
            }
        }

        return true;
    }

    return false;
}

bool King::Line2DF::IsParallel(const Line2DF& lineIn) const
{
    auto d = float2::DotProduct(GetDirectionFrom0to1(), lineIn.GetDirectionFrom0to1()).GetX();
    if (d < -0.999995f || d > 0.999995f)
        return true;
    else
        return false;
}

bool King::Line2DF::IsPerpendicular(const Line2DF& lineIn) const
{
    auto d = float2::DotProduct(GetDirectionFrom0to1(), lineIn.GetDirectionFrom0to1()).GetX();
    d = fabsf(d);
    if (d < 0.000005f)
        return true;
    else
        return false;
    return false;
}

bool King::Triangle2DF::Intersects(Triangle2DF & triIn)
{
    // exclusion principle first
    Rectangle2DF bb(*this);
    Rectangle2DF bbIn(triIn);
    if (!bb.Intersects(bbIn)) return false;
    // brute force
    // does not cover all true cases
    if (Contains(triIn.GetVertex(0))) return true;
    if (Contains(triIn.GetVertex(1))) return true;
    if (Contains(triIn.GetVertex(2))) return true;
    if (triIn.Contains(GetVertex(0))) return true;
    if (triIn.Contains(GetVertex(1))) return true;
    if (triIn.Contains(GetVertex(2))) return true;
    return false;
}

bool King::Triangle2DF::Contains(const FloatPoint2 & ptIn)
{
    auto p0 = ptIn - pt[0];
    auto p1 = ptIn - pt[1];
    auto p10 = pt[1] - pt[0];
    auto p20 = pt[2] - pt[0];
    auto p21 = pt[2] - pt[1];

    bool s_ab = Cross(p10, p0).GetX() > 0.f;
    if (Cross(p20, p0).GetX() > 0.f == s_ab) return false;
    if (Cross(p21, p1).GetX() > 0.f != s_ab) return false;
    return true;
}
 bool __vectorcall King::Circle2DF::Intersects(const FloatPoint2 pt2In) const 
{ 
    FloatPoint2 test(pt2In - GetCenter()); 
    test *= test;
    auto r_sq(GetRadius());
    r_sq *= r_sq;
    return test > r_sq ? false : true;
}

 bool King::Circle2DF::Intersects(const float & xIn, const float & yIn) const 
{ 
    FloatPoint2 test(xIn, yIn); 
    test -= GetCenter(); 
    test *= test;
    auto r_sq(GetRadius());
    r_sq *= r_sq;
    return test > r_sq ? false : true; 
}

 bool King::Circle2DF::Intersects(const Rectangle2DF & rectIn) const
{
    auto rectPointNearest = rectIn.FindNearestPoint(GetCenter());
    return Intersects(rectPointNearest);
}

 FloatPoint2 King::Circle2DF::FindNearestPoint(const FloatPoint2 & pt2In) const
{
    auto c = GetCenter();
    auto r = GetRadius();

    auto dist = pt2In - c;
    dist.MakeNormalize();

    return c + (dist * r);
}

 void King::Circle2DF::Set(const Polygon2DF& polyIn)
 {
     auto center = float2::Average(polyIn._pt);
     auto radius = 0.f;

     for (const auto& pt : polyIn._pt)
     {
         auto delta = pt - center;
         float dist = delta.GetMagnitude();
         radius = max(radius, dist);
     }
     SetCenter(center);
     SetRadius(radius);
 }

 void King::Rectangle2DF::Set(const Polygon2DF& polyIn)
 {
     float2 mn(FLT_MAX);
     float2 mx(FLT_MIN);

     for (const auto& pt : polyIn._pt)
     {
         mn = Min(mn, pt);
         mx = Max(mx, pt);
     }

     SetLT(mn);
     SetRB(mx);
 }

 bool King::Rectangle2DF::Intersects(const Rectangle2DF & rectIn) const { return (rectIn.lt.GetX() < rb.GetX()) && (lt.GetX() < rectIn.rb.GetX()) && (rectIn.lt.GetY() < rb.GetY()) && (lt.GetY() < rectIn.rb.GetY()); }

 bool King::Rectangle2DF::Intersects(const RECT & rectIn) const { return (rectIn.left < rb.GetX()) && (lt.GetX() < rectIn.right) && (rectIn.top < rb.GetY()) && (lt.GetY() < rectIn.bottom); }


 bool King::Rectangle2DF::Intersects(const float & xIn, const float & yIn) const { return (lt.GetX() <= xIn) && (xIn < rb.GetX()) && (lt.GetY() <= yIn) && (yIn < rb.GetY()); }

 bool King::Rectangle2DF::Intersects(const King::Circle2DF & circleIn) const
{
    auto rectPointNearest = FindNearestPoint(circleIn.GetCenter());
    return circleIn.Intersects(rectPointNearest);
}

 /******************************************************************************
*    Rectangle2DF::Clip
*        Desc:       Lines are clipped to the reactangle treating it as a
*                    bounding box.
*        Input:      a line in 2-dimensional space
*        Output:     none
*        Returns:    a line with both points at (0,0) if completely outside of 
                     the rectangle.
*        Remarks:    Based on the work of Liang-Barsky function by Daniel White @ http://www.skytopia.com/project/articles/compsci/clipping.html
******************************************************************************/
 Line2DF King::Rectangle2DF::Clip(const Line2DF &src)
 {
     // make sure our edges are set correctly
     //float left = min(lt.GetX(), rb.GetX());
     //float right = max(lt.GetX(), rb.GetX());
     //float top = max(lt.GetY(), rb.GetY()); // for our algorithim, (+)y is up
     //float bottom = min(lt.GetY(), rb.GetY());
     // faster than above noting that lt is mins and rb is maxes
     float left = lt.GetX();
     float bottom = lt.GetY();
     float right = rb.GetX();
     float top = rb.GetY();

     Line2DF clip;

     float t0 = 0.0;    
     float t1 = 1.0;
     float2 delta = src.pt[1] - src.pt[0];
     float dx = delta.GetX();
     float dy = delta.GetY();
     float p, q, r;

     auto pt0 = src.GetVertex(0).Get_XMFLOAT2();
     auto pt1 = src.GetVertex(1).Get_XMFLOAT2();

     for (int edge = 0; edge < 4; edge++) 
     {  
         if (edge == 0) { p = -dx; q = -(left - pt0.x); }
         else if (edge == 1) { p = dx; q = (right - pt0.x); }
         else if (edge == 2) { p = -dy; q = -(bottom - pt0.y); }
         else { p = dy; q = (top - pt0.y); }
         
         // parallel line outside?
         if (p == 0 && q < 0) return Line2DF();

         r = q / p;

         if (p < 0) 
         {
             // is it completely outside?
             if (r > t1) return Line2DF();
             else if (r > t0) t0 = r;
         }
         else if (p > 0) 
         {
             // is it completely outside?
             if (r < t0) return Line2DF();
             else if (r < t1) t1 = r;
         }
     }
     Line2DF clipped;
     clipped.SetVertex(0, src.pt[0] + t0 * delta);
     clipped.SetVertex(1, src.pt[0] + t1 * delta);
     return clipped;
 }

 King::Polygon2DF King::Rectangle2DF::Clip(const King::Triangle2DF& src)
 {
     Polygon2DF tri;
     tri.Add_pt(src.GetVertex(0));
     tri.Add_pt(src.GetVertex(1));
     tri.Add_pt(src.GetVertex(2));
     auto rtn = Clip(tri);
     return rtn;
 }

 King::Rectangle2DF King::Rectangle2DF::Clip(const King::Rectangle2DF& rectIn)
{
    auto rtn = *this;
    rtn.SetLT(Max(GetLT(), rectIn.GetLT()));
    rtn.SetRB(Min(GetRB(), rectIn.GetRB()));
    return rtn;
}
 // *** TO DO *** one scenario not accounted for, when the polylines re-enter on a different edge, a diagonal will be drawn
 // rather than going to the rectangle corner first to start the new edge
 Polygon2DF King::Rectangle2DF::Clip(const Polygon2DF& ptsIn)
 {
     Polygon2DF rtn;
     bool outside(false);
     float2 last;

     for (size_t i = 0; i < ptsIn._pt.size(); ++i)
     {
         float2 ea = ptsIn._pt[i];
         auto istore = i;

        if (Intersects(ea))
            rtn.Add_pt(ea);
        else
        {
            // inside the rectangle and going outside
            outside = true;
            {
                auto c = Clip(Line2DF(rtn._pt.back(), ea));
                if (rtn._pt.back() != c.pt[1])
                    rtn.Add_pt(c.pt[1]);
                else if (rtn._pt.back() != c.pt[0])
                    rtn.Add_pt(c.pt[0]);
            }
        }
        last = ea;
        if (outside)
        {
            ++i;
            while (outside && i < ptsIn._pt.size()+1)
            {
                // special case must wrap to the beginning
                if (i >= ptsIn._pt.size())
                    ea = rtn._pt.front();
                else
                    ea = ptsIn._pt[i];
                
                if (Intersects(ea))
                {
                    // outside of the rectangle and coming back in
                    {
                        outside = false;
                        auto c = Clip(Line2DF(last, ea));
                        if (ea == c.pt[0]) rtn.Add_pt(c.pt[1]);
                        else rtn.Add_pt(c.pt[0]);
                        rtn.Add_pt(ea);
                    }
                }
                else
                {
                    // last and current are outside, keep skipping
                    last = ea;
                    ++i;
                }
            }
        }
     }
     // confirm that the last and first do not duplicate in certain scenarios
     if (rtn._pt.front() == rtn._pt.back())
         rtn._pt.pop_back();

     return rtn;
 }

 void King::Rectangle2DF::ClipTo(const King::Rectangle2DF& rectIn)
{
    SetLT( Max(GetLT(), rectIn.GetLT()) );
    SetRB( Min(GetRB(), rectIn.GetRB()) );
}

 King::FloatPoint2 King::Rectangle2DF::FindNearestPoint(const King::FloatPoint2 & pt2In) const
{
    return Max(lt, Min(pt2In, rb));
}

 King::IntPoint2 King::Rectangle2D::FindNearestPoint(const King::IntPoint2& pt2In) const
{
    return Max(lt, Min(pt2In, rb));
}

/******************************************************************************
*    Polygon2DF::Contains
*        Desc:       Is the point inside the polygon? For convex and concave polygons.
*        Input:      a point in 2-dimensional space
*        Output:     none
*        Returns:    true or false
*        Remarks:    Based on the work of https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
******************************************************************************/
bool __vectorcall King::Polygon2DF::Contains(const FloatPoint2 ptIn) const
{
    bool collision(false);

    for (auto it = _pt.begin(); it != _pt.end(); ++it)
    {
        auto next = std::next(it, 1);

        if (it == _pt.end()) 
            next = _pt.begin();
    
        const DirectX::XMFLOAT2 pt = ptIn;
        const DirectX::XMFLOAT2 vc = *it;
        const DirectX::XMFLOAT2 vn = *next;

        if (((vc.y >= pt.y && vn.y < pt.y) || (vc.y < pt.y && vn.y >= pt.y)) &&
            (pt.x < (vn.x - vc.x) * (pt.y - vc.y) / (vn.y - vc.y) + vc.x)) 
        {
            collision = !collision;
        }
    }

    return collision;
}
/******************************************************************************
*    Polygon2DF:    Offset
*       Transforms the ploygon outward by a set amount
******************************************************************************/
void King::Polygon2DF::Offset(const float dist)
{
    auto temp = _pt; // copy
    for (size_t i = 1; i < _pt.size(); ++i)
    {
        Line2DF l(_pt.at(i - 1), _pt.at(i));
        l.Offset(dist);
        temp[i - 1] = l.GetVertex(0);
        temp[i] = l.GetVertex(1);
    }
    _pt.swap(temp);


}
