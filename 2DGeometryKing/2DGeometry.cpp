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
void King::Line2DF::Traverse(std::function<void(IntPoint2 ptOut)> callBack) const
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

     // we need to back reverse the top and bottom for y
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

/******************************************************************************
*    ImageBlock
******************************************************************************/

inline void King::ImageBlock::Initialize(const uint32_t& w, const uint32_t& h, const uint32_t& texelInBytes)
{
    _w = w;
    _h = h;
    SetStride(texelInBytes);
    size_t length = (size_t)w * h * texelInBytes;
    MemoryBlock<unsigned char>::Initialize(length);
}

inline bool King::ImageBlock::Read(std::ifstream& dataFileIn)
{
    if (!dataFileIn.is_open()) return false;
    if (!dataFileIn.good()) return false;

    dataFileIn.read(reinterpret_cast<char*>(&_w), sizeof(_w));
    if (dataFileIn.fail()) return false;

    auto rtn = MemoryBlock<unsigned char>::Read(dataFileIn);

    if (rtn)
        _h = GetElements() / _w;

    return rtn;
}

inline bool King::ImageBlock::Write(std::ofstream& outfileIn)
{
    if (!outfileIn.is_open()) return false;

    // write extra dimension at the beginning so we can take the image block
    // and determine its dimensions after loading
    outfileIn.write(reinterpret_cast<const char*>(&_w), sizeof(_w));

    if (outfileIn.fail()) return false;
    return MemoryBlock<unsigned char>::Write(outfileIn);
}

void King::ImageBlock::Draw(const Line2DF& lineIn, float4 colorIn, const float fractionIn)
{
    assert(fractionIn <= 1.0f && fractionIn > 0.f);

    Rectangle2DF me(_w, _h);
    auto line = me.Clip(lineIn);

    line.SetVertex(1, line.GetVertex(0) + fractionIn * (line.pt[1] - line.pt[0]));

    const auto l = line.GetLength();
    if (l.GetX() < 1.0f && l.GetY() < 1.0f)
        return;

    // color
    MemoryBlock<unsigned char> c;
    if (_stride > 0)
        c.PushBack((uint8_t)(255.f * colorIn.GetX()));
    if (_stride > 1)
        c.PushBack((uint8_t)(255.f * colorIn.GetY()));
    if (_stride > 2)
        c.PushBack((uint8_t)(255.f * colorIn.GetZ()));
    if (_stride > 3)
        c.PushBack((uint8_t)(255.f * colorIn.GetW()));

    uint8_t* src = c.GetData();
    std::function<void(IntPoint2 ptOut)> callBack = [&](IntPoint2 ptOut) { SetPixel(ptOut.GetX(), ptOut.GetY(), src, false); };

    //std::lock_guard<std::mutex> guard(_mutex);
    line.Traverse(callBack);
}

void King::ImageBlock::Draw(const Triangle2DF& triIn, float4 colorIn, const float fractionIn)
{
    Rectangle2DF me(_w, _h);
    const Polygon2DF& poly = me.Clip(triIn);
    if (poly._pt.size() < 2) return;

    std::lock_guard<std::mutex> guard(_mutex);
    for (size_t i = 0; i < poly._pt.size(); ++i)
    {
        auto n(i+1);
        const auto& ptFrom = poly._pt[i];
        if (n >= poly._pt.size())
            n = 0;
        const auto& ptTo = poly._pt[n];
        Draw(Line2DF(ptFrom, ptTo), colorIn, fractionIn);
    }
}

void King::ImageBlock::Draw(const Polygon2DF& polyIn, float4 colorIn, const float fractionIn)
{
    Rectangle2DF me(_w, _h);
    const Polygon2DF& poly = me.Clip(polyIn);
    if (poly._pt.size() < 2) return;

    std::lock_guard<std::mutex> guard(_mutex);
    for (size_t i = 0; i < poly._pt.size(); ++i)
    {
        auto n(i + 1);
        const auto& ptFrom = poly._pt[i];
        if (n >= poly._pt.size())
            n = 0;
        const auto& ptTo = poly._pt[n];
        Draw(Line2DF(ptFrom, ptTo), colorIn, fractionIn);
    }
}

void King::ImageBlock::Draw(const Rectangle2DF& rectIn, float4 colorIn, const float fractionIn)
{
    assert(fractionIn <= 1.0f && fractionIn > 0.f);
    assert(rectIn.GetLT() < rectIn.GetRB());
    std::lock_guard<std::mutex> guard(_mutex);

    Rectangle2DF me(_w, _h);
    const auto rect = me.Clip(rectIn);
    const auto x = (uint32_t)rect.GetLeft();
    const auto y = (uint32_t)rect.GetTop();
    const auto w = (uint32_t)rect.GetWidth();
    const auto h = (uint32_t)rect.GetHeight();

    // color
    MemoryBlock<unsigned char> c;
    if (_stride > 0)
        c.PushBack((uint8_t)(255.f * colorIn.GetX()));
    if (_stride > 1)
        c.PushBack((uint8_t)(255.f * colorIn.GetY()));
    if (_stride > 2)
        c.PushBack((uint8_t)(255.f * colorIn.GetZ()));
    if (_stride > 3)
        c.PushBack((uint8_t)(255.f * colorIn.GetW()));

    // horizontal lines
    auto destTopStart = (uint8_t*)&Get((size_t)y * _w);
    auto destBttmStart = (uint8_t*)&Get(((size_t)y + h) * _w);
    for (uint32_t i = x * _stride; i < (x + w + 1) * _stride * fractionIn; i += _stride)
    {
        for (uint32_t byteNumber = 0; byteNumber < c.GetLength(); ++byteNumber)
        {
            *(destTopStart + i + byteNumber) = c[byteNumber];
            *(destBttmStart + i + byteNumber) = c[byteNumber];
        }
    }
    // vertical lines
    for (uint32_t j = y; j < y + h; ++j)
    {
        auto destLeft = (uint8_t*)&Get((size_t)j * _w + x);
        auto destRight = (uint8_t*)&Get((size_t)j * _w + x + w);

        for (uint32_t byteNumber = 0; byteNumber < c.GetLength(); ++byteNumber)
        {
            *(destLeft + byteNumber) = c[byteNumber];
            *(destRight + byteNumber) = c[byteNumber];
        }
    }
}

void King::ImageBlock::Draw(const Circle2DF& cirIn, float4 colorIn, const float fractionIn)
{
    assert(fractionIn <= 1.0f && fractionIn > 0.f);

    // clipping
    Rectangle2DF clip(_w, _h);
    // completely outside?
    if (!clip.Intersects(cirIn)) return;

    // color
    MemoryBlock<unsigned char> c;
    if (_stride > 0)
        c.PushBack((uint8_t)(255.f * colorIn.GetX()));
    if (_stride > 1)
        c.PushBack((uint8_t)(255.f * colorIn.GetY()));
    if (_stride > 2)
        c.PushBack((uint8_t)(255.f * colorIn.GetZ()));
    if (_stride > 3)
        c.PushBack((uint8_t)(255.f * colorIn.GetW()));

    // using the generalized ellipse midpoint algorithm
    // reference: https://www.geeksforgeeks.org/midpoint-ellipse-drawing-algorithm/ 
    int rx, ry;
    rx = ry = (int)cirIn.GetRadius();
    int xc, yc;
    const int2& center = cirIn.GetCenter();
    xc = (int)center.GetX();
    yc = (int)center.GetY();

    // fraction
    int fractionStop = (int)((float)rx * fractionIn);
    bool stop(false);

    // algorithm
    float dx, dy, d1, d2, x, y;
    x = 0;
    y = ry;

    // Initial decision parameter of region 1
    d1 = (ry * ry) - (rx * rx * ry) +
        (0.25 * rx * rx);
    dx = 2 * ry * ry * x;
    dy = 2 * rx * rx * y;

    std::lock_guard<std::mutex> guard(_mutex);

    // For region 1
    while (dx < dy && !stop)
    {
        // 4-way symmetry
        int xp = x + xc;
        int yp = y + yc;
        int xn = -x + xc;
        int yn = -y + yc;
        if (clip.Intersects(xp, yp))
            SetPixel(xp, yp, c.GetData(), false);
        if (clip.Intersects(xn, yp))
            SetPixel(xn, yp, c.GetData(), false);
        if (clip.Intersects(xp, yn))
            SetPixel(xp, yn, c.GetData(), false);
        if (clip.Intersects(xn, yn))
            SetPixel(xn, yn, c.GetData(), false);

        // Checking and updating value of
        // decision parameter based on algorithm
        if (d1 < 0)
        {
            x++;
            dx = dx + (2 * ry * ry);
            d1 = d1 + dx + (ry * ry);
        }
        else
        {
            x++;
            y--;
            dx = dx + (2 * ry * ry);
            dy = dy - (2 * rx * rx);
            d1 = d1 + dx - dy + (ry * ry);
        }

        if (x > fractionStop)
            stop = true;
    }

    // Decision parameter of region 2
    d2 = ((ry * ry) * ((x + 0.5) * (x + 0.5))) +
        ((rx * rx) * ((y - 1) * (y - 1))) -
        (rx * rx * ry * ry);

    // Plotting points of region 2
    stop = false;
    while (y >= 0 && !stop)
    {
        // 4-way symmetry
        int xp = x + xc;
        int yp = y + yc;
        int xn = -x + xc;
        int yn = -y + yc;
        if (clip.Intersects(xp, yp))
            SetPixel(xp, yp, c.GetData(), false);
        if (clip.Intersects(xn, yp))
            SetPixel(xn, yp, c.GetData(), false);
        if (clip.Intersects(xp, yn))
            SetPixel(xp, yn, c.GetData(), false);
        if (clip.Intersects(xn, yn))
            SetPixel(xn, yn, c.GetData(), false);

        // Checking and updating parameter
        // value based on algorithm
        if (d2 > 0)
        {
            y--;
            dy = dy - (2 * rx * rx);
            d2 = d2 + (rx * rx) - dy;
        }
        else
        {
            y--;
            x++;
            dx = dx + (2 * ry * ry);
            dy = dy - (2 * rx * rx);
            d2 = d2 + dx - dy + (rx * rx);
        }

        if (x > fractionStop)
            stop = true;
    }
}

void King::ImageBlock::DrawFilled(const Triangle2DF& triIn, float4 colorIn)
{
    // Algorithm draws a triangle in two parts: 
    //      1) Top half with the middle vertex, y1, forming a horizontal line over to the edge between y0 adn y2
    //      2) Bottom half being the remainder of the triangle
    // reference: http://tfpsly.free.fr/english/index.html?url=http://tfpsly.free.fr/Docs/3dIca/3dica.htm
    Rectangle2DF me(_w, _h);
    Triangle2DF tri(triIn);

    // color
    MemoryBlock<unsigned char> c;
    if (_stride > 0)
        c.PushBack((uint8_t)(255.f * colorIn.GetX()));
    if (_stride > 1)
        c.PushBack((uint8_t)(255.f * colorIn.GetY()));
    if (_stride > 2)
        c.PushBack((uint8_t)(255.f * colorIn.GetZ()));
    if (_stride > 3)
        c.PushBack((uint8_t)(255.f * colorIn.GetW()));

    // helpers
    auto SwapPt = [](float2& a, float2& b) { const float2 temp(a); a = b; b = temp; };

    // sort verticies in the y direction such that y0 < y1 < y2
    if (tri.pt[1].GetY() < tri.pt[0].GetY())
        SwapPt(tri.pt[1], tri.pt[0]);
    if (tri.pt[2].GetY() < tri.pt[1].GetY())
        SwapPt(tri.pt[2], tri.pt[1]);
    if (tri.pt[2].GetY() < tri.pt[0].GetY())
        SwapPt(tri.pt[2], tri.pt[0]);

    // distance of each edge
    int2 delta[3];
    delta[0] = tri.pt[1] - tri.pt[0];
    delta[1] = tri.pt[2] - tri.pt[1];
    delta[2] = tri.pt[0] - tri.pt[2];
    // inverse slope of each edge (as we will be incrementing y)
    float inv_m[3];
    for (uint8_t i = 0; i < 3; ++i)
    {
        if (delta[i])
            inv_m[i] = (float)delta[i].GetX() / (float)delta[i].GetY();
        else
            inv_m[i] = 0.f;
    }
    // draw
    std::lock_guard<std::mutex> guard(_mutex);

    const auto& x0 = (uint32_t)tri.pt[0].GetX();
    const auto& x1 = (uint32_t)tri.pt[1].GetX();
    const auto& y0 = (uint32_t)tri.pt[0].GetY();
    const auto& y1 = (uint32_t)tri.pt[1].GetY();
    const auto& y2 = (uint32_t)tri.pt[2].GetY();

    //const auto x = min(x0, x1);
    // top of triangle
    for (auto y = y0; y <= y1; ++y)
        DrawFilledScanLine((uint32_t)(x0 + (y - y0) * inv_m[0]), (uint32_t)(x0 + (y - y0) * inv_m[2]), y, c.GetData(), false);

    // bottom of triangle
    for (auto y = y1; y <= y2; ++y)
        DrawFilledScanLine((uint32_t)(x1 + (y - y1) * inv_m[1]), (uint32_t)(x0 + (y - y0) * inv_m[2]), y, c.GetData(), false);
}

void King::ImageBlock::DrawFilled(const Rectangle2DF& rectIn, float4 colorIn)
{
    std::lock_guard<std::mutex> guard(_mutex);

    Rectangle2DF me(_w, _h);
    const auto rect = me.Clip(rectIn);
    const auto x = (uint32_t)rect.GetLeft();
    const auto y = (uint32_t)rect.GetTop();
    const auto w = (uint32_t)rect.GetWidth();
    const auto h = (uint32_t)rect.GetHeight();

    // color
    MemoryBlock<unsigned char> c;
    if (_stride > 0)
        c.PushBack((uint8_t)(255.f * colorIn.GetX()));
    if (_stride > 1)
        c.PushBack((uint8_t)(255.f * colorIn.GetY()));
    if (_stride > 2)
        c.PushBack((uint8_t)(255.f * colorIn.GetZ()));
    if (_stride > 3)
        c.PushBack((uint8_t)(255.f * colorIn.GetW()));

    for (uint32_t j = y; j < y + h; ++j)
    {
        auto destStart = (uint8_t *)&Get((size_t)j * _w + x);

        for (uint32_t i = 0; i < w * _stride; i += _stride)
        {
            for (uint32_t byteNumber = 0; byteNumber < c.GetLength(); ++byteNumber)
            {
                destStart[i + byteNumber] = c[byteNumber];
            }
        }
    }
}

void King::ImageBlock::DrawFilled(const Circle2DF& cirIn, float4 colorIn)
{
    // clipping
    Rectangle2DF clip(_w, _h);
    // completely outside?
    if (!clip.Intersects(cirIn)) return;

    // color
    MemoryBlock<unsigned char> c;
    if (_stride > 0)
        c.PushBack((uint8_t)(255.f * colorIn.GetX()));
    if (_stride > 1)
        c.PushBack((uint8_t)(255.f * colorIn.GetY()));
    if (_stride > 2)
        c.PushBack((uint8_t)(255.f * colorIn.GetZ()));
    if (_stride > 3)
        c.PushBack((uint8_t)(255.f * colorIn.GetW()));

    // using the generalized ellipse midpoint algorithm
    // reference: https://www.geeksforgeeks.org/midpoint-ellipse-drawing-algorithm/ 
    int rx, ry;
    rx = ry = (int)cirIn.GetRadius();
    int xc, yc;
    const int2& center = cirIn.GetCenter();
    xc = (int)center.GetX();
    yc = (int)center.GetY();

    // fraction

    // algorithm
    float dx, dy, d1, d2, x, y;
    x = 0;
    y = ry;

    // Initial decision parameter of region 1
    d1 = (ry * ry) - (rx * rx * ry) +
        (0.25 * rx * rx);
    dx = 2 * ry * ry * x;
    dy = 2 * rx * rx * y;

    std::lock_guard<std::mutex> guard(_mutex);

    // For region 1
    int old_yn(-1), old_yp(-1);
    while (dx < dy)
    {
        // 4-way symmetry
        int xp = x + xc;
        int yp = y + yc;
        int xn = -x + xc;
        int yn = -y + yc;

        if (clip.Intersects(xn, yn))
            SetPixel(xn, yn, c.GetData(), false);
        if (clip.Intersects(xp, yn))
            SetPixel(xp, yn, c.GetData(), false);

        if (clip.Intersects(xn, yp))
            SetPixel(xn, yp, c.GetData(), false);
        if (clip.Intersects(xp, yp))
            SetPixel(xp, yp, c.GetData(), false);

        // fill interior
        if (yn != old_yn)
        {
            old_yn = yn;
            for (int i = xn + 1; i < xp; ++i)
                SetPixel(i, yn, c.GetData(), false);
        }
        if (yp != old_yp)
        {
            old_yp = yp;
            for (int i = xn + 1; i < xp; ++i)
                SetPixel(i, yp, c.GetData(), false);
        }
        
        // Checking and updating value of
        // decision parameter based on algorithm
        if (d1 < 0)
        {
            x++;
            dx = dx + (2 * ry * ry);
            d1 = d1 + dx + (ry * ry);
        }
        else
        {
            x++;
            y--;
            dx = dx + (2 * ry * ry);
            dy = dy - (2 * rx * rx);
            d1 = d1 + dx - dy + (ry * ry);
        }
    }

    // Decision parameter of region 2
    d2 = ((ry * ry) * ((x + 0.5) * (x + 0.5))) +
        ((rx * rx) * ((y - 1) * (y - 1))) -
        (rx * rx * ry * ry);

    // Plotting points of region 2
    old_yn = old_yp = -1;
    while (y >= 0)
    {
        // 4-way symmetry
        int xp = x + xc;
        int yp = y + yc;
        int xn = -x + xc;
        int yn = -y + yc;
        if (clip.Intersects(xp, yp))
            SetPixel(xp, yp, c.GetData(), false);
        if (clip.Intersects(xn, yp))
            SetPixel(xn, yp, c.GetData(), false);
        if (clip.Intersects(xp, yn))
            SetPixel(xp, yn, c.GetData(), false);
        if (clip.Intersects(xn, yn))
            SetPixel(xn, yn, c.GetData(), false);

        // fill interior
        if (yn != old_yn)
        {
            old_yn = yn;
            for (int i = xn + 1; i < xp; ++i)
                SetPixel(i, yn, c.GetData(), false);
        }
        if (yp != old_yp)
        {
            old_yp = yp;
            for (int i = xn + 1; i < xp; ++i)
                SetPixel(i, yp, c.GetData(), false);
        }

        // Checking and updating parameter
        // value based on algorithm
        if (d2 > 0)
        {
            y--;
            dy = dy - (2 * rx * rx);
            d2 = d2 + (rx * rx) - dy;
        }
        else
        {
            y--;
            x++;
            dx = dx + (2 * ry * ry);
            dy = dy - (2 * rx * rx);
            d2 = d2 + dx - dy + (rx * rx);
        }
    }
}

void King::ImageBlock::DrawFilledScanLine(const uint32_t& xFrom, const uint32_t& xTo, const uint32_t& y_HorzScanLine, unsigned char* colorBufferIn, const bool guardIn)
{
    // note colorIn size must match _stride
    assert((xFrom < _w) && (xTo < _w) && (y_HorzScanLine < _h));
    unsigned char* dest = &Get((size_t)y_HorzScanLine * _w);

    uint32_t start;
    uint32_t stop;
    if (xFrom < xTo)
    {
        start = _stride * xFrom;
        stop = _stride * xTo;
    }
    else
    {
        start = _stride * xFrom;
        stop = _stride * xTo;
    }

    if (stop >= _w * _stride)
        stop = (_w - 1) * _stride;

    if (guardIn)
        std::lock_guard<std::mutex> guard(_mutex);

    for (auto x = start; x <= stop; x += _stride)
        std::copy(colorBufferIn, colorBufferIn + _stride, dest + x);
}

void King::ImageBlock::CopyRectOut(const Rectangle2DF& srcRectIn, ImageBlock* destOut)
{
    Rectangle2DF me(_w, _h);
    const auto srcRect = me.Clip(srcRectIn);
    const auto x = (uint32_t)srcRect.GetLeft();
    const auto y = (uint32_t)srcRect.GetTop();
    const auto w = (uint32_t)srcRect.GetWidth();
    const auto h = (uint32_t)srcRect.GetHeight();

    if (destOut != nullptr && w > 0 && h > 0)
    {
        destOut->Initialize(w, h, GetStride());
    }
    else
        return;

    auto dest = destOut->GetData();

    std::lock_guard<std::mutex> guard(_mutex);
    for (uint32_t j = y; j < y + h; ++j)
    {
        auto srcStart = &Get((size_t)j * _w + (size_t)x);
        auto srcEnd = srcStart + w * _stride;
        std::copy(srcStart, srcEnd, dest);

        dest += w * _stride;
    }
}

void King::ImageBlock::CopyImageBlockIn(const ImageBlock& srcIn, const float& xIn, const float& yIn)
{
    const auto xDest = (uint32_t)xIn;
    const auto yDest = (uint32_t)yIn;
    assert(xDest < _w && yDest < _h);

    auto srcStride = srcIn.GetStride();
    auto destStride = GetStride();

    Rectangle2DF me(_w, _h);
    Rectangle2DF srcRectFromBlock(srcIn.GetWidth(), srcIn.GetHeight());
    auto srcRect = me.Clip(srcRectFromBlock);
    const auto x = (uint32_t)srcRect.GetLeft();
    const auto y = (uint32_t)srcRect.GetTop();
    const auto w = (uint32_t)srcRect.GetWidth();
    const auto h = (uint32_t)srcRect.GetHeight();

    unsigned char* dest = &Get((size_t)yDest * _w + (size_t)xDest);

    if (w > 0 && h > 0)
    {
        std::lock_guard<std::mutex> guard(_mutex);
        for (uint32_t j = y; j < y + h; ++j)
        {
            auto srcStart = &srcIn.Get((size_t)j * w + (size_t)x);
            auto srcEnd = srcStart + w * srcStride;

            if (srcStride == destStride)
                std::copy(srcStart, srcEnd, dest);
            else
            {
                // copy row
                for (uint32_t i = 0; i < w; ++i)
                {
                    if (srcStride > destStride)
                    {
                        auto offset = (size_t)(j - y) * _w + (size_t)i * destStride;
                        auto src = srcStart + i * srcStride;
                        // copy bytewise and skipping source bytes per iteration
                        for (uint32_t ea = 0; ea < destStride; ++ea)
                        {
                            dest[offset + ea] = src[ea];
                        }
                    }
                    else
                    {
                        auto offset = (size_t)(j - y) * _w + (size_t)i * destStride;
                        auto src = srcStart + i * srcStride;
                        // copy bytewise
                        for (uint32_t ea = 0; ea < srcStride; ++ea)
                        {
                            dest[offset + ea] = src[ea];
                        }
                        // srcStride is small than destStride, so atleast one byte needs filled to opaque by default
                        dest[offset + srcStride] = 255;
                        if (destStride - srcStride > 1)
                            dest[offset + srcStride + 1] = 255;
                        if (destStride - srcStride > 2)
                            dest[offset + srcStride + 2] = 255;
                    }
                }
            }

            dest += _w * destStride;
        }
    }
}

void King::ImageBlock::FlipVertically()
{
    MemoryBlock<uint8_t> buffer(_w, _stride);
    auto temp = buffer.GetData();
    const auto rowBytes = buffer.GetByteSize();

    for (size_t j = 0; j < _h / 2; ++j)
    {
        // start of last row
        auto lastrow = &Get(((size_t)_h - j - 1) * _w);
        auto firstrow = &Get(j * _w);

        // buffer
        std::copy(lastrow, lastrow + rowBytes, temp);
        std::copy(firstrow, firstrow + rowBytes, lastrow);
        std::copy(temp, temp + rowBytes, firstrow);
    }
}

// Accessors

void King::ImageBlock::GetPixel(const uint32_t& x, const uint32_t& y, unsigned char* colorOut)
{
    // note colorOut size must match _stride
    assert(x < _w && y < _h);
    unsigned char* src = &Get((size_t)y * _w + (size_t)x);

    std::copy(src, src + _stride, colorOut);
}

void King::ImageBlock::SetPixel(const uint32_t& x, const uint32_t& y, unsigned char* colorIn, const bool guardIn)
{
    // note colorIn size must match _stride
    assert(x < _w && y < _h);
    unsigned char* dest = &Get((size_t)y * _w + (size_t)x);
    if (guardIn)
        std::lock_guard<std::mutex> guard(_mutex);

    std::copy(colorIn, colorIn + _stride, dest);
}

/******************************************************************************
*    ImageTGA
*       Reference: http://www.paulbourke.net/dataformats/tga/
******************************************************************************/

// Uncompressed, RGB images
// Runlength encoded, RGB images.
bool King::ImageTGA::ReadTGA(std::ifstream& dataFileIn)
{
    if (!dataFileIn.is_open()) return false;
    if (!dataFileIn.good()) return false;

    // read the header
    {
        header.idlength = Read1Byte(dataFileIn);
        header.colormaptype = Read1Byte(dataFileIn);
        header.datatypecode = Read1Byte(dataFileIn);
        header.colormaporigin = Read2ByteWord(dataFileIn);
        header.colormaplength = Read2ByteWord(dataFileIn);
        header.colormapdepth = Read1Byte(dataFileIn);
        header.x_origin = Read2ByteWord(dataFileIn);
        header.y_origin = Read2ByteWord(dataFileIn);
        header.width = Read2ByteWord(dataFileIn);
        header.height = Read2ByteWord(dataFileIn);
        header.bitsperpixel = Read1Byte(dataFileIn);
        header.imagedescriptor = Read1Byte(dataFileIn);
    }

    if (header.colormaptype != 0 && header.colormaptype != 1)
        return false;
    if (header.datatypecode != 2 && header.datatypecode != 10)
        return false;
    if ((header.width < 1) || (header.height < 1))
        return false;
    if (header.bitsperpixel != 24 &&
        header.bitsperpixel != 32)
        return false;

    ImageBlock::Initialize(header.width, header.height, 4);
    // when we right less than 4 bytes in little endian we fill with zero so 
    // we do not have to right the remainder out of 4 as zeros later on
    ImageBlock::Fill(0);

    char* dest = reinterpret_cast<char*>(GetData());

    // original format
    if (header.idlength > 0)
    {
        string buffer(header.idlength, ' ');
        dataFileIn.read(&buffer[0], header.idlength);
        IdentificationFieldString = buffer;
    }

     int skipover(0);
     skipover += header.colormaptype * header.colormaplength;
     if (skipover) dataFileIn.seekg(skipover, SEEK_CUR);

     int n = 0;
     //const int bytesToRead(header.bitsperpixel / 8);

     {
         std::lock_guard<std::mutex> guard(_mutex);

         // read each texel one at a time
         while (n < header.width * header.height)
         {
             if (header.datatypecode == 2)
             {
                 // uncompressed, RGB images
                 // easy to implement
                 auto texel = Read4ByteBGRAasRGBA(dataFileIn);
                 if (dataFileIn.fail()) return false;
                 // reverse BGRA format to our RGBA format
                 //auto dword = HelperBuildTexel32(buffer, bytesToRead);
                 auto src = reinterpret_cast<char*>(&texel);
                 std::copy(src, src + _stride, dest);
                 dest += _stride;
                 ++n;
             }
             else if (header.datatypecode == 10)
             {
                 // RLE compression, RGB images
                 auto id = Read1Byte(dataFileIn);
                 auto texel = Read4ByteBGRAasRGBA(dataFileIn);
                 if (dataFileIn.fail()) return false;

                 // number of texels produced from this packet
                 int j = id & 0x7f;
                 // first texel
                 auto src = reinterpret_cast<char*>(&texel);
                 std::copy(src, src + _stride, dest);
                 dest += _stride;
                 ++n;

                 if (id & 0x80)
                 {
                     // run length encoded packet, one color repeated
                     for (int i = 0; i < j; ++i)
                     {
                         std::copy(src, src + _stride, dest);
                         dest += _stride;
                         ++n;
                     }
                 }
                 else
                 {
                     // raw packet (each pixel unique)
                     for (int i = 0; i < j; ++i)
                     {
                         auto texel = Read4ByteBGRAasRGBA(dataFileIn);
                         if (dataFileIn.fail()) return false;

                         auto src = reinterpret_cast<char*>(&texel);
                         std::copy(src, src + _stride, dest);
                         dest += _stride;
                         ++n;
                     }
                 }
             }
         }
     }
     if (dataFileIn.fail()) return false;

     if (!(header.imagedescriptor & (1 << 5)))
         FlipVertically();

     return true;
 }
 // Uncompressed, RGB images
 bool King::ImageTGA::WriteTGA(std::ofstream& outfileIn)
 {
     if (!outfileIn.is_open()) return false;
     if (!GetLength()) return false;

     IdentificationFieldString = "File writer from 2DGeometryKing with code on GitHub.com";
     // original format
     header.idlength = IdentificationFieldString.size();
     header.colormaptype = 0;
     header.datatypecode = 2;
     assert(header.datatypecode == 2);// || header.datatypecode == 10); *** TO DO *** RLE writer
     header.colormaporigin = 0;
     header.colormaplength = 0;
     header.colormapdepth = 0;
     assert(header.x_origin >= 0 && header.x_origin <= header.width);
     assert(header.y_origin >= 0 && header.y_origin <= header.height);
     header.width = _w;
     header.height = _h;
     assert(header.bitsperpixel == 24 || header.bitsperpixel == 32);
     header.imagedescriptor = 1 << 5; // top to bottom pixel ordering

     // due to alignment, we cannot write all at once and need to do bytewise and therefore encode 2 byte words in little endian order
     //outfileIn.write(reinterpret_cast<const char*>(&header), sizeof(header));
     Write1Byte(outfileIn, header.idlength);                    
     Write1Byte(outfileIn, header.colormaptype);
     Write1Byte(outfileIn, header.datatypecode);
     Write2ByteWord(outfileIn, header.colormaporigin);
     Write2ByteWord(outfileIn, header.colormaplength);
     Write1Byte(outfileIn, header.colormapdepth);
     Write2ByteWord(outfileIn, header.x_origin);
     Write2ByteWord(outfileIn, header.y_origin);
     Write2ByteWord(outfileIn, header.width);
     Write2ByteWord(outfileIn, header.height);
     Write1Byte(outfileIn, header.bitsperpixel);
     Write1Byte(outfileIn, header.imagedescriptor);
     
     // id text
     outfileIn.write(IdentificationFieldString.c_str(), header.idlength);

     // color map
     int blankSize(0);
     blankSize += header.colormaptype * header.colormaplength;
     char zero(0);
     outfileIn.write(&zero, blankSize);

     // data
     // RGBA needs to be converted to BGRA
     const auto& bpp = header.bitsperpixel; // allow compiler to optimize the for loop below
     ImageBlock BGRA(_w, _h, bpp / 8);
     const auto bytes = GetByteSize();
     for (size_t i = 0; i < bytes; i += _stride)
     {
         if (bpp >= 24)
             *(BGRA.GetData() + i + 0) = *(GetData() + i + 2);
             *(BGRA.GetData() + i + 1) = *(GetData() + i + 1);
             *(BGRA.GetData() + i + 2) = *(GetData() + i + 0);
         if (bpp >= 32)
            *(BGRA.GetData() + i + 3) = *(GetData() + i + 3);
     }

     // bottom to top pixel ordering
     if (!(header.imagedescriptor & (1 << 5)))
         BGRA.FlipVertically();

     if (header.datatypecode == 2)
         outfileIn.write(reinterpret_cast<const char*>(BGRA.GetData()), BGRA.GetByteSize());
     // RLE *** TO DO ***
     else if (header.datatypecode == 10)
     {
         uint8_t id(0); // packet type
         uint8_t j(0); // packet length
         bool writePacket(false);
         for (size_t i = 0; i < BGRA.GetElements(); ++i)
         {
             // get next color value
             auto colorPtr = &(BGRA[i]);
             // check for repeating run

             // have run, build RLE packet
             {
                 id = 0x80;
                 ++j;
             }

             // no run, build raw packet
             {
                 uint8_t id = 0;

             }
             // check for limit of repeat or raw packet size
             if (j == 127)
             {
                // end the packet size, start a new
                 writePacket = true;
             }

             // write RLE packet
             if (writePacket)
             {
                 // write the packet id & length byte
                 id += j;
                 Write1Byte(outfileIn, id);
                 // write the color value
                 if (bpp >= 24)
                 {
                     // write as BGR
                     // source is ((a << 24) | (b << 16) | (g << 8) | r), DO WE NEED TO REVERSE THE ORDER??? test
                     assert(bpp == 32);
                     BGRA.Write1Byte(outfileIn, colorPtr[0]); // b
                     BGRA.Write1Byte(outfileIn, colorPtr[1]); // g
                     BGRA.Write1Byte(outfileIn, colorPtr[2]); // r
                 }
                 else if (bpp >= 32)
                 {
                     // write as BGRA
                     BGRA.Write1Byte(outfileIn, colorPtr[3]); // a
                 }
                 // reset the RLE length to 0
                 j = 0;
                 writePacket = false;
             }
             // write RAW packet
             if (writePacket)
             {
                 // write the packet id & length byte
                 id += j;
                 Write1Byte(outfileIn, id);
                 // write first color value, then j more color values (use pointer start to pointer end of BGRA)
                 // write the color value
                 if (bpp >= 24)
                     ; // std::copy
                 if (bpp >= 32)
                     ; // std::copy
                 // reset the RAW length to 0
                 j = 0;
                 writePacket = false;
             }
         }
     }

     if (outfileIn.fail()) return false;
     return true;
 }

 uint32_t King::ImageTGA::Read4ByteBGRAasRGBA(std::ifstream& dataFileIn)
 {
     // little endian ordering
     auto b = Read1Byte(dataFileIn);
     auto g = Read1Byte(dataFileIn);
     auto r = Read1Byte(dataFileIn);
     auto a = Read1Byte(dataFileIn);
     if (dataFileIn.fail()) return 0;
     return ((a << 24) | (b << 16) | (g << 8) | r);
 }

 uint32_t King::ImageTGA::Read3ByteBGRasRGBA(std::ifstream& dataFileIn)
 {
     // little endian ordering
     auto b = Read1Byte(dataFileIn);
     auto g = Read1Byte(dataFileIn);
     auto r = Read1Byte(dataFileIn);
     auto a = 255;
     if (dataFileIn.fail()) return 0;
     return ((a << 24) | (b << 16) | (g << 8) | r);
 }
