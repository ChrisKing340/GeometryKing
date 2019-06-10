#include "2DGeometryKing.h"

using namespace King;
using namespace std;

/******************************************************************************
*   json
******************************************************************************/
void King::to_json(json& j, const Line2DF & from) { j = json{ {"pt0", from.GetVertex(0)}, {"pt1", from.GetVertex(1)} }; }
void King::to_json(json& j, const Triangle2DF & from) { j = json{ {"pt0", from.GetVertex(0)}, {"pt1", from.GetVertex(1)}, {"pt2", from.GetVertex(2)} }; }
void King::to_json(json& j, const Rectangle2DF & from) { j = json{ {"lt", from.GetLT()}, {"rb", from.GetRB()} }; }
void King::to_json(json& j, const Rectangle2D & from) { j = json{ {"lt", from.GetLT()}, {"rb", from.GetRB()} }; }

void King::from_json(const json& j, Line2DF & to) { j.at("pt0").get_to(to.pt[0]); j.at("pt1").get_to(to.pt[1]); }
void King::from_json(const json& j, Triangle2DF & to) { j.at("pt0").get_to(to.pt[0]); j.at("pt1").get_to(to.pt[1]); j.at("pt2").get_to(to.pt[2]); }
void King::from_json(const json& j, Rectangle2DF & to) { j.at("lt").get_to(to.lt); j.at("rb").get_to(to.rb); }
void King::from_json(const json& j, Rectangle2D & to) { j.at("lt").get_to(to.lt); j.at("rb").get_to(to.rb); }

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
*	Line2DF::FindNearestPointOnLineSegment
*		Desc:		Given a line (segment) and a point in 2-dimensional space,
*					find the point on the line (segment) that is closest to the
*					point.
*		Input:		a point in 2-dimensional space
*		Output:		nearest point on line to input point
*		Returns:	none
*		Remarks:	Adapted from:
|						 Book Title: Game Programming Gems II
|						 Chapter Title: Fast, Robust Intersection of 3D Line Segments
|						 Author: Graham Rhodes
|						 Revisions: 05-Apr-2001 - GSR. Original.
******************************************************************************/
King::FloatPoint2 King::Line2DF::FindNearestPointOnLineSegment(const FloatPoint2 & pointIn)
{
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    FloatPoint2 ray = pt[1] - pt[0];

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