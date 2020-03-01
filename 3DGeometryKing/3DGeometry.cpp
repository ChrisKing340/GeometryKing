#include "..\MathSIMD\MathSIMD.h"
#include "..\2DGeometryKing\2DGeometry.h"
#include "..\3DGeometryKing\3DGeometry.h"
#include "..\3DGeometryKing\Model_IO.h"

using namespace King;
using namespace std;

// global constants
extern const FloatPoint3 g_boxCorners[8] =
{
    { -1.0f, -1.0f, -1.0f }, // lbb (min)
    { -1.0f, -1.0f,  1.0f }, // lbf
    { 1.0f, -1.0f,  1.0f }, // rbf
    { 1.0f, -1.0f, -1.0f }, // rbb
    { -1.0f,  1.0f, -1.0f }, // ltb
    { -1.0f,  1.0f,  1.0f }, // ltf
    { 1.0f,  1.0f,  1.0f }, // rtf (max)
    { 1.0f,  1.0f, -1.0f } // rtb
};

//namespace King {
    std::vector<std::string> King::VertexAttrib::description = { "Position", "Color", "TextureCoord" , "Normal" , "Tangent" , "Bi-Tangent" };
//}
/******************************************************************************
*    Method:    operator<< operator>> Box
******************************************************************************/
std::ostream & King::operator<< (std::ostream &os, const King::Box &in)
{
    return os << "{ center" << in.GetCenter() << " pt_min" << in.GetMin() << " pt_max" << in.GetMax() << " }"; // text out
}
std::istream & King::operator>> (std::istream &is, King::Box &out)
{
    return is >> out.pt_min >> out.pt_max; // binary in
}

std::ostream & King::operator<< (std::ostream &os, const King::Pose &in)
{
    return os << "{ S" << in.GetScale() << " Q" << in.GetRotation() << " T" << in.GetTranslation() << " }"; // text out
}
std::wostream & King::operator<< (std::wostream &os, const King::Pose &in)
{
    return os << L"{ S" << in.GetScale() << L" Q" << in.GetRotation() << L" T" << in.GetTranslation() << L" }"; // text out
}

std::istream & King::operator>> (std::istream &is, King::Pose &out)
{
    return is >> out._scale >> out._rotation >> out._translation; // binary in
}
std::wistream & King::operator>> (std::wistream &is, King::Pose &out)
{
    return is >> out._scale >> out._rotation >> out._translation; // binary in
}

/******************************************************************************
*    json
******************************************************************************/
void King::to_json(json& j, const Pose & from) { j = json{ {"S", from._scale}, {"Q", from._rotation}, {"T", from._translation} }; }
void King::from_json(const json& j, Pose& to) { j.at("S").get_to(to._scale); j.at("Q").get_to(to._rotation); j.at("T").get_to(to._translation); }

/******************************************************************************
*    Triangle Intersects
*        Desc:        ray intersection test
*        Input:        ray to test against (with origin and direction)
*        Output:        FloatPoint3 containing the point of intersection
*        Returns:    true if intersection occurred, otherwise false
*        Remarks:    Möller, Tomas; Trumbore, Ben (1997). "Fast, Minimum Storage Ray-Triangle Intersection". Journal of Graphics Tools. 2: 21–28.
*                    https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
*            edge3
*       pt0 ------- pt2
*           \     /
*     edge1  \   / edge2
*             \ /
*             pt1
******************************************************************************/
bool King::Triangle::Intersects(const FloatPoint3 &rayOriginIn, const FloatPoint3 &rayDirectionIn, FloatPoint3 *intersectPointOut)
{
    King::FloatPoint3 hv, sv, qv;
    float a, f, u, v;

    auto edge2 = pt[2] - pt[0];
    auto edge1 = pt[1] - pt[0];

    hv = King::FloatPoint3::CrossProduct(rayDirectionIn, edge1); // result is an orthogonal vector
    a = King::FloatPoint3::DotProduct(edge2, hv).GetX(); // project other edge onto the orthogonal vector
    if (a > -FLT_EPSILON && a < FLT_EPSILON) // if projection is zero, then these vectors are also orthogonal, so the plane is parallel to the ray
        return false;
    f = 1 / a;
    sv = rayOriginIn - pt[0]; // ray relative to triangle
    u = f * King::FloatPoint3::DotProduct(sv, hv).GetX();
    if (u < 0.0f || u > 1.0f)
        return false;
    qv = sv.CrossProduct(edge2);
    v = f * King::FloatPoint3::DotProduct(rayDirectionIn, qv).GetX();
    if (v < 0.0f || u + v > 1.0f)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * edge1.DotProduct(qv);
    if (t >= 0.f) // ray intersection
    {
        *intersectPointOut = rayOriginIn + rayDirectionIn * t;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

bool King::TriangleIndexed::Intersects(const FloatPoint3 &rayOriginIn, const FloatPoint3 &rayDirectionIn, FloatPoint3 *intersectPointOut)
{
    assert(intersectPointOut);
    King::FloatPoint3 hv, sv, qv;
    float a, f, u, v;

    auto edge2 = GetVertexPosition(2) - GetVertexPosition(0);
    auto edge1 = GetVertexPosition(1) - GetVertexPosition(0);

    hv = King::FloatPoint3::CrossProduct(rayDirectionIn, edge1); // result is an orthogonal vector
    a = King::FloatPoint3::DotProduct(edge2, hv).GetX(); // project other edge onto the orthogonal vector
    if (a > -FLT_EPSILON && a < FLT_EPSILON) // if projection is zero, then these vectors are also orthogonal, so the plane is parallel to the ray
        return false;
    f = 1 / a;
    sv = rayOriginIn - GetVertex(0); // ray relative to triangle
    u = f * King::FloatPoint3::DotProduct(sv, hv).GetX();
    if (u < 0.0f || u > 1.0f)
        return false;
    qv = sv.CrossProduct(edge2);
    v = f * King::FloatPoint3::DotProduct(rayDirectionIn, qv).GetX();
    if (v < 0.0f || u + v > 1.0f)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * edge1.DotProduct(qv);
    if (t >= 0.f) // ray intersection
    {
        *intersectPointOut = rayOriginIn + rayDirectionIn * t;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

// byte pointer to start of vertex data
inline FloatPoint3 King::LineIndexed::GetVertexPosition(const uint32_t indexIn) const
{
    assert(vb != nullptr);
    auto attrI = vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = vertexFormat.GetAttribute(attrI).GetOffset();
    return FloatPoint3(reinterpret_cast<float*>((size_t)vb + (size_t)pt[indexIn] * vertexFormat.GetByteSize() + offset));
}
inline FloatPoint3 King::TriangleIndexed::GetVertexPosition(const uint32_t indexIn) const
{
    assert(vb != nullptr);
    auto attrI = vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = vertexFormat.GetAttribute(attrI).GetOffset();
    return FloatPoint3(reinterpret_cast<float*>((size_t)vb + (size_t)pt[indexIn] * vertexFormat.GetByteSize() + offset));
}
// Determine the number of verticies by finding the larget index in the index buffer
auto King::LineMesh::GetNumVerticies()
{
    uint32_t numVerticies = 0;
    auto lambdaMax = [](uint32_t a, uint32_t b) { return ((a > b) ? a : b); };

    const auto & ib = GetIB();
    const auto & ni = GetNumIndicies();

    for (uint32_t i = 0; i < ni; ++i)
    {
        numVerticies = lambdaMax(*(ib + i), numVerticies);
    }
    return ni > 0 ? numVerticies + 1 : 0; // indexing is zero based, size is therefore +1
}
uint32_t King::TriangleMesh::GetNumVerticies() const
{
    uint32_t numVerticies = 0;
    auto lambdaMax = [](uint32_t a, uint32_t b) { return ((a > b) ? a : b); };

    const auto & ib = GetIB();
    const auto & ni = GetNumIndicies();

    for (uint32_t i = 0; i < ni; ++i)
    {
        numVerticies = lambdaMax(*(ib+i), numVerticies);
    }
    return ni > 0 ? numVerticies+1 : 0; // indexing is zero based, size is therefore +1
}
inline FloatPoint3 King::LineMesh::GetVertexPosition(const uint32_t indexIn) const
{
    assert(_vb != nullptr);
    auto attrI = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = _vertexFormat.GetAttribute(attrI).GetOffset();
    return FloatPoint3(reinterpret_cast<float*>(GetVertexAddr(indexIn) + offset));
}
inline FloatPoint3 King::TriangleMesh::GetVertexPosition(const uint32_t indexIn) const
{
    assert(_vb != nullptr);
    auto attrI = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = _vertexFormat.GetAttribute(attrI).GetOffset();
    return FloatPoint3(reinterpret_cast<float*>(GetVertexAddr(indexIn) + offset));
}

inline bool __vectorcall King::Point::Intersects(const FloatPoint3 &pointIn) const
{
    FloatPoint3 diff = pointIn - static_cast<FloatPoint3>(*this);
    diff.MakeAbsolute();

    return FloatPoint3::SumComponents(diff) < FLT_EPSILON ? true : false;
}

inline bool King::Point::Intersects(const Point & pointIn) const
{
    FloatPoint3 diff(pointIn - *this);
    diff.MakeAbsolute();

    return FloatPoint3::SumComponents(diff) < FLT_EPSILON ? true : false;
}

inline bool King::Point::Collision(Point const & pointIn) const { return Intersects(pointIn); }

inline bool King::Point::Collision(Line const & lineIn) const { return lineIn.Intersects(*this); }

inline bool King::Point::Collision(Sphere const & sphereIn) const { return sphereIn.Contains(*this); }

inline bool King::Point::Collision(Box const & boxIn) const { return boxIn.Contains(*this); }

inline bool __vectorcall King::Line::Intersects(const Point & pointIn) const
{ 
    auto nPt = FindNearestPointOnLineSegment(pointIn); 

    FloatPoint3 diff = pointIn - nPt;
    diff.MakeAbsolute();

    return FloatPoint3::SumComponents(diff) < FLT_EPSILON ? true : false;
}

/******************************************************************************
*    Line::Intersects
*        Desc:        Find the nearest point between two finite length line segments
*        Input:        a line in 3-dimensional space
*        Output:        true if within tolerance, false otherwise
*                    FloatPoint3 *intersectOut is point of intersection
*        Returns:    none
*        Remarks:    Adapted from:
|                         Book Title: Game Programming Gems II
|                         Chapter Title: Fast, Robust Intersection of 3D Line Segments
|                         Author: Graham Rhodes
|                         Revisions: 05-Apr-2001 - GSR. Original.
******************************************************************************/
inline bool __vectorcall King::Line::Intersects(const Line & lineIn, FloatPoint3 *intersectionOut) const
{
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    FloatPoint3 rayA = pt[1] - pt[0];
    FloatPoint3 rayB = lineIn.pt[1] - lineIn.pt[0];

    FloatPoint3 pointOnSegA, pointOnSegB;

    // line segment is degenerate (distance between end points is near zero)
    float L11 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayA, FloatPoint3( 2.f, 2.f ,2.f ))));
    float L22 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayB, FloatPoint3( 2.f, 2.f, 2.f ))));

    if (L11 < epsilon_squared)
    {
        pointOnSegB = lineIn.FindNearestPointOnLineSegment(pt[0]);
    }
    else if (L22 < epsilon_squared)
    {
        pointOnSegA = FindNearestPointOnLineSegment(lineIn.pt[0]);
    }
    else
    {
        FloatPoint3 AB = lineIn.pt[0] - pt[0];
        float L12 = -FloatPoint3::SumComponents(rayA * rayB);
        float DetL = L11 * L22 - L12 * L12;

        // Lines/Segments A and B are parallel
        if (fabsf(DetL) < FLT_EPSILON)
        {
            return false;
        }
        // The general case
        else
        {
            float ra = FloatPoint3::SumComponents(rayA * AB);
            float rb = -FloatPoint3::SumComponents(rayB * AB);

            float t = (L11 * rb - ra * L12) / DetL;
            float s = (ra - L12 * t) / L11;

            pointOnSegA = s * rayA + pt[0];
            pointOnSegB = t * rayB + lineIn.pt[0];
        }
    
    }
    // point of intersection is the midpoint of the two
    if(intersectionOut != nullptr)
        *intersectionOut = (pointOnSegA + pointOnSegB) * 0.5f;

    FloatPoint3 nearestVector = pointOnSegB - pointOnSegA;
    nearestVector.MakeAbsolute();

    return FloatPoint3::SumComponents(nearestVector) < FLT_EPSILON ? true : false;
}
/******************************************************************************
*    Line::FindNearestPointOnLineSegment
*        Desc:        Given a line (segment) and a point in 3-dimensional space,
*                    find the point on the line (segment) that is closest to the
*                    point.
*        Input:        a point in 3-dimensional space
*        Output:        nearest point on line to input point
*        Returns:    none
*        Remarks:    Adapted from:
|                         Book Title: Game Programming Gems II
|                         Chapter Title: Fast, Robust Intersection of 3D Line Segments
|                         Author: Graham Rhodes
|                         Revisions: 05-Apr-2001 - GSR. Original.
******************************************************************************/
King::FloatPoint3 King::Line::FindNearestPointOnLineSegment(const FloatPoint3 & pointIn) const
{
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    FloatPoint3 ray = pt[1] - pt[0];

    // Line/Segment is degenerate (distance between them is near zero)
    float d = FloatPoint3::SumComponents(DirectX::XMVectorPow(ray, { 2.f,2.f ,2.f ,2.f }));
    if (d < epsilon_squared)
        return pt[0];

    FloatPoint3 ab = pointIn - pt[0];

    // parametric coordinate of the nearest point along the line (infinite in direction of ray)
    float parameter = FloatPoint3::SumComponents(ray * ab) / d;
    parameter = std::fmaxf(0.0f, std::fminf(1.0f, parameter)); // limit to the line segment (with range 0.0 to 1.0)

    FloatPoint3 nearestPoint = parameter * ray + pt[0];
    
    return nearestPoint;
}
inline bool King::Line::Collision(Point const & pointIn) const { return Intersects(pointIn); }
inline bool King::Line::Collision(Line const & lineIn) const { return Intersects(lineIn); }
inline bool King::Line::Collision(Sphere const & sphereIn) const { return sphereIn.Intersects(*this); }
inline bool King::Line::Collision(Box const & boxIn) const { return boxIn.Intersects(*this); }
/******************************************************************************
*    Method:    Intersects
******************************************************************************/

bool King::Quad::Intersects(const FloatPoint3 & rayOriginIn, const FloatPoint3 & rayDirectionIn, FloatPoint3 * intersectPointOut)
{
    auto t1 = GetTriangle1();
    if (t1.Intersects(rayOriginIn, rayDirectionIn, intersectPointOut))
        return true;

    auto t2 = GetTriangle2();
    if (t2.Intersects(rayOriginIn, rayDirectionIn, intersectPointOut))
        return true;

    return false;
}
/******************************************************************************
*    Method:    SubDivide
******************************************************************************/
void King::Quad::SubDivide(std::vector<Quad>* quadsOut)
{
    if (quadsOut == nullptr) return;
    
    FloatPoint3 c = GetCenter();
    FloatPoint3 c0 = 0.5f * (pt[1] + pt[0]);
    FloatPoint3 c1 = 0.5f * (pt[2] + pt[1]);
    FloatPoint3 c2 = 0.5f * (pt[3] + pt[2]);
    FloatPoint3 c3 = 0.5f * (pt[0] + pt[3]);

    Quad q;

    // 0
    q.pt[0].Set(pt[0]);
    q.pt[1].Set(c0);
    q.pt[2].Set(c);
    q.pt[3].Set(c3);
    quadsOut->push_back(q);
    // 1
    q.pt[0].Set(c0);
    q.pt[1].Set(pt[1]);
    q.pt[2].Set(c1);
    q.pt[3].Set(c);
    quadsOut->push_back(q);
    // 2
    q.pt[0].Set(c);
    q.pt[1].Set(c1);
    q.pt[2].Set(pt[2]);
    q.pt[3].Set(c2);
    quadsOut->push_back(q);
    // 3
    q.pt[0].Set(c3);
    q.pt[1].Set(c);
    q.pt[2].Set(c2);
    q.pt[3].Set(pt[3]);
    quadsOut->push_back(q);
}
/******************************************************************************
*    Method:    Intersects
*       Sphere vs. Point
******************************************************************************/
bool King::Sphere::Contains(const Point & ptIn) const
{
    auto d = GetCenter() - static_cast<FloatPoint3>(*this);
    auto dsq = d * d;

    float r = GetRadius();
    float3 r3 = DirectX::XMVectorReplicatePtr(&r);
    auto rsq = r3 * r3;

    return  DirectX::XMVector3LessOrEqual(dsq, rsq);
}
/******************************************************************************
*    Method:    Intersects
*       Sphere vs. Ray
*      /----\           ray
*     /center\   -----*---->
*     |   *<-|---ray2
*     \      /
*      \----/
*   Reference: DirectXCollision.inl
******************************************************************************/
inline bool King::Sphere::Intersects(const Ray & ray, float & distOut) const
{
    float3 r(GetRadius());
    float3 r2 = XMVectorMultiply(r, r);
    float3 ray2( GetCenter() - ray.GetOrigin());
    float3 s( XMVector3Dot(ray2, ray.GetDirection())); // projection
    float3 l2( XMVector3Dot(ray2, ray2) );

    // m2 is squared distance from the center of the sphere to the projection.
    float3 m2 = XMVectorNegativeMultiplySubtract(s, s, l2);

    XMVECTOR NoIntersection;
    // If the ray origin is outside the sphere and the center of the sphere is
    // behind the ray origin there is no intersection.
    NoIntersection = XMVectorAndInt(XMVectorLess(s, XMVectorZero()), XMVectorGreater(l2, r2));
    // If the squared distance from the center of the sphere to the projection
    // is greater than the radius squared the ray will miss the sphere.
    NoIntersection = XMVectorOrInt(NoIntersection, XMVectorGreater(m2, r2));

    if (XMVector4NotEqualInt(NoIntersection, XMVectorTrueInt()))
    {
        // The ray hits the sphere, compute the nearest intersection point.
        float3 q( Sqrt(r2 - m2) );
        float3 t1( s - q );
        float3 t2( s + q );

        l2 <= r2;

        XMVECTOR OriginInside = XMVectorLessOrEqual(l2, r2);
        XMVECTOR t = XMVectorSelect(t1, t2, OriginInside);
        XMStoreFloat(&distOut, t);
        return true;
    }
    distOut = 0.f;
    return false;
}
/******************************************************************************
*    Method:    Intersects
*       Sphere vs. Line
******************************************************************************/
inline bool King::Sphere::Intersects(const Line & lineIn) const
{
    auto nearestPoint = lineIn.FindNearestPointOnLineSegment(GetCenter());
    if (Contains(nearestPoint))
        return true;
    else
        return false;
}
/******************************************************************************
*    Method:    Intersects Sphere
*       Sphere vs. Sphere
******************************************************************************/
bool King::Sphere::Intersects(const Sphere & rhs) const
{
    float3 d( rhs.GetCenter() - GetCenter() );
    float3 dsq( DirectX::XMVector3LengthSq(d) );

    float rA = GetRadius();
    float3 r3A(rA);
    float rB = rhs.GetRadius();
    float3 r3B(rB);
    auto rsq = r3A + r3B;
    rsq = rsq * rsq;

    return  dsq <= rsq;
}

inline bool King::Sphere::Collision(Point const & pointIn) const { return Contains(pointIn); }
inline bool King::Sphere::Collision(Ray const & rayIn) const { float distOut; return Intersects(rayIn, distOut); }
inline bool King::Sphere::Collision(Line const & lineIn) const { return Intersects(lineIn); }
inline bool King::Sphere::Collision(Sphere const & sphereIn) const { return Intersects(sphereIn); }
inline bool King::Sphere::Collision(Box const & boxIn) const { return boxIn.Intersects(*this); }
/******************************************************************************
*    Method:    Merge Box
******************************************************************************/
void King::Box::Merge(const Box & bIn)
{
        pt_min = Min(pt_min, bIn.pt_min);
        pt_max = Max(pt_max, bIn.pt_max);
        assert(pt_min <= pt_max);
}
/******************************************************************************
*    Method:    Merge Point
******************************************************************************/
inline void __vectorcall King::Box::Merge(const FloatPoint3 & ptIn)
{
    pt_min = Min(pt_min, ptIn);
    pt_max = Max(pt_max, ptIn);
}
/******************************************************************************
*    Method:    Intersects Ray
******************************************************************************/
bool King::Box::Intersects(const Ray &rayIn, float &distOut) const
{
    // from DirectXCollision.h

    assert(DirectX::Internal::XMVector3IsUnit(rayIn.GetDirection()));

    // Load the box.
    XMVECTOR vExtents = GetExtents();

    // Compute the dot product againt each axis of the box.
    // Since the axii are (1,0,0), (0,1,0), (0,0,1) no computation is necessary.
    float3 AxisDotOrigin = rayIn.GetOrigin() - GetCenter();
    float3 AxisDotDirection = rayIn.GetDirection();

    // if (fabs(AxisDotDirection) <= Epsilon) the ray is nearly parallel to the slab.
    XMVECTOR IsParallel = DirectX::XMVectorLessOrEqual(DirectX::XMVectorAbs(AxisDotDirection), g_RayEpsilon);

    // Test against all three axii simultaneously.
    XMVECTOR InverseAxisDotDirection = DirectX::XMVectorReciprocal(AxisDotDirection);
    XMVECTOR t1 = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(AxisDotOrigin, vExtents), InverseAxisDotDirection);
    XMVECTOR t2 = DirectX::XMVectorMultiply(DirectX::XMVectorAdd(AxisDotOrigin, vExtents), InverseAxisDotDirection);

    // Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
    // use the results from any directions parallel to the slab.
    XMVECTOR t_min = DirectX::XMVectorSelect(DirectX::XMVectorMin(t1, t2), g_FltMin, IsParallel);
    XMVECTOR t_max = DirectX::XMVectorSelect(DirectX::XMVectorMax(t1, t2), g_FltMax, IsParallel);

    // t_min.x = maximum( t_min.x, t_min.y, t_min.z );
    // t_max.x = minimum( t_max.x, t_max.y, t_max.z );
    t_min = DirectX::XMVectorMax(t_min, DirectX::XMVectorSplatY(t_min));  // x = max(x,y)
    t_min = DirectX::XMVectorMax(t_min, DirectX::XMVectorSplatZ(t_min));  // x = max(max(x,y),z)
    t_max = DirectX::XMVectorMin(t_max, DirectX::XMVectorSplatY(t_max));  // x = min(x,y)
    t_max = DirectX::XMVectorMin(t_max, DirectX::XMVectorSplatZ(t_max));  // x = min(min(x,y),z)

    // if ( t_min > t_max ) return false;
    XMVECTOR NoIntersection = DirectX::XMVectorGreater(DirectX::XMVectorSplatX(t_min), DirectX::XMVectorSplatX(t_max));

    // if ( t_max < 0.0f ) return false;
    NoIntersection = DirectX::XMVectorOrInt(NoIntersection, DirectX::XMVectorLess(DirectX::XMVectorSplatX(t_max), DirectX::XMVectorZero()));

    // if (IsParallel && (-Extents > AxisDotOrigin || Extents < AxisDotOrigin)) return false;
    XMVECTOR ParallelOverlap = DirectX::XMVectorInBounds(AxisDotOrigin, vExtents);
    NoIntersection = DirectX::XMVectorOrInt(NoIntersection, DirectX::XMVectorAndCInt(IsParallel, ParallelOverlap));

    if (!DirectX::Internal::XMVector3AnyTrue(NoIntersection))
    {
        // Store the x-component to *pDist
        XMStoreFloat(&distOut, t_min);
        return true;
    }

    distOut = 0.f;
    return false;
}
inline bool King::Box::Intersects(const Line & lineIn) const 
{ 
    Point pt1(lineIn.GetVertex(0));
    Point pt2(lineIn.GetVertex(1));
    Ray r1(pt1, pt2 - pt1); // pt1 *--->   * pt2
    Ray r2(pt2, pt1 - pt2); // pt1 *   <---* pt2
    float distOut;
    if (Intersects(r1, distOut) && Intersects(r2, distOut))
        return true;
    else
        return false;
}
/******************************************************************************
*    Method:    Intersects Box
*
*   |------|*this
*   |      |
*   |      |
*   | |----|-| boxIn
*   |------| |
*     |      |
*     |      |
*     |------|
******************************************************************************/
bool King::Box::Intersects(const Box & boxIn) const
{
    if (pt_min > boxIn.pt_max || boxIn.pt_min > pt_max)
        return false;

    return true;
}
/******************************************************************************
*    Method:    Intersects Sphere
*
*   |------|*this
*   |      |
*   |      |
*   |  /---|\ sphereIn
*   |------| \
*     |      |
*     \      /
*      \----/
******************************************************************************/
bool King::Box::Intersects(const Sphere & sphereIn) const
{
    auto centerToNearestPointDistance = FindNearestPointOnBox(sphereIn.GetCenter()) - sphereIn.GetCenter();

    // square components and sum them together.
    XMVECTOR distanceSquared = XMVector3Dot(centerToNearestPointDistance, centerToNearestPointDistance);

    XMVECTOR r = XMVectorReplicate(sphereIn.GetRadius());
    return XMVector3LessOrEqual(distanceSquared, XMVectorMultiply(r, r));
}
/******************************************************************************
*    Method:    Find Nearest Point On Box
*   |------|
*   |      |
*   |      |*<--------------------* pt3In
*   |      |return
*   |------|
******************************************************************************/
FloatPoint3 __vectorcall King::Box::FindNearestPointOnBox(const FloatPoint3 &pt3In) const
{
    return Max(pt_min, Min(pt_max, rbf));
}
/******************************************************************************
*    Method:    Find Nearest Point On Box (transformed)
*   \-----\               ------*  pt3In (world)
*    \     \ return ------
*     \     \*<-----
*      \     \  *<---------------* localSpacePoint (box can be treated as a AABB)
*       \-----\ (npOnBox)  
******************************************************************************/
FloatPoint3 __vectorcall King::Box::FindNearestPointOnBox(const FloatPoint3 & pt3In, const DirectX::FXMMATRIX & M) const
{
    // world to box space
    FloatPoint3 localSpacePoint(pt3In);
    localSpacePoint *= M;

    auto npOnBox = FindNearestPointOnBox(localSpacePoint);
    
    // box to world space
    auto det = DirectX::XMMatrixDeterminant(M);
    auto invM = DirectX::XMMatrixInverse(&det, M);
    npOnBox *= invM;

    return npOnBox;
}
inline bool King::Box::Collision(Ray const & rayIn) const { float distOut; return Intersects(rayIn, distOut); }
inline bool King::Box::Collision(Point const & pointIn) const { return DirectX::XMVector3InBounds((XMVECTOR)pointIn - GetCenter(), GetExtents()); }
inline bool King::Box::Collision(Line const & lineIn) const { return Intersects(lineIn); }
inline bool King::Box::Collision(Sphere const & sphereIn) const { return Intersects(sphereIn); }
inline bool King::Box::Collision(Box const & boxIn) const { return Intersects(boxIn); }
/******************************************************************************
*    Method:    Contains Point
*   |------|
*   |      |
*   |  *   |
*   |  ptIn|
*   |------|
******************************************************************************/
bool __vectorcall King::Box::Contains(const FloatPoint3 & ptIn) const
{
    return DirectX::XMVector3InBounds(ptIn - GetCenter(), GetExtents());
}
/******************************************************************************
*    Method:    Contains Box
*   |------|
*   |  --- |
*   |  | | |
*   |  --- |
*   |------|
******************************************************************************/
// boxIn is completely inside of this Box
bool King::Box::Contains(const Box & boxIn) const
{
    if(pt_min >= boxIn.pt_min && pt_max <= boxIn.pt_max)
        return false;

    return true;
}
/******************************************************************************
*    Method:    SetAABBfromThisTransformedBox
******************************************************************************/
void King::Box::SetAABBfromThisTransformedBox(FXMMATRIX M)
{
    auto corners = GetCorners8Transformed(M);

    FloatPoint3 min, max, corner;
    min = max = corner = corners[0];

    for (size_t i = 1; i < 8; ++i)
    {
        min = Min(min, corner[i]);
        max = Max(max, corner[i]);
    }
    Set(min, max);
}
/******************************************************************************
*    Method:    GetCorners8Transformed
******************************************************************************/
std::vector<King::FloatPoint3> __vectorcall King::Box::GetCorners8Transformed(const DirectX::FXMMATRIX &M)
{
    DirectX::XMFLOAT3 pArray[8];
    GetCorners8(pArray);

    DirectX::XMFLOAT4 pArrayOut[8];
    DirectX::XMVector3TransformStream(pArrayOut, sizeof(XMFLOAT4), pArray, sizeof(XMFLOAT3), 8, M);

    std::vector<King::FloatPoint3> rtn;
    for (int i = 0; i < 8; ++i)    rtn.push_back(FloatPoint3(pArrayOut[i]));
    
    return rtn;
}
/******************************************************************************
*    Method:    GetCorners8
******************************************************************************/
std::vector<King::FloatPoint3> __vectorcall King::Box::GetCorners8(const Quaternion * quaternionIn)
{
    DirectX::XMFLOAT3 pArrayInOut[8];
    GetCorners8(pArrayInOut, quaternionIn);

    std::vector<FloatPoint3> rtn;
    for (int i = 0; i < 8; ++i)    rtn.push_back(FloatPoint3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCornersTop4
******************************************************************************/
std::vector<King::FloatPoint3> __vectorcall King::Box::GetCornersTop4(const Quaternion * quaternionIn) // scaled, rotated and translated
{
    DirectX::XMFLOAT3 pArrayInOut[4];
    GetCornersTop4(pArrayInOut, quaternionIn);

    std::vector<King::FloatPoint3> rtn;
    for (int i = 0; i < 4; ++i)    rtn.push_back(King::FloatPoint3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCornersBottom4
******************************************************************************/
std::vector<King::FloatPoint3> __vectorcall King::Box::GetCornersBottom4(const Quaternion * quaternionIn) // scaled, rotated and translated
{
    DirectX::XMFLOAT3 pArrayInOut[4];
    GetCornersBottom4(pArrayInOut, quaternionIn);

    std::vector<King::FloatPoint3> rtn;
    for (int i = 0; i < 4; ++i)    rtn.push_back(King::FloatPoint3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCornersLowest4
******************************************************************************/
std::vector<King::FloatPoint3> __vectorcall King::Box::GetCornersLowest4(const Quaternion * quaternionIn) // scaled, rotated and translated
{
    DirectX::XMFLOAT3 pArrayInOut[4];
    GetCornersLowest4(pArrayInOut, quaternionIn);

    std::vector<King::FloatPoint3> rtn;
    for (int i = 0; i < 4; ++i)    rtn.push_back(King::FloatPoint3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCorners8
******************************************************************************/
void King::Box::GetCorners8(DirectX::XMFLOAT3 *pArrayInOut, const Quaternion * quaternionIn)
{
    assert(pArrayInOut != nullptr);

    King::FloatPoint3 corn;

    auto c = GetCenter();
    auto e = pt_max - c;

    if (quaternionIn == nullptr)
    {
        for (size_t i = 0; i < 8; ++i)
            pArrayInOut[i] = King::FloatPoint3(XMVectorMultiplyAdd(g_boxCorners[i], e, c)); // scale and translate
    }
    else
    {
        for (size_t i = 0; i < 8; ++i)
        {
            corn = DirectX::XMVectorMultiply(g_boxCorners[i], e); // scale to extents
            corn = DirectX::XMVector3Rotate(corn, *quaternionIn); // rotate
            corn = DirectX::XMVectorAdd(corn, c); // translate
            pArrayInOut[i] = corn;
        }
    }
}
/******************************************************************************
*    Method:    GetCornersBottom4
******************************************************************************/
void King::Box::GetCornersBottom4(DirectX::XMFLOAT3 *pArrayInOut, const Quaternion * quaternionIn)
{
    assert(pArrayInOut != nullptr);

    King::FloatPoint3 corn;

    King::FloatPoint3 c = GetCenter();
    King::FloatPoint3 e = pt_max - c;

    if (quaternionIn == nullptr)
    {
        for (size_t i = 0; i < 4; ++i)
            pArrayInOut[i] = FloatPoint3(XMVectorMultiplyAdd(g_boxCorners[i], e, c)); // scale and translate
        return;
    }
    else
    {
        for (size_t i = 0; i < 4; ++i)
        {
            corn = DirectX::XMVectorMultiply(g_boxCorners[i], e); // scale to extents
            corn = DirectX::XMVector3Rotate(corn, *quaternionIn); // rotate
            corn = DirectX::XMVectorAdd(corn, c); // translate
            pArrayInOut[i] = corn;
        }
    }
}
/******************************************************************************
*    Method:    GetCornersTop4
******************************************************************************/
void King::Box::GetCornersTop4(DirectX::XMFLOAT3 *pArrayInOut, const Quaternion * quaternionIn)
{
    assert(pArrayInOut != nullptr);

    King::FloatPoint3 corn;

    King::FloatPoint3 c = GetCenter();
    King::FloatPoint3 e = pt_max - c;

    if (quaternionIn == nullptr)
    {
        for (size_t i = 0; i < 4; ++i)
            pArrayInOut[i] = FloatPoint3(XMVectorMultiplyAdd(g_boxCorners[i+4], e, c)); // scale and translate
        return;
    }
    else
    {
        for (size_t i = 0; i < 4; ++i)
        {
            corn = DirectX::XMVectorMultiply(g_boxCorners[i+4], e); // scale to extents
            corn = DirectX::XMVector3Rotate(corn, *quaternionIn); // rotate
            corn = DirectX::XMVectorAdd(corn, c); // translate
            pArrayInOut[i] = corn;
        }
    }
}
/******************************************************************************
*    Method:    GetCornersLowest4
******************************************************************************/
void King::Box::GetCornersLowest4(DirectX::XMFLOAT3 * pArrayInOut, const Quaternion * quaternionIn)
{
    assert(pArrayInOut != nullptr);

    DirectX::XMFLOAT3 corn[8];
    GetCorners8(corn, quaternionIn);

    pArrayInOut[0] = { 0.f,FLT_MAX,0.f };
    pArrayInOut[1] = { 0.f,FLT_MAX,0.f };
    pArrayInOut[2] = { 0.f,FLT_MAX,0.f };
    pArrayInOut[3] = { 0.f,FLT_MAX,0.f };

    for (size_t i = 0; i < 8; ++i)
    {
        // find the larget value
        size_t keep = 0;
        for (size_t j = 0; j < 3; ++j)
        {
            if (pArrayInOut[keep].y != std::fmaxf(pArrayInOut[keep].y, pArrayInOut[j+1].y))
                keep = j + 1;
        }
        // compare y height to max value in the minimum array and store the lowest
        if (corn[i].y < pArrayInOut[keep].y)
            pArrayInOut[keep] = corn[i];
    }
}
/******************************************************************************
*    Method:    operator= ; copy assignment
******************************************************************************/
inline King::LineModel & King::LineModel::operator= (const King::LineModel &in)
{
    _modelName = in._modelName;
    _vertexFormat = in._vertexFormat;
    _indexFormat = in._indexFormat;
    _boundingBox = in._boundingBox;

    // buffers are more complicated. Each mesh holds a pointer to the data
    //  in these buffers, so when we copy the new data we need the new addresses 
    //  assigned to the copies of the meshes.
    _vertexBufferMaster = in._vertexBufferMaster;
    _indexBufferMaster = in._indexBufferMaster;
    _meshes = in._meshes;

    for (auto & m : _meshes)
    {
        m->SetVB(&_vertexBufferMaster.GetData());
        m->SetIB(&_indexBufferMaster.GetData());
    }
    return *this;
}
/******************************************************************************
*    Method:    operator= ; copy assignment
******************************************************************************/
inline King::Model & King::Model::operator= (const King::Model &in)
{
    _modelName = in._modelName;
    _materials = in._materials;
    _vertexFormat = in._vertexFormat;
    _indexFormat = in._indexFormat;
    _boundingBox = in._boundingBox;

    // buffers are more complicated. Each mesh holds a pointer to the data
    //  in these buffers, so when we copy the new data we need the new addresses 
    //  assigned to the copies of the meshes.
    _vertexBufferMaster = in._vertexBufferMaster;
    _indexBufferMaster = in._indexBufferMaster;
    _meshes = in._meshes;

    for (auto & m : _meshes)
    {
        m.SetVB(&_vertexBufferMaster.GetData());
        m.SetIB(&_indexBufferMaster.GetData());
    }
    return *this;
}
/******************************************************************************
*    Method:    Load
******************************************************************************/
bool King::Model::Load(std::wstring fileNameIN)
{ 
    Model_IO m;
    
    std::filesystem::path p(fileNameIN);
    
    // simple models and a very common file format for inter-operability
    if ((p.extension() == L".obj" || p.extension() == L".OBJ" || p.extension() == L".Obj"))
    {
        auto models = m.Load_OBJ(fileNameIN, &_vertexFormat); // returns a standard model
        if (models.size())// just want one
        {
            *this = *models.back(); // copies model contents to our model
        }
        if (models.size()) return true;
    }
    // complex models with bones and animation clips with a good readable file format but poorly supported
    if ((p.extension() == L".m3d" || p.extension() == L".M3D" || p.extension() == L".M3d" || p.extension() == L".m3D"))
    {
        //auto models = m.Load_M3D(fileNameIN, &_vertexFormat); // returns a standard model
        //if (models.size())// just want one
        //{
        //    *this = *models.back(); // model will ignore the bones and animation clips on copy
        //}
        //if (models.size()) return true;
    }

    return false;
}
/******************************************************************************
*    Method:    Save
******************************************************************************/
bool King::Model::Save(std::wstring fileNameIN) 
{ 
    bool rtn;
    Model_IO m; 
    std::vector<std::shared_ptr<Model>> models; 
    models.push_back(std::dynamic_pointer_cast<Model>(shared_from_this()));
    rtn = m.Save_OBJ(fileNameIN, models); 
    if(rtn)
        rtn = m.Save_MTL(fileNameIN, models); 
    return rtn;
}
/******************************************************************************
*    Method:    CalculateBoundingBox
******************************************************************************/
Box King::LineModel::CalculateBoundingBox()
{
    _boundingBox.SetZero();

    for (auto & mesh : _meshes)
    {
        _boundingBox.Merge(mesh->CalculateBoundingBox());
    }
    return _boundingBox;
}
/******************************************************************************
*    Method:    CalculateBoundingBox
******************************************************************************/
Box King::Model::CalculateBoundingBox() 
{
    _boundingBox.SetZero();

    for (auto & mesh : _meshes)
    {
        _boundingBox.Merge(mesh.CalculateBoundingBox());
    }
    return _boundingBox;
}
/******************************************************************************
*    Method:    CalculateTangentsAndBiTangents
    3    2
    -----
    |\  |
    | \ |
    |  \|
    -----
    0   1
CCW winding order for right hand system.
tr1 = 1, 3, 0 ; Order our vertex using blender vertex ordering
tr2 = 1, 2, 3

EX: Bottom plane of box
    vn 0.000000 -1.000000 0.000000
    v0 0.500000 -0.500000 -0.500000     (+x, -y, -z)
    v1 0.500000 -0.500000 0.500000      (+x, -y, +z)
    v2 -0.500000 -0.500000 0.500000     (-x, -y, +z)
    v3 -0.500000 -0.500000 -0.500000    (-x, -y, -z)
    vt0 0.500000 0.000000
    vt1 0.500000 0.250000
    vt2 0.250043 0.250000
    vt3 0.250043 0.000000

    tr1 = v1, v3, v0 = (+x, -y, +z) (-x, -y, -z) (+x, -y, -z)
    tr2 = v1, v2, v3 = (+x, -y, +z) (-x, -y, +z) (-x, -y, -z)
            -x
              | -----
              | |\  |
              | | \ |
              | |  \|
            +x| -----
            -z_________+z
            and the normal points away from us in the -y direction
******************************************************************************/
void King::Model::CalculateTangentsAndBiTangents()
{
    if (!(_vertexFormat.Has(VertexAttrib::position) &&
        _vertexFormat.Has(VertexAttrib::textureCoord) &&
        _vertexFormat.Has(VertexAttrib::normal) &&
        _vertexFormat.Has(VertexAttrib::tangent) &&
        _vertexFormat.Has(VertexAttrib::bitangent) &&
        (_vertexBufferMaster)))
        return;

    const auto positionAttribute = _vertexFormat.GetAttribute(_vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::position));
    const auto textureCoordAttribute = _vertexFormat.GetAttribute(_vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::textureCoord));
    const auto normalAttribute = _vertexFormat.GetAttribute(_vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::normal));
    const auto tangentAttribute = _vertexFormat.GetAttribute(_vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::tangent));
    const auto bitangentAttribute = _vertexFormat.GetAttribute(_vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::bitangent));

    float2 uv;
    float3 tangent;
    float3 bitangent;
    const auto & ib = _indexBufferMaster;
    auto & vb = _vertexBufferMaster;

    for (size_t i = 0; i < ib.GetLength(); i+=3)
    {
        float3 pt0(reinterpret_cast<float*>(&vb[ib[i + 0]] + positionAttribute.GetOffset()));
        float3 pt1(reinterpret_cast<float*>(&vb[ib[i + 1]] + positionAttribute.GetOffset()));
        float3 pt2(reinterpret_cast<float*>(&vb[ib[i + 2]] + positionAttribute.GetOffset()));

        float3 uv0(reinterpret_cast<float*>(&vb[ib[i + 0]] + textureCoordAttribute.GetOffset()));
        float3 uv1(reinterpret_cast<float*>(&vb[ib[i + 1]] + textureCoordAttribute.GetOffset()));
        float3 uv2(reinterpret_cast<float*>(&vb[ib[i + 2]] + textureCoordAttribute.GetOffset()));

        float3 n0(reinterpret_cast<float*>(&vb[ib[i + 0]] + normalAttribute.GetOffset()));
        float3 n1(reinterpret_cast<float*>(&vb[ib[i + 0]] + normalAttribute.GetOffset()));
        float3 n2(reinterpret_cast<float*>(&vb[ib[i + 0]] + normalAttribute.GetOffset()));
        /*
        pt0 :  0.500000 -0.500000  0.500000
        pt1 : -0.500000 -0.500000 -0.500000
        pt2 :  0.500000 -0.500000 -0.500000
        uv0 :  0.500000  0.250000
        uv1 :  0.250043  0.000000
        uv2 :  0.500000  0.000000
        -t| 1
          | |\  
          | | \ 
          | |  \
          | -----
        +t| 2   0
        -b_________+b

        In ccw (right hand system), the normal, N, is out of the page.  cross(N,T) = B (+). 
        Now, if the normal is reversed but the winding system is left as CCW
        Or, winding system is reversed in the model to get a reverse normal, we need to negate the tangent
        */
        auto dpt1 = pt1 - pt0; // -1.0 0.0 -1.0
        auto dpt2 = pt2 - pt0; //  0.0 0.0 -1.0

        auto duv1 = uv1 - uv0; // -0.25 -0.25
        auto duv2 = uv2 - uv0; // 0 -0.25

        float r = 1.0f / (duv1.GetX() * duv2.GetY() - duv1.GetY() * duv2.GetX()); // 1.0f / ( -0.25 * -0.25 - -0.25 * 0 ) = 16
        tangent = (dpt1 * duv2.GetY() - dpt2 * duv1.GetY())*r; // [(-1.0 0.0 -1.0) * -0.25 - (0.0 0.0 -1.0) * -0.25] * 16 = (4 0 0)
        bitangent = (dpt2 * duv1.GetX() - dpt1 * duv2.GetX())*r; // [(0.0 0.0 -1.0) * -0.25 - (0.0 0.0 -1.0) * 0] * 16 = (0 0 8)
        tangent.MakeNormalize(); // (1 0 0)
        bitangent.MakeNormalize(); // (0 0 1)

        // handiness correctness
        float3 normal;
        normal = n0 + n1 + n2;
        normal.MakeNormalize();
        if (Dot(Cross(normal, tangent), bitangent) < 0.0f)
            tangent *= -1.0f;

        // save the results
        DirectX::XMFLOAT3 t, bt;
        t = tangent;
        bt = bitangent;
        auto t0 = reinterpret_cast<float*>(&vb[ib[i + 0]] + tangentAttribute.GetOffset());
        auto t1 = reinterpret_cast<float*>(&vb[ib[i + 1]] + tangentAttribute.GetOffset());
        auto t2 = reinterpret_cast<float*>(&vb[ib[i + 2]] + tangentAttribute.GetOffset());

        t0[0] = t.x;
        t0[1] = t.y;
        t0[2] = t.z;

        t1[0] = t.x;
        t1[1] = t.y;
        t1[2] = t.z;

        t2[0] = t.x;
        t2[1] = t.y;
        t2[2] = t.z;

        auto bt0 = reinterpret_cast<float*>(&vb[ib[i + 0]] + bitangentAttribute.GetOffset());
        auto bt1 = reinterpret_cast<float*>(&vb[ib[i + 1]] + bitangentAttribute.GetOffset());
        auto bt2 = reinterpret_cast<float*>(&vb[ib[i + 2]] + bitangentAttribute.GetOffset());

        bt0[0] = bt.x;
        bt0[1] = bt.y;
        bt0[2] = bt.z;

        bt1[0] = bt.x;
        bt1[1] = bt.y;
        bt1[2] = bt.z;

        bt2[0] = bt.x;
        bt2[1] = bt.y;
        bt2[2] = bt.z;
    }
}
/******************************************************************************
*    Method:    OptimizeMeshVertexBuffer
*       Re-order vertex to match index buffer ordering from first encountered
*       to last encountered.
*       2    3    5    7    9    11
*       ---------------------
*       |\  |\  |\  |\  |\  |
*       | \ | \ | \ | \ | \ |
*       |  \|  \|  \|  \|  \|
*       ---------------------
*       0   1    4    6    8    10
*       
*       CCW index order then is: 0,1,2 2,1,3 1,4,3 3,4,5 4,6,5 5,6,7
*       Vertex order would be optimized to: 0,1,2,3,4,5,6, etc.
******************************************************************************/
void King::Model::OptimizeMeshVertexBuffer(const TriangleMesh & forMeshIn)
{
    vector<uint8_t> new_verts;
    vector<uint32_t> newIndexBuffer;
    auto ib = forMeshIn.GetIB();
   
    const auto meshNumVerts = forMeshIn.GetNumVerticies();
    const auto meshNumIndex = forMeshIn.GetNumIndicies();

    auto stride = GetVertexStride();
    vector<uint8_t> vert(stride);

    newIndexBuffer.resize(meshNumIndex, UINT32_MAX);
    new_verts.reserve((size_t)(meshNumVerts * stride));
    uint32_t vertexSwapped = 0;

    vector<pair<uint32_t, uint32_t>> encounteredIndicies; // old, remapped
    encounteredIndicies.reserve(meshNumVerts); // there cannot be more than our verticies

    for (uint32_t i=0; i < meshNumIndex; ++i)
    {
        if (newIndexBuffer[i] == UINT32_MAX)
        {
            // first visit to new index space
            // have we encountered the value at this original index before?
            auto value = *(ib + i);

            auto it = find_if(encounteredIndicies.begin(), encounteredIndicies.end(),
                [&value](const pair<uint32_t, uint32_t>& element) { return element.first == value; });

            if (it != encounteredIndicies.end())
            {
                // already seen before, remapped the index
                newIndexBuffer[i] = it->second;
            }
            else
            {
                // no, create a vertex in our new buffer
                uint8_t* vbAddr = GetVertexAddr(i, forMeshIn.GetVBStart(), forMeshIn.GetIBStart());
                copy(vbAddr, vbAddr + stride, vert.data());
                for(unsigned int j=0;j<stride;++j)
                    new_verts.push_back(vert[j]); 

                newIndexBuffer[i] = (uint32_t)new_verts.size() / stride - 1; // zero based
                encounteredIndicies.push_back(pair<uint32_t, uint32_t>(value, newIndexBuffer[i]));

                if (value != newIndexBuffer[i])
                    ++vertexSwapped;
            }
        }
    }
    // count indicies swapped
    uint32_t indexRemapped = 0;
    for (uint32_t i = 0; i < meshNumIndex; ++i)
    {
        if (newIndexBuffer[i] != *(ib+i)) ++indexRemapped;
    }
    // copy reordered vertex buffer
    if (meshNumVerts == new_verts.size()/stride)
        copy(new_verts.begin(), new_verts.end(), forMeshIn.GetVB());
    // copy remapped index buffer
    if (meshNumIndex == newIndexBuffer.size())
        copy(newIndexBuffer.begin(), newIndexBuffer.end(), ib);

    cout << "OptimizeVertexBuffer" << '\n';
    cout << "  " << "Verticies before: " << meshNumVerts << '\n';
    cout << "  " << "Verticies after:  " << new_verts.size()/stride << '\n';
    cout << "  " << "Verticies reordered: " << vertexSwapped << '\n';
    cout << "  " << "Indicies before: " << meshNumIndex << '\n';
    cout << "  " << "Indicies after: " << newIndexBuffer.size() << '\n';
    cout << "  " << "Indicies remapped: " << indexRemapped << '\n';
}
/******************************************************************************
*    Method:    ReportOut
******************************************************************************/
void King::Model::LogReport(const std::wstring fileNameIn)
{
    std::ofstream file(fileNameIn, ios::trunc);
    char str[50];

    if(file.good())
    {
        file << "# Model Report" << '\n';
        file << "# " << '\n';
        std::string fileName(fileNameIn.begin(), fileNameIn.end()); // truncates and mostly only works for latin alpha
        file << "# File name: " << fileName << '\n';
        auto t = std::time(nullptr);
        tm tm;
        localtime_s( &tm, &t );
        asctime_s(str, sizeof str, &tm);
        file << "# Created: " << str;
        file << '\n';
        file << "# Statistics:" << '\n';
        // stats
        file << "# " << "Model Name: " << GetModelName() << '\n' ;
        file << "# " << "Meshes: " << std::to_string(_meshes.size()) << '\n' ;
        file << "# " << "Materials: " << std::to_string(_materials.size()) << '\n' ;
        file << "# " << "Vertex Stride: " << std::to_string(GetVertexStride()) << " b"<< '\n' ;
        file << "# " << "Vertex format: ";
        auto format = GetVertexFormat();
        for (int i = 0; i < format.GetNumAttributes(); ++i)
            file << " " << format.GetAttribute(i).GetDescriptionString() << "(" << std::to_string(format.GetAttribute(i).GetByteSize()) << ")";
        file << '\n' ;
        file << "# " << "Total Verticies: " << std::to_string(GetVertexCount()) << '\n' ;
        file << "# " << "Total Indicies: " << std::to_string(GetIndexCount()) << '\n' ;
        file << "# " << "Bounding Box: " << GetBoundingBox() << '\n' ;
        file << "# " << "Memory storage: " << std::to_string((sizeof(*this) + GetVertexBufferMaster().GetLength() + GetIndexBufferMaster().GetLength())/1024) << " Kb" << '\n' ;

        size_t counter = 0;
        for (auto & mesh : _meshes)
        {
            file << '\n' ;
            file << "# " << "Mesh[" << std::to_string(counter) << "]" << '\n' ;
            file << "# " << "  " << "Name: " << mesh.Get_name() << '\n' ;
            file << "# " << "  " << "Material: " << mesh.Get_materialName() << '\n' ;
            file << "# " << "  " << "Triangles: " << std::to_string(mesh.GetNumTriangles()) << '\n' ;
            file << "# " << "  " << "VB start: " << std::to_string(mesh.GetVBStart()) << '\n' ;
            file << "# " << "  " << "IB start: " << std::to_string(mesh.GetIBStart()) << '\n' ;
            ++counter;
        }
        counter = 0;
        for (auto & mtl : _materials)
        {
            file << '\n' ;
            file << "# " << "Material[" << mtl.first << "]" << '\n' ;
            file << "# " << "  " << "Name: " << mtl.second->Get_name() << '\n' ;
            file << "# " << "  " << "File Dependencies: " << '\n' ;
            auto fn = mtl.second->Get_fileNames();
            if (fn.light.size()) { file << "# " << "    " << "light: " << fn.light << '\n' ; }
            if (fn.diffuse.size()) { file << "# " << "    " << "diffuse: " << fn.diffuse << '\n' ; }
            if (fn.specular.size()) { file << "# " << "    " << "specular: " << fn.specular << '\n' ; }
            if (fn.specular_strength.size()) { file << "# " << "    " << "specular_strength: " << fn.specular_strength << '\n' ; }
            if (fn.normal.size()) { file << "# " << "    " << "normal: " << fn.normal << '\n' ; }
            if (fn.emmissive.size()) { file << "# " << "    " << "emmissive: " << fn.emmissive << '\n' ; }
            if (fn.transparency.size()) { file << "# " << "    " << "transparency: " << fn.transparency << '\n' ; }
            if (fn.displacement.size()) { file << "# " << "    " << "displacement: " << fn.displacement << '\n' ; }
            if (fn.stencil.size()) { file << "# " << "    " << "stencil: " << fn.stencil << '\n' ; }
            if (fn.reflection.size()) { file << "# " << "    " << "reflection: " << fn.reflection << '\n' ; }
            ++counter;
        }
    }
    file.close();
}
/******************************************************************************
*    Method:    GetVertexAddr
******************************************************************************/
inline uint8_t* King::ModelScaffold::GetVertexAddr(const uint32_t indexIn, uint32_t vbStartIn, uint32_t ibStartIn)
{ 
    assert(indexIn < _indexBufferMaster.GetElements());

    auto bs = _vertexFormat.GetByteSize(); 
    uint8_t* vb = &_vertexBufferMaster.GetData() + (size_t)vbStartIn * bs;
    uint32_t* ib = &_indexBufferMaster.GetData() + ibStartIn;
    uint32_t vertNum = *(ib + indexIn);
    assert(vertNum < _vertexBufferMaster.GetElements());

    return vb + *(ib + indexIn) * (size_t)bs;
}
/******************************************************************************
*    Method:    AddVertexAttribute
******************************************************************************/
void King::ModelScaffold::AddVertexAttribute(const VertexAttrib::enumDesc & propertyIn, const VertexAttrib::enumFormat & formatIn)
{
    auto i = _vertexFormat.SetNext(propertyIn, formatIn);
    _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
}
/******************************************************************************
*    Method:    AddMaterial
******************************************************************************/
void King::Model::AddMaterial(std::shared_ptr<Material> mtl_IN)
{
    const auto & mtlName = mtl_IN->Get_name();

    _materials[mtlName] = mtl_IN;

    return;
}
/******************************************************************************
*    Method:    SetVertex
******************************************************************************/
void King::ModelScaffold::SetVertex(const size_t & vertexIndexIn, uint8_t * dataIn)
{
    auto length = _vertexFormat.GetByteSize();
    std::copy(dataIn, dataIn + length, &_vertexBufferMaster[vertexIndexIn]);
}

/******************************************************************************
*    Method:    SetVertexElement
******************************************************************************/
void King::ModelScaffold::SetVertexElement(const size_t & vertexIndexIn, const VertexAttrib::enumDesc & propertyIn, uint8_t * dataIn)
{
    auto index = _vertexFormat.GetAttributeIndexFromDescription(propertyIn);
    assert(index < 8);

    auto length = _vertexFormat.GetAttribute(index).GetByteSize();
    std::copy(dataIn, dataIn + length, &_vertexBufferMaster[vertexIndexIn]);
}

/******************************************************************************
*    Method:    Has VertexAttrib
******************************************************************************/
bool King::VertexFormat::Has(VertexAttrib::enumDesc descIn)
{
    if (GetAttributeIndexFromDescription(descIn) != UINT16_MAX)  // ! error code
        return true;
    else
        return false;
}
/******************************************************************************
*    Method:    GetAttributeIndexFromDescription
******************************************************************************/
uint16_t King::VertexFormat::GetAttributeIndexFromDescription(VertexAttrib::enumDesc descIn) const
{
    for (uint16_t i = 0; i < nextAttribute; ++i)
        if (attributes[i]._desc == descIn)
        {
            return i;
        }
    return UINT16_MAX; // error code
}
/******************************************************************************
*    Method:    CalculateBoundingBox
******************************************************************************/
Box King::LineMesh::CalculateBoundingBox()
{
    Box rtn;
    rtn.SetZero();
    if (_vb && _vertexFormat.Has(VertexAttrib::enumDesc::position))
    {
        rtn.Merge(GetVertexPosition(0));

        const auto to = GetNumIndicies();
        for (uint32_t i = 1; i < to; ++i)
        {
            rtn.Merge(GetVertexPosition(i));
        }
    }
    _boundingBox = rtn;
    return rtn;
}
/******************************************************************************
*    Method:    CalculateBoundingBox
******************************************************************************/
Box King::TriangleMesh::CalculateBoundingBox() 
{
    Box rtn;
    rtn.SetZero();

    if (_vb && _vertexFormat.Has(VertexAttrib::enumDesc::position))
    {
        rtn.Merge(GetVertexPosition(0));

        const auto to = GetNumIndicies();
        for (uint32_t i = 1; i < to; ++i)
        {
            rtn.Merge(GetVertexPosition(i));
        }
    }
    _boundingBox = rtn;
    return rtn;
}
/******************************************************************************
*    Class:    HeightGrid
*    Method:    InitializeXZ
*    Description:    Generates a memory block of height values using interpolation
*        between verts of input mesh for fast lookup later on.  XZ is a reminder
*        that it operates in the XZ plane with Y as the height field.
*    Inputs:        triMeshIn        source of height data
*                densityIn        density of grid
*    Outputs:    heightGrid        re-initializes memory block 
*    Returns:    IntPoint2        X,Z size of height grid
******************************************************************************/
IntPoint2 King::HeightGrid::InitializeXZ(const TriangleMesh & triMeshIn, long densityXIn, long densityZIn)
{
    assert(triMeshIn.GetNumTriangles() > 0);

    _boundingBox = triMeshIn.GetBoundingBox();
    float3 boundingSize = _boundingBox.GetSize();

    float aspectRatio = boundingSize.GetZ() / boundingSize.GetX();
    if (densityXIn == 0)
    {
        // make a guess at a good density 
        auto numQuads = triMeshIn.GetNumTriangles() / 2;
        densityXIn = (long)sqrtf((float)numQuads);
    }
    if (densityZIn == 0)
    { 
        densityZIn = (long)((float)densityXIn * aspectRatio);
    }

    // New memory
    _gridMem.Initialize((size_t)(densityXIn * densityZIn));
    _gridMem.Fill(0.0f);
    
    assert(_gridMem);

    // calculate height values from mesh
    auto strideX = boundingSize.GetX() / (float)densityXIn; // width
    auto strideZ = boundingSize.GetZ() / (float)densityZIn; // depth
    auto above = _boundingBox.GetMax().GetY() + 1.0f; // a little above for ray casting down
    Ray projectDown; 
    projectDown.SetDirection(float3(0.f, -1.f, 0.f));

    for (long z = 0; z < densityZIn; ++z)
    {
        // grid coordinates
        float3 testPt(_boundingBox.GetMin().GetX() + 0.0f, above, (float)z * strideZ + _boundingBox.GetMin().GetZ());
        // grid coordinates
        for (long x = 0; x < densityXIn; ++x)
        {
            projectDown.SetOrigin(testPt);
            float3 origin = projectDown.GetOrigin();
            float2 originXZ(origin.GetX(), origin.GetZ());

            // test  each triangle in mesh until found
            const auto & numTri = triMeshIn.GetNumTriangles();
            for (size_t j = 0; j < numTri; ++j) // all triangles
            {
                auto t = triMeshIn[(uint32_t)j]; // individual triangle
                auto p0 = t.GetVertexPosition(0);
                auto p1 = t.GetVertexPosition(1);
                auto p2 = t.GetVertexPosition(2);
                Triangle2DF triXZ(float2(p0.f[0], p0.f[2]), float2(p1.f[0], p1.f[2]), float2(p2.f[0], p2.f[2]));

                float3 ptOut;
                if( triXZ.Contains(originXZ) && t.Intersects(projectDown, &ptOut)) // contain is a cheaper test and exits early
                {
                    _gridMem[((size_t)z*densityXIn + (size_t)x)] = ptOut.GetY();
                    break; // triangle found, there can be only one
                }
            }
            testPt.SetX(testPt.GetX() + strideX);
        }
    }
    SetXZSize(int2(densityXIn, densityZIn));
    return _xzSize;
}
/******************************************************************************
*    Method:    Height
******************************************************************************/
float King::HeightGrid::Height(const float2 & xzWorldIn)
{
    float h;
    if (Intersects(float3(xzWorldIn.GetX(), 0.f, xzWorldIn.GetY()), &h))
        return h;
    else
        return 100.f;
}
/******************************************************************************
*    Method:    Intersects
******************************************************************************/
/*
    RHS:
        (-)w_z
        ^----------- (+,-)
        |            |
        |            |
        |origin        |
        |(0,0)        |
        ------------> (+)w_x
    GRID:
        -------->
        |        |
        |        |
        --------- (+)x,(+)z
    TRANSPOSE:
        g(x) = (w_x - origin_x) / cell_stride_x
        g(z) = (w_z - origin_z) / cell_stride_z
*/
bool King::HeightGrid::Intersects(const float3 & positionIn, float *heightOut)
{
    if (!_boundingBox.Intersects(Ray(positionIn, float3(0.f, -1.f, 0.f)), *heightOut))
        return false;

    float2 originXZ = float2(_boundingBox.GetMin().GetX(), _boundingBox.GetMin().GetZ()); // world origin
    float3 boundingSize = _boundingBox.GetSize(); // world size
    float2 cell_strideXZ(boundingSize.GetX() / (float)_xzSize.GetX(), boundingSize.GetZ() / (float)_xzSize.GetY());
    float2 posXZ(positionIn.GetX(), positionIn.GetZ());

    // to local grid space
    posXZ -= originXZ;
    // index to value in height grid
    uint2 index = posXZ / cell_strideXZ;
    assert(index.GetX() < (unsigned long)_xzSize.GetX());
    assert(index.GetY() < (unsigned long)_xzSize.GetY());

    float2 interpDelta = (posXZ - (float2)index * cell_strideXZ) / cell_strideXZ; // 0 to 1.f

    *heightOut = _gridMem[(size_t)index.GetX() + (size_t)index.GetY() * _xzSize.GetX()];

    // test for right and bottom edge
    if (index.GetX() + 1 >= (unsigned long)_xzSize.GetX())
        interpDelta.SetX(0.f);
    if (index.GetY() + 1 >= (unsigned long)_xzSize.GetY())
        interpDelta.SetY(0.f);

    float2 interp(float2(1.0f, 1.0f) - interpDelta);
    interp *= *heightOut;
    bool checkDiag = false;

    if (interpDelta.GetX() > 0.f)
    {
        interp.f[0] += interpDelta.GetX() * _gridMem[(size_t)index.GetX() + 1 + index.GetY() * _xzSize.GetX()]; // x
        checkDiag = true;
    }
    if (interpDelta.GetY() > 0.f)
    {
        interp.f[1] += interpDelta.GetY() * _gridMem[(size_t)index.GetX() + (index.GetY() + 1l) * _xzSize.GetX()]; // z
    }
    else
        checkDiag = false;

    if (checkDiag)
    {
        float dist = float2::Magnitude(interpDelta);

        dist /= 1.41421356f;
        float diag = dist * _gridMem[(size_t)index.GetX() + 1l + (index.GetY() + 1l) * _xzSize.GetX()] + *heightOut * (1.0f - dist);
        
        *heightOut = (interp.GetX() + interp.GetY() + diag) / 3.0f; // average
    }
    else 
        *heightOut = (interp.GetX() + interp.GetY()) / 2.0f; // average

    return true;
}

inline Frustum & King::Frustum::operator*=(const DirectX::XMMATRIX & m)
{
    DirectX::XMMATRIX xForm = DirectX::XMMatrixTranspose(XMMatrixInverse(nullptr, m));

    for (int i = 0; i < 8; ++i)
        _FrustumCorners[i] = DirectX::XMVector3Transform(_FrustumCorners[i], m);

    for (int i = 0; i < 6; ++i)
        _FrustumPlanes[i] = DirectX::XMVector4Transform(_FrustumPlanes[i], xForm);
}

bool __vectorcall King::Frustum::Intersect(const Box &boxIn) const
{
    for (int i = 0; i < 6; ++i)
    {
        Plane p = _FrustumPlanes[i];
        // if normal component is (+), use max otherwise min
        FloatPoint3 mask(p.GetNormal() > FloatPoint3()); 
        FloatPoint3 farCorner = DirectX::XMVectorSelect(boxIn.GetMin(), boxIn.GetMax(), mask);
        if (p.DistanceFromPoint(farCorner) < 0.0f)
            return false;
    }
    return true;
}

bool __vectorcall King::Frustum::Intersect(const Sphere &sphereIn) const
{
    float3 c(sphereIn.GetCenter());
    float radius = sphereIn.GetRadius();
    for (int i = 0; i < 6; ++i)
    {
        if (_FrustumPlanes[i].DistanceFromPoint(c) + radius < 0.0f)
            return false;
    }
    return true;
}
/******************************************************************************
*    Method:    ConstructPerspectiveFrustum
******************************************************************************/
/*
         ltb---------rtb
           /|      _/|
          / |   _/    |
      ltf/-----/rtf |
         |lbb-|---_/rbb
         |/   | _/
      lbf------/rbf 
 */
void King::Frustum::ConstructPerspectiveFrustum(float HTan, float VTan, float NearClip, float FarClip)
{
    const float NearX = HTan * NearClip;
    const float NearY = VTan * NearClip;
    const float FarX = HTan * FarClip;
    const float FarY = VTan * FarClip;

    // Define the frustum corners
    _FrustumCorners[lbf] = FloatPoint3(-NearX, -NearY, -NearClip);    // Near lower left
    _FrustumCorners[ltf] = FloatPoint3(-NearX, NearY, -NearClip);    // Near upper left
    _FrustumCorners[rbf] = FloatPoint3(NearX, -NearY, -NearClip);    // Near lower right
    _FrustumCorners[rtf] = FloatPoint3(NearX, NearY, -NearClip);    // Near upper right
    _FrustumCorners[lbb] = FloatPoint3(-FarX, -FarY, -FarClip);    // Far lower left
    _FrustumCorners[ltb] = FloatPoint3(-FarX, FarY, -FarClip);    // Far upper left
    _FrustumCorners[rbb] = FloatPoint3(FarX, -FarY, -FarClip);    // Far lower right
    _FrustumCorners[rtb] = FloatPoint3(FarX, FarY, -FarClip);    // Far upper right

    const float NHx = 1.f / sqrtf(1.0f + HTan * HTan);
    const float NHz = -NHx * HTan;
    const float NVy = 1.f / sqrtf(1.0f + VTan * VTan);
    const float NVz = -NVy * VTan;

    // Define the bounding planes
    _FrustumPlanes[kNearPlane] = Plane(0.0f, 0.0f, -1.0f, -NearClip);
    _FrustumPlanes[kFarPlane] = Plane(0.0f, 0.0f, 1.0f, FarClip);
    _FrustumPlanes[kLeftPlane] = Plane(NHx, 0.0f, NHz, 0.0f);
    _FrustumPlanes[kRightPlane] = Plane(-NHx, 0.0f, NHz, 0.0f);
    _FrustumPlanes[kTopPlane] = Plane(0.0f, -NVy, NVz, 0.0f);
    _FrustumPlanes[kBottomPlane] = Plane(0.0f, NVy, NVz, 0.0f);
}

void King::Frustum::ConstructOrthographicFrustum(float Left, float Right, float Top, float Bottom, float Front, float Back)
{
    // Define the frustum corners
    _FrustumCorners[lbf] = FloatPoint3(Left, Bottom, -Front);    // Near lower left
    _FrustumCorners[ltf] = FloatPoint3(Left, Top, -Front);    // Near upper left
    _FrustumCorners[rbf] = FloatPoint3(Right, Bottom, -Front);    // Near lower right
    _FrustumCorners[rtf] = FloatPoint3(Right, Top, -Front);    // Near upper right
    _FrustumCorners[lbb] = FloatPoint3(Left, Bottom, -Back);    // Far lower left
    _FrustumCorners[ltb] = FloatPoint3(Left, Top, -Back);    // Far upper left
    _FrustumCorners[rbb] = FloatPoint3(Right, Bottom, -Back);    // Far lower right
    _FrustumCorners[rtb] = FloatPoint3(Right, Top, -Back);    // Far upper right

    // Define the bounding planes
    _FrustumPlanes[kNearPlane] = Plane(0.0f, 0.0f, -1.0f, -Front);
    _FrustumPlanes[kFarPlane] = Plane(0.0f, 0.0f, 1.0f, Back);
    _FrustumPlanes[kLeftPlane] = Plane(1.0f, 0.0f, 0.0f, -Left);
    _FrustumPlanes[kRightPlane] = Plane(-1.0f, 0.0f, 0.0f, Right);
    _FrustumPlanes[kTopPlane] = Plane(0.0f, -1.0f, 0.0f, Bottom);
    _FrustumPlanes[kBottomPlane] = Plane(0.0f, 1.0f, 0.0f, -Top);
}

King::Frustum::Frustum(const DirectX::XMMATRIX &projectionMatrixIn)
{
    const float* ProjMatF = (const float*)&projectionMatrixIn;

    const float RcpXX = 1.0f / ProjMatF[0];
    const float RcpYY = 1.0f / ProjMatF[5];
    const float RcpZZ = 1.0f / ProjMatF[10];

    // Identify if the projection is perspective or orthographic by looking at the 4th row.
    if (ProjMatF[3] == 0.0f && ProjMatF[7] == 0.0f && ProjMatF[11] == 0.0f && ProjMatF[15] == 1.0f)
    {
        // Orthographic
        float Left = (-1.0f - ProjMatF[12]) * RcpXX;
        float Right = (1.0f - ProjMatF[12]) * RcpXX;
        float Top = (1.0f - ProjMatF[13]) * RcpYY;
        float Bottom = (-1.0f - ProjMatF[13]) * RcpYY;
        float Front = (0.0f - ProjMatF[14]) * RcpZZ;
        float Back = (1.0f - ProjMatF[14]) * RcpZZ;

        // Check for reverse Z here.  The bounding planes need to point into the frustum.
        if (Front < Back)
            ConstructOrthographicFrustum(Left, Right, Top, Bottom, Front, Back);
        else
            ConstructOrthographicFrustum(Left, Right, Top, Bottom, Back, Front);
    }
    else
    {
        // Perspective
        float NearClip, FarClip;

        if (RcpZZ > 0.0f)    // Reverse Z
        {
            FarClip = ProjMatF[14] * RcpZZ;
            NearClip = FarClip / (RcpZZ + 1.0f);
        }
        else
        {
            NearClip = ProjMatF[14] * RcpZZ;
            FarClip = NearClip / (RcpZZ + 1.0f);
        }
        ConstructPerspectiveFrustum(RcpXX, RcpYY, NearClip, FarClip);
    }
}
/******************************************************************************
*    Method:    CalculateVertexSkinPositions
*
*    Remarks: Normally processed in the vertex shader by passing 
*        float4x4 boneFinalTransforms[numBones]
*        float4 boneWeights (weight of each bone influence)
*        uint4 boneIndices (4 maximum influences)
*        
*        Code below allows for CPU processing and variable number of influences
******************************************************************************/
std::vector<float3> King::SkinnedModel::CalculateVertexSkinPositions(size_t meshIndex)
{
    assert(meshIndex > _meshes.size() - 1);
    auto &m = _meshes[meshIndex];

    std::vector<float3> skinPositions;
    float4 pos; // position of skinned vertex, w = 1.0f
    uint8_t n; // number of bones influencing the vertex
    uint8_t bi; // absolute bone index
    DirectX::XMMATRIX boneToParentTrans; // bind pose transform of the bone
    DirectX::XMMATRIX M; // transform of the bone
    float w; // weight of the bone influence

    // identify vertex positions
    const auto vb = m.GetVB();
    assert(vb != nullptr);
    const auto ib = m.GetIB();
    const auto & ibStart = m.GetIBStart();
    const auto & vFormat = m.GetVertexFormat();
    const auto & stride = vFormat.GetByteSize();
    const auto & attrP = vFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    const auto & offset = vFormat.GetAttribute(attrP).GetOffset();
    // byte pointer to start of vertex data
    auto GetVertexAddr = [&](const uint32_t indexIn) { return vb + ((size_t) *(ib + ibStart + indexIn) * stride); };

    // copy vertex positions
    const auto numVerticies = m.GetNumVerticies();
    for (uint32_t i = 0; i < numVerticies; ++i)
    {
        skinPositions.push_back( float3(reinterpret_cast<float*>(GetVertexAddr(i) + offset)) );
    }
    // transform vertex positions
    size_t boneIndexOffset = 0; // into skin.boneIndex[]
    for (uint32_t i = 0; i < numVerticies; ++i)
    {
        pos = float4(skinPositions[i], 1.0f);
        float3 posSkinned;
        n = _boneHierarchy->_boneInfluenceCount[i]; // number of bones influencing this vertex

        // apply bone influences
        for (uint8_t b=0; b<n; ++b)
        {
            bi = _boneHierarchy->_boneIndex[boneIndexOffset + b]; // bone that influences this vertex
            w = _boneHierarchy->_boneWeight[boneIndexOffset + b]; // weight of this influence
            M = DirectX::XMLoadFloat4x4(&_boneHierarchy->_skeleton._bones[bi]._transform); // transform of bone (rotation, scale)
            boneToParentTrans = DirectX::XMLoadFloat4x4(&_boneHierarchy->_skeleton._toParentTransform[bi]); // heiarchy
            
            auto detBi = DirectX::XMMatrixDeterminant(boneToParentTrans);
            auto BiInverse = DirectX::XMMatrixInverse(&detBi, boneToParentTrans);

            auto r1 = DirectX::XMVector4Transform(pos.GetVecConst(), M); // bone orientation
            auto r2 = DirectX::XMVector4Transform(r1, BiInverse); // relative to parent

            posSkinned += w * float3(r2);
        }
        skinPositions[i] = posSkinned;
        boneIndexOffset += n;
    }
    
    return skinPositions; // compiler should involk std::move() optimization
}

inline King::Pose::Pose(const XMMATRIX & M3x3, float3 translate) : _translation(translate) { _rotation = DirectX::XMQuaternionRotationMatrix(M3x3); }
King::Pose::Pose(const DirectX::XMMATRIX &M4x4) { *this = M4x4; } // involke conversion copy assignment operator
King::Pose::Pose(const DirectX::XMFLOAT4X4 &F4x4) { *this = F4x4; } // involke conversion copy assignment operator
inline Pose & King::Pose::operator= (const DirectX::XMMATRIX &M) // conversion copy assignment
{
    assert(DirectX::XMMatrixDecompose(&_scale.GetVec(), &_rotation.GetVec(), &_translation.GetVec(), M));
    return *this;
}
inline Pose & King::Pose::operator= (const DirectX::XMFLOAT4X4 &A) // conversion copy assignment
{
    DirectX::XMMATRIX M(DirectX::XMLoadFloat4x4(&A));
    assert(DirectX::XMMatrixDecompose(&_scale.GetVec(), &_rotation.GetVec(), &_translation.GetVec(), M));
    return *this;
}
// rotate
inline Pose King::Pose::operator* (const quat &in)
{
    Pose rtn(*this);
    rtn.SetRotation(_rotation * in);
    return rtn;
}
inline Pose & King::Pose::operator*= (const quat &in)
{
    _rotation *= in;
    return *this;
}
// translate
inline Pose King::Pose::operator+ (const float3 &in)
{
    Pose rtn(*this);
    rtn.SetTranslation(_translation + in);
    return rtn;
}
inline Pose & King::Pose::operator+= (const float3 &in)
{
    _translation += in;
    return *this;
}
inline Pose King::Pose::operator- (const float3 &in)
{
    Pose rtn(*this);
    rtn.SetTranslation(_translation - in);
    return rtn;
}
inline Pose & King::Pose::operator-= (const float3 &in)
{
    _translation -= in;
    return *this;
}

inline float3 King::Pose::operator* (float3 vec) const { return _rotation * (vec * _scale) + _translation; }
inline float4 King::Pose::operator* (float4 vec) const {
    float4 rot = _rotation * (vec * float4(_scale,1.0f));
    rot.SetW(0.0f);
    float4 trans(_translation);
    trans.SetW(1.0f);
    trans *= vec.GetW();
    return rot + trans;
}
inline Pose King::Pose::operator~ () const {
    Quaternion invertedRotation = ~_rotation;
    return Pose(invertedRotation, invertedRotation * - _translation);
}

// Functionality
Pose King::Pose::Interpolate(const Pose &in, const float fractionIn) const
{
    Pose rtn;
    rtn._rotation = DirectX::XMQuaternionSlerp(_rotation, in._rotation, fractionIn);
    rtn._scale = DirectX::XMVectorLerp(_scale, in._scale, fractionIn);
    rtn._translation = DirectX::XMVectorLerp(_translation, in._translation, fractionIn);
    return rtn;
}

/******************************************************************************
*    Method:    GetDataAttribute
*       Strips out the vertex composite data to just one data set for the given
*       attribute.
*   INPUTS:
*       attributeEnum       attribute data type to retrieve
*       vbStartIn           vertex offset if master is assembled without index modification
*       ibStartIn           index offset if indicies are shifted for a subset
******************************************************************************/
vector<vector<float>> King::ModelScaffold::GetDataAttribute(VertexAttrib::enumDesc attributeEnum, uint32_t vbStartIn, uint32_t ibStartIn)
{
    assert(_vertexFormat.Has(attributeEnum));

    vector<vector<float>> v;
    auto & vb = GetVertexBufferMaster(); // uint8_t but with stride indexing
    uint8_t * vbAddr = &vb.GetData();
    auto vbStride = vb.GetStride();
    const auto & ib = GetIndexBufferMaster(); // uint32_t
    const auto attribIndex = _vertexFormat.GetAttributeIndexFromDescription(attributeEnum);
    const auto atrribute = _vertexFormat.GetAttribute(attribIndex);
    const auto attribOffset = atrribute.GetOffset();
    const auto atrribFormat = atrribute.GetFormat();

    // cycle through indicies and store vertex info    
    const auto indexCount = GetIndexCount();
    const auto vertexCount = GetVertexCount();

    if (atrribFormat == VertexAttrib::enumFormat::format_float32x4)
    {
        for (uint32_t i = 0; i < indexCount; ++i)
        {
            assert(i < vertexCount);
            float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
            vector<float> xyzw(4);
            xyzw[0] = *v_data;
            xyzw[1] = *(v_data + 1);
            xyzw[2] = *(v_data + 2);
            xyzw[3] = *(v_data + 3);
            v.push_back(xyzw);
        }
    }
    else if (atrribFormat == VertexAttrib::enumFormat::format_float32x3)
    {
        for (uint32_t i = 0; i < indexCount; ++i)
        {
            float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
            vector<float> xyz(3); // xyz
            xyz[0] = *v_data;
            xyz[1] = *(v_data + 1);
            xyz[2] = *(v_data + 2);
            v.push_back(xyz);
        }
    }
    else if (atrribFormat == VertexAttrib::enumFormat::format_float32x2)
    {
        for (uint32_t i = 0; i < indexCount; ++i)
        {
            float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
            vector<float> xy(2);
            xy[0] = *v_data;
            xy[1] = *(v_data + 1);
            v.push_back(xy);
        }
    }
    else if (atrribFormat == VertexAttrib::enumFormat::format_float32)
    {
        for (uint32_t i = 0; i < indexCount; ++i)
        {
            float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
            vector<float> x(1);
            x[0] = *v_data;
            v.push_back(x);
        }
    }

    return v;
}

inline bool King::Ray::Collision(Sphere const & sphereIn) const { return sphereIn.Collision(*this); }

inline bool King::Ray::Collision(Box const & boxIn) const { return boxIn.Collision(*this); }

inline bool __vectorcall King::Plane::Intersects(const Point & pointIn) const
{
    float dist = fabs(DistanceFromPoint(pointIn));
    if(dist < 5.e-5f)
        return true;
    else 
        return false;
}

inline bool __vectorcall King::Plane::Intersects(const Sphere & sphereIn) const
{
    float4 center(sphereIn.GetCenter(), 1.0f);
    float4 radius(sphereIn.GetRadius());
    float4 dist(XMVector4Dot(center, *this));

    float4 greater( XMVectorGreater(dist, radius) );
    float4 less( XMVectorLess(dist, -radius) );
    // in front of plane
    if (XMVector4EqualInt(greater, XMVectorTrueInt()))
        return false;
    // in back of plane
    if (XMVector4EqualInt(less, XMVectorTrueInt()))
        return false;
    // intersecting plane
    return true;
}

inline FloatPoint3 __vectorcall King::Plane::FindNearestPointOnPlane(const FloatPoint3 & pointIn) const
{ 
    float dist = DistanceFromPoint(pointIn);
    FloatPoint3 pt( -GetNormal() * dist );

    return pt;
}
