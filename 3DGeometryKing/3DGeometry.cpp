﻿#include "..\..\MathSIMD\MathSIMD.h"
#include "..\2DGeometryKing\2DGeometry.h"
#include "..\3DGeometryKing\3DGeometry.h"
#include "..\3DGeometryKing\Model_IO.h"

using namespace King;
using namespace std;
using namespace DirectX;

// global constants
extern const float3 g_boxCorners[8] =
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

size_t ModelScaffold::s_modelNameCounter = 0;

//namespace King {
    std::vector<std::string> King::VertexAttrib::description = { "Position", "Color", "TextureCoord" , "Normal" , "Tangent" , "Bi-Tangent" };
//}
/******************************************************************************
*    Method:    operator<< operator>>
******************************************************************************/
std::ostream& King::operator<< (std::ostream& os, const King::Sphere& in)
{
    return os << "{ center" << in.GetCenter() << " radius" << in.GetRadius() << " }"; // text out
}
std::istream& King::operator>> (std::istream& is, King::Sphere& out)
{
    float3 center;
    float radius;
    is >> center >> radius; // binary in
    out.SetCenter(center);
    out.SetRadius(radius);
    
    return is;
}
std::wostream& King::operator<< (std::wostream& os, const King::Sphere& in)
{
    return os << L"{ center" << in.GetCenter() << L" radius" << in.GetRadius() << L" }"; // text out
}
    std::ostream & King::operator<< (std::ostream &os, const King::Box &in)
{
    return os << "{ center" << in.GetCenter() << " pt_min" << in.GetMin() << " pt_max" << in.GetMax() << " }"; // text out
}
std::istream & King::operator>> (std::istream &is, King::Box &out)
{
    return is >> out.pt_min >> out.pt_max; // binary in
}
std::wostream& King::operator<< (std::wostream& os, const King::Box& in)
{
    return os << L"{ center" << in.GetCenter() << L" pt_min" << in.GetMin() << L" pt_max" << in.GetMax() << L" }"; // text out
}
std::wistream& King::operator>> (std::wistream& is, King::Box& out)
{
    return is >> out.pt_min >> out.pt_max; // binary in
}

std::ostream& King::operator<<(std::ostream& os, const King::Frustum& in)
{
    //os << "{ corners{";
    //for (unsigned int i = 0; i < 8; ++i)
    //    os << " [" << i << "]:" << in._FrustumCorners[i] << os << "} ";
    os << "planes{";
    for (unsigned int i = 0; i < 6; ++i)
        os << " [" << i << "]:" << in._FrustumPlanes[i];
    os << "} }";

    return os;
}

std::istream& King::operator>>(std::istream& is, King::Frustum& out)
{
    //for (unsigned int i = 0; i < 8; ++i)
    //    is >> out._FrustumCorners[i];
    for (unsigned int i = 0; i < 6; ++i)
        is >> out._FrustumPlanes[i];
    return is; // binary in
}

std::ostream & King::operator<< (std::ostream &os, const King::Pose &in)
{
    return os << "{ S" << in.GetScale() << " Q" << in.GetRotation() << " O" << in.GetRotationOrigin() << " T" << in.GetTranslation() << " }"; // text out
}
std::wostream & King::operator<< (std::wostream &os, const King::Pose &in)
{
    return os << L"{ S" << in.GetScale() << L" Q" << in.GetRotation() << L" O" << in.GetRotationOrigin() << L" T" << in.GetTranslation() << L" }"; // text out
}

std::istream & King::operator>> (std::istream &is, King::Pose &out)
{
    return is >> out._scale >> out._rotation >> out._translation; // binary in
}
std::wistream & King::operator>> (std::wistream &is, King::Pose &out)
{
    return is >> out._scale >> out._rotation >> out._translation; // binary in
}

std::ostream& King::operator<<(std::ostream& os, const King::Path& in) 
{ 
    os << "{ "; 
    for (const auto& pt : in) 
    { 
        os << pt << "\n"; 
    } 
    os << " }"; 
    return os; 
}

// *** TO DO *** when adding new functions, if you are missing a lower level,it throws linker errors.  Not sure what we are missing below for Point.  Ray was missing Point. 
//std::ostream& King::operator<< (std::ostream& os, const King::Point& in)
//{
//    return os << in.Get_position(); // text out
//}
//std::wostream& King::operator<< (std::wostream& os, const King::Point& in)
//{
//    return os << in.Get_position(); // text out
//}
//std::istream& King::operator>> (std::istream& is, King::Point& out)
//{
//    return is >> out._position; // binary in
//}
//std::wistream& King::operator>> (std::wistream& is, King::Point& out)
//{
//    return is >> out._position; // binary in
//}
//
//std::ostream& King::operator<< (std::ostream& os, const King::Ray& in)
//{
//    return os << "{ origin" << in.GetOrigin() << " dir" << in.GetDirection() << " }"; // text out
//}
//std::wostream& King::operator<< (std::wostream& os, const King::Ray& in)
//{
//    return os << L"{ origin" << in.GetOrigin() << L" dir" << in.GetDirection() << L" }"; // text out
//}
//std::istream& King::operator>> (std::istream& is, King::Ray& out)
//{
//    return is >> out.origin >> out.direction; // binary in
//}
//std::wistream& King::operator>> (std::wistream& is, King::Ray& out)
//{
//    return is >> out.origin >> out.direction; // binary in
//}
/******************************************************************************
*    json
******************************************************************************/
//void King::to_json(json& j, const King::Point& from) { King::to_json(j, static_cast<Position>(from)); }
//void King::from_json(const json& j, King::Point& to) { j.at("Pos").get_to(to.Get_position()); }
//
//void King::to_json(json& j, const King::Ray& from) { j = json{ {"origin", from.origin}, {"dir", from.direction} }; }
//void King::from_json(const json& j, King::Ray& to) { j.at("origin").get_to(to.origin); j.at("dir").get_to(to.direction); }

void King::to_json(json& j, const Pose& from) { j = json{ { "S", from._scale }, { "Q", from._rotation }, {"O", from._rotationOrigin }, { "T", from._translation } }; }
void King::from_json(const json& j, Pose& to) { j.at("S").get_to(to._scale); j.at("Q").get_to(to._rotation); j.at("O").get_to(to._rotationOrigin); j.at("T").get_to(to._translation); }

void King::to_json(json& j, const Sphere& from) { j = json{ {"C", from.GetCenter()}, {"r", from.GetRadius()} }; }
void King::from_json(const json& j, Sphere& to) { float3 c; float r; j.at("C").get_to(c); j.at("r").get_to(r); to.Set(c,r); }

void King::to_json(json& j, const Box & from) { j = json{ {"pt_min", from.GetMin()}, {"pt_max", from.GetMax()} }; }
void King::from_json(const json& j, Box& to) { j.at("pt_min").get_to(to.pt_min); j.at("pt_max").get_to(to.pt_max); }

void King::to_json(json& j, const Frustum& from) { j = json{ {"C0", from._FrustumCorners[0]}, {"C1", from._FrustumCorners[1]}, {"C2", from._FrustumCorners[2]}, {"C3", from._FrustumCorners[3]}, {"C4", from._FrustumCorners[4]}, {"C5", from._FrustumCorners[5]}, {"C6", from._FrustumCorners[6]}, {"C7", from._FrustumCorners[7]}, 
{"P0", from._FrustumPlanes[0]}, {"P1", from._FrustumPlanes[1]}, {"P2", from._FrustumPlanes[2]}, {"P3", from._FrustumPlanes[3]}, {"P4", from._FrustumPlanes[4]}, {"P5", from._FrustumPlanes[5]} }; }
void King::from_json(const json& j, Frustum& to) 
{ 
    j.at("C0").get_to(to._FrustumCorners[0]); 
    j.at("C1").get_to(to._FrustumCorners[1]); 
    j.at("C2").get_to(to._FrustumCorners[2]); 
    j.at("C3").get_to(to._FrustumCorners[3]); 
    j.at("C4").get_to(to._FrustumCorners[4]); 
    j.at("C5").get_to(to._FrustumCorners[5]); 
    j.at("C6").get_to(to._FrustumCorners[6]); 
    j.at("C7").get_to(to._FrustumCorners[7]);
    j.at("P0").get_to(to._FrustumPlanes[0]);
    j.at("P1").get_to(to._FrustumPlanes[1]);
    j.at("P2").get_to(to._FrustumPlanes[2]);
    j.at("P3").get_to(to._FrustumPlanes[3]);
    j.at("P4").get_to(to._FrustumPlanes[4]);
    j.at("P5").get_to(to._FrustumPlanes[5]);
}

bool __vectorcall King::Triangle::Contains(const Point& ptIn) const 
{
    const float epsilon = 1e-6f;

    // Get the vertices of the triangle
    float3 v0 = GetVertex(0);
    float3 v1 = GetVertex(1);
    float3 v2 = GetVertex(2);

    // Vector from point to triangle vertices
    float3 v0p = ptIn - v0;
    float3 v1p = ptIn - v1;
    float3 v2p = ptIn - v2;

    // Cross products to determine areas
    float3 cross0 = Cross(v1 - v0, v0p);
    float3 cross1 = Cross(v2 - v1, v1p);
    float3 cross2 = Cross(v0 - v2, v2p);

    // Check if the point is on the same side of all edges
    if (Dot(cross0, cross1) < -epsilon || Dot(cross1, cross2) < -epsilon || Dot(cross2, cross0) < -epsilon) {
        return false; // Point is outside the triangle
    }

    // Optional: Check if the point is exactly on the triangle's edges
    //if (std::abs(Dot(cross0, cross1)) < epsilon || std::abs(Dot(cross1, cross2)) < epsilon || std::abs(Dot(cross2, cross0)) < epsilon) {
    //    return true; // Point is on the edge
    //}

    return true; // Point is inside the triangle
}

/******************************************************************************
*    Triangle Intersects
*        Desc:       ray intersection test
*        Input:      ray to test against (with origin and direction)
*        Output:     float3 containing the point of intersection
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
bool King::Triangle::Intersects(const float3 &rayOriginIn, const float3 &rayDirectionIn, float3 *intersectPointOut) const
{
    King::float3 hv, sv, qv;
    float a, f, u, v;

    auto edge2 = pt[2] - pt[0];
    auto edge1 = pt[1] - pt[0];

    hv = King::float3::CrossProduct(rayDirectionIn, edge1); // result is an orthogonal vector
    a = King::float3::DotProduct(edge2, hv).GetX(); // project other edge onto the orthogonal vector
    if (a > -FLT_EPSILON && a < FLT_EPSILON) // if projection is zero, then these vectors are also orthogonal, so the plane is parallel to the ray
        return false;
    f = 1 / a;
    sv = rayOriginIn - pt[0]; // ray relative to triangle
    u = f * King::float3::DotProduct(sv, hv).GetX();
    if (u < 0.0f || u > 1.0f)
        return false;
    qv = sv.CrossProduct(edge2);
    v = f * King::float3::DotProduct(rayDirectionIn, qv).GetX();
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

bool King::TriangleIndexed::Intersects(const float3 &rayOriginIn, const float3 &rayDirectionIn, float3 *intersectPointOut)
{
    assert(intersectPointOut);
    King::float3 hv, sv, qv;
    float a, f, u, v;

    auto edge2 = GetVertexPosition(2) - GetVertexPosition(0);
    auto edge1 = GetVertexPosition(1) - GetVertexPosition(0);

    hv = King::float3::CrossProduct(rayDirectionIn, edge1); // result is an orthogonal vector
    a = King::float3::DotProduct(edge2, hv).GetX(); // project other edge onto the orthogonal vector
    if (a > -FLT_EPSILON && a < FLT_EPSILON) // if projection is zero, then these vectors are also orthogonal, so the plane is parallel to the ray
        return false;
    f = 1 / a;
    sv = rayOriginIn - GetVertex(0); // ray relative to triangle
    u = f * King::float3::DotProduct(sv, hv).GetX();
    if (u < 0.0f || u > 1.0f)
        return false;
    qv = sv.CrossProduct(edge2);
    v = f * King::float3::DotProduct(rayDirectionIn, qv).GetX();
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
inline float3 King::LineIndexed::GetVertexPosition(const uint32_t indexIn) const
{
    assert(vb != nullptr);
    auto attrI = vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = vertexFormat.GetAttribute(attrI).GetOffset();
    return float3(reinterpret_cast<float*>((size_t)vb + (size_t)pt[indexIn] * vertexFormat.GetByteSize() + offset));
}
inline float3 King::TriangleIndexed::GetVertexPosition(const uint32_t indexIn) const
{
    assert(vb != nullptr);
    auto attrI = vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = vertexFormat.GetAttribute(attrI).GetOffset();
    return float3(reinterpret_cast<float*>((size_t)vb + (size_t)pt[indexIn] * vertexFormat.GetByteSize() + offset));
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
        // highest index referenced
        numVerticies = lambdaMax(*(ib+i), numVerticies);
    }
    return ni > 0 ? numVerticies+1 : 0; // indexing is zero based, size is therefore +1
}
inline float3 King::LineMesh::GetVertexPosition(const uint32_t vertNumIn) const
{
    assert(_vb != nullptr);
    auto attrI = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = _vertexFormat.GetAttribute(attrI).GetOffset();
    //auto n = GetNumVerticies();
    //assert(vertNumIn >= n);
    return float3(reinterpret_cast<float*>(GetVertexAddr(vertNumIn) + offset));
}
inline float3 King::TriangleMesh::GetVertexPosition(const uint32_t vertexNum) const
{
    assert(_vb != nullptr);
    auto attrI = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = _vertexFormat.GetAttribute(attrI).GetOffset();
    return float3(reinterpret_cast<float*>(GetVertexAddr(vertexNum) + offset));
}

void King::TriangleMesh::SetVertexPosition(const uint32_t indexIn, float3 posIn)
{
    assert(_vb != nullptr);
    auto attrI = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    auto offset = _vertexFormat.GetAttribute(attrI).GetOffset();
    
    auto addr = (reinterpret_cast<float*>(GetVertexAddr(indexIn) + offset));
    *addr = posIn.GetX();
    *(addr + 1) = posIn.GetY();
    *(addr + 2) = posIn.GetZ();
}
/******************************************************************************
    HelperMoveVertexOnXZGrid
        Performs two actions on a vertex on a regular grid of wxh verticies:
        1) Translate the vertex by delta
        2) Re-arrange the index orders so that the quad seems point to the target vertex
******************************************************************************/
void King::TriangleMesh::HelperMoveVertexOnGrid(const float3 delta, const unsigned int targetVertexNum, const unsigned int w, const unsigned int h)
{
    auto v = GetVertexPosition(targetVertexNum);
    v += delta;
    SetVertexPosition(targetVertexNum, v);
    /*
    Q0   Q1
    0--1--2
    |\ | /|
    | \|/ |
    3--*--4
    | /|\ |
    |/ | \|
    5--6--7
    Q2   Q3

    Q0 = (targetVertexNum / 4 - 1) * 6; // 6 indicies per quad w/quad0 had vertex#4 and 6 indicies (0-5). So vertex#4 for quad0 is (4/4 - 1) * 6 = 0; for vertex #8 for quad1 is (8/4 - 1) * 6 = 6
    Q1 = Q0 + 6;
    Q2 = ((targetVertexNum + w) / 4 - 1) * 6;
    Q3 = Q2 + 6;
    */
    bool q0, q1, q2, q3;
    // parse in j direction
    if (targetVertexNum > (h * w - 1) || targetVertexNum < (w + 1))
    {
        // out of bounds
        q0 = q1 = q2 = q3 = false;
    }
    else if (targetVertexNum < w)
    {
        // top row
        q0 = q1 = false;
        q2 = q3 = true;
    }
    else if (targetVertexNum > ((h - 1) * w - 1))
    {
        // bottom row
        q0 = q1 = true;
        q2 = q3 = false;
    }
    else
    {
        // on the grid from row 1 to h - 1
        q0 = q1 = q2 = q3 = true;
    }

    // parse in i direction
    auto wTest = targetVertexNum % w;
    if (!wTest)
    {
        // left edge
        q0 = q2 = false;
    }
    else if (wTest == w - 1)
    {
        // right edge
        q1 = q3 = false;
    }

    auto vn1 = targetVertexNum - w;
    auto vn0 = vn1 - 1;
    auto vn2 = vn1 + 1;
    auto vn3 = targetVertexNum - 1;
    auto vn4 = targetVertexNum + 1;
    auto vn6 = targetVertexNum + w;
    auto vn5 = vn6 - 1;
    auto vn7 = vn6 + 1;

    auto ib = GetIB();
    if (q0)
    {
        /*
            0--1 Q0
            |\ |
            | \|
            3--*
        */
        auto Q0 = ((targetVertexNum - w - 1)) * 6;
        cout << "  IB[" << Q0 << "] for Q0 = " << ib[Q0 + 0];
        cout << ", " << ib[Q0 + 1];
        cout << ", " << ib[Q0 + 2];
        cout << ", " << ib[Q0 + 3];
        cout << ", " << ib[Q0 + 4];
        cout << ", " << ib[Q0 + 5] << '\n';
        ib[Q0 + 0] = vn0;
        ib[Q0 + 1] = vn3;
        ib[Q0 + 2] = targetVertexNum;

        ib[Q0 + 3] = vn0;
        ib[Q0 + 4] = targetVertexNum;
        ib[Q0 + 5] = vn1;

        cout << "  IB[" << Q0 << "] for Q0 = " << ib[Q0 + 0];
        cout << ", " << ib[Q0 + 1];
        cout << ", " << ib[Q0 + 2];
        cout << ", " << ib[Q0 + 3];
        cout << ", " << ib[Q0 + 4];
        cout << ", " << ib[Q0 + 5] << '\n';

    }
    if (q1)
    {
        /*
            1--2 Q1
            | /|
            |/ |
            *--4
        */
        auto Q1 = ((targetVertexNum - w)) * 6;
        ib[Q1 + 0] = vn1;
        ib[Q1 + 1] = targetVertexNum;
        ib[Q1 + 2] = vn2;

        ib[Q1 + 3] = vn2;
        ib[Q1 + 4] = targetVertexNum;
        ib[Q1 + 5] = vn4;
    }
    if (q2)
    {
        /*
            3--*
            | /|
            |/ |
            5--6 Q2
        */
        auto Q2 = ((targetVertexNum - 2)) * 6;
        ib[Q2 + 0] = vn3;
        ib[Q2 + 1] = vn5;
        ib[Q2 + 2] = targetVertexNum;

        ib[Q2 + 3] = targetVertexNum;
        ib[Q2 + 4] = vn5;
        ib[Q2 + 5] = vn6;
    }
    if (q3)
    {
        /*
            *--4
            |\ |
            | \|
            6--7 Q3
        */
        auto Q3 = ((targetVertexNum - 1)) * 6;
        ib[Q3 + 0] = targetVertexNum;
        ib[Q3 + 1] = vn6;
        ib[Q3 + 2] = vn7;

        ib[Q3 + 3] = vn7;
        ib[Q3 + 4] = vn4;
        ib[Q3 + 5] = targetVertexNum;
    }
}

inline bool King::Point::Intersects(const float3 &pointIn) const
{
    float3 diff = pointIn - static_cast<float3>(*this);
    diff.Absolute();

    return float3::SumComponents(diff) < FLT_EPSILON ? true : false;
}

inline bool King::Point::Intersects(const Point & pointIn) const
{
    float3 diff(pointIn - *this);
    diff.Absolute();

    return float3::SumComponents(diff) < FLT_EPSILON ? true : false;
}

inline bool King::Point::Collision(Point const & pointIn) const { return Intersects(pointIn); }

inline bool King::Point::Collision(Ray const& rayIn) const { return rayIn.Collision(*this); }

inline bool King::Point::Collision(Line const & lineIn) const { return lineIn.Intersects(*this); }

inline bool King::Point::Collision(Sphere const & sphereIn) const { return sphereIn.Contains(*this); }

inline bool King::Point::Collision(Box const & boxIn) const { return boxIn.Contains(*this); }

inline bool King::Point::Collision(Plane const& planeIn) const { return planeIn.Collision(*this); }

inline bool King::Point::Collision(Capsule const& capsuleIn) const { return capsuleIn.Collision(*this); }

inline bool King::Point::Collision(Pyramid const& pyramidIn) const { return false; } // { return pyramidIn.Collision(*this); }


// double dispatch

inline bool King::Ray::Collision(Point const& pointIn) const { return Intersects(pointIn); }

inline bool King::Ray::Collision(Ray const& rayIn) const { Position ptOut; return Intersects(rayIn, &ptOut); }

bool King::Ray::Collision(Line const& lineIn) const { Position ptOut; return Intersects(lineIn, &ptOut); }

inline bool King::Ray::Collision(Plane const& planeIn) const { Point ptOut; return planeIn.Intersects(*this, &ptOut); }

inline bool King::Ray::Collision(Sphere const& sphereIn) const { return sphereIn.Collision(*this); }

inline bool King::Ray::Collision(Capsule const& capsuleIn) const { return capsuleIn.Collision(*this); }

inline bool King::Ray::Collision(Box const& boxIn) const { return boxIn.Collision(*this); }

inline bool King::Ray::Collision(Pyramid const& pyramidIn) const { return false; }// { return pyramidIn.Collision(*this); }

bool King::Ray::Intersects(const Point& pointIn) const
{
    auto pt = FindNearestPoint(pointIn);
    float3 diff(pointIn - pt);
    diff.Absolute();

    return float3::SumComponents(diff) < FLT_EPSILON ? true : false;
}

bool King::Ray::Intersects(const Ray& rayIn, Position* intersectionOut) const
{
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    const float3& rayA = GetDirection();
    const float3& rayB = rayIn.GetDirection();

    float3 pointOnSegA, pointOnSegB;

    // this is the same as Dot(rayA,rayA) and Dot(rayB,rayB)
    float testL11 = Dot(rayA, rayA);
    float L11 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayA, float3(2.f, 2.f, 2.f))));
    assert(testL11 == L11);
    float L22 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayB, float3(2.f, 2.f, 2.f))));
    float testL22 = Dot(rayB, rayB);
    assert(testL22 == L22);
    cout << testL11 << '\n';
    cout << testL22 << '\n';

    float3 AB = rayIn.GetOrigin() - GetOrigin();
    // - dot product of A and B
    float L12 = -float3::SumComponents(rayA * rayB);
    float DetL = L11 * L22 - L12 * L12;

    // Lines/Segments A and B are parallel
    if (fabsf(DetL) < FLT_EPSILON)
    {
        return false;
    }
    else
    {
        float ra = float3::SumComponents(rayA * AB);
        float rb = -float3::SumComponents(rayB * AB);

        float t = (L11 * rb - ra * L12) / DetL;
        float s = (ra - L12 * t) / L11;

        pointOnSegA = rayA * s + GetOrigin();
        pointOnSegB = rayB * t + rayIn.GetOrigin();
    }

    // point of intersection is the midpoint of the two
    *intersectionOut = (pointOnSegA + pointOnSegB) * 0.5f;

    float3 nearestVector = pointOnSegB - pointOnSegA;
    nearestVector.Absolute();

    return float3::SumComponents(nearestVector) < FLT_EPSILON ? true : false;
}

bool King::Ray::Intersects(const Line& lineIn, Position* intersectionOut) const
{
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    const float3& rayA = GetDirection();
    float3 rayB = lineIn.pt[1] - lineIn.pt[0];

    float3 pointOnSegA, pointOnSegB;

    // line segment is degenerate (distance between end points is near zero)
    // this is the same as Dot(rayA,rayA) and Dot(rayB,rayB)
    float L11 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayA, float3(2.f, 2.f, 2.f))));
    float L22 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayB, float3(2.f, 2.f, 2.f))));

    if (L22 < epsilon_squared)
    {
        pointOnSegA = FindNearestPoint(lineIn.pt[0]);
    }
    else
    {
        float3 AB = lineIn.pt[0] - GetOrigin();
        // - dot product of A and B
        float L12 = -float3::SumComponents(rayA * rayB);
        float DetL = L11 * L22 - L12 * L12;

        // Lines/Segments A and B are parallel
        if (fabsf(DetL) < FLT_EPSILON)
        {
            return false;
        }
        // The general case
        else
        {
            // AB is the distance between the origin of the ray and line
            float ra = float3::SumComponents(rayA * AB);
            float rb = -float3::SumComponents(rayB * AB);

            float t = (L11 * rb - ra * L12) / DetL;
            float s = (ra - L12 * t) / L11;

            pointOnSegA = rayA * s + GetOrigin();
            pointOnSegB = rayB * t + lineIn.pt[0];
        }

    }
    // point of intersection is the midpoint of the two
    if (intersectionOut != nullptr)
        *intersectionOut = (pointOnSegA + pointOnSegB) * 0.5f;

    float3 nearestVector = pointOnSegB - pointOnSegA;
    nearestVector.Absolute();

    return float3::SumComponents(nearestVector) < FLT_EPSILON ? true : false;
}

// Based on Möller–Trumbore intersection algorithm
inline bool __vectorcall Ray::Intersects(const Triangle& triangleIn, float3* intersectionOut) const {
    const float EPSILON = 1.0e-6f;
    
    // Get the vertices of the triangle
    float3 v0 = triangleIn.GetVertex(0);
    float3 v1 = triangleIn.GetVertex(1);
    float3 v2 = triangleIn.GetVertex(2);

    // Calculate the two edges of the triangle
    float3 edge1 = v1 - v0;
    float3 edge2 = v2 - v0;

    // Calculate the determinant (cross product of direction and edge2)
    float3 h = Cross(direction, edge2);
    float a = Dot(edge1, h);

    // If a is close to 0, the ray is parallel to the triangle (no intersection)
    if (fabs(a) < EPSILON) {
        return false;
    }

    // Calculate the inverse determinant
    float f = 1.0f / a;

    // Calculate the distance from v0 to the ray's origin
    float3 s = origin - v0;

    // Calculate the first barycentric coordinate
    float u = f * Dot(s, h);
    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    // Calculate the second barycentric coordinate
    float3 q = Cross(s, edge1);
    float v = f * Dot(direction, q);
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }

    // Calculate the distance t to the intersection point
    float t = f * Dot(edge2, q);

    // If t is positive, the ray intersects the triangle
    if (t > EPSILON) {
        // If intersectionOut is provided, calculate the intersection point
        if (intersectionOut != nullptr) {
            *intersectionOut = origin + direction * t;
        }
        return true;
    }

    // No valid intersection
    return false;
}


inline bool __vectorcall Ray::Intersects(const Quad& quadIn, float3* intersectionOut) const {
    // Extract the two triangles that form the quad
    Triangle tri1 = quadIn.GetTriangle1();
    Triangle tri2 = quadIn.GetTriangle2();

    // Check intersection with the first triangle
    if (this->Intersects(tri1, intersectionOut)) {
        return true; // If intersection occurs with the first triangle, return true
    }

    // Check intersection with the second triangle
    if (this->Intersects(tri2, intersectionOut)) {
        return true; // If intersection occurs with the second triangle, return true
    }

    // No intersection with either triangle
    return false;
}

/******************************************************************************
*    Ray::FindNearestPoint
*        Desc:       Given a ray and a point in 3-dimensional space,
*                    find the point on the ray that is closest to the
*                    point.
*        Input:      a point in 3-dimensional space
*        Output:     nearest point on ray to input point
*        Returns:    none
*        Remarks:    
******************************************************************************/
King::float3 King::Ray::FindNearestPoint(const Point& ptIn) const
{
    const float3& dir = GetDirection();
    const float3& origin = GetOrigin();

    float3 l(ptIn - GetOrigin());

    // project l onto our ray, direction must be normalized
    float3 projection = Dot(l, dir) * dir;

    return projection;
}
/******************************************************************************
*    Method:    Line::operator* Distance
*       Quad q = l * d;
******************************************************************************/
King::Quad King::Line::operator* (const King::Distance& rhs) const
{ 
    King::Line edge4(pt[0], pt[0] + rhs); 
    King::Line edge2(pt[1], pt[1] + rhs); 
    return King::Quad(edge4, edge2); 
}

void King::Line::Offset(const float distIn, Plane offsetPlaneIn)
{
    float3 dir;

    auto normalUnitVector = - GetPerpendicular(offsetPlaneIn);

    auto dist = distIn * normalUnitVector;

    SetVertex(0, GetVertex(0) + dist);
    SetVertex(1, GetVertex(1) + dist);

    // note floating point error
}

/******************************************************************************
*    Line::Intersects
*        Desc:       Find the nearest point between two finite length line segments
*        Input:      a line in 3-dimensional space
*        Output:     true if within tolerance, false otherwise
*                    float3 *intersectOut is point of intersection
*        Returns:    none
*        Remarks:    Adapted from:
|                         Book Title: Game Programming Gems II
|                         Chapter Title: Fast, Robust Intersection of 3D Line Segments
|                         Author: Graham Rhodes
|                         Revisions: 05-Apr-2001 - GSR. Original.
******************************************************************************/
inline bool __vectorcall King::Line::Intersects(const Point& pointIn) const
{
    auto nPt = FindNearestPointOnLineSegment(pointIn);

    float3 diff = pointIn - nPt;
    diff.Absolute();

    return float3::SumComponents(diff) < FLT_EPSILON ? true : false;
}

inline bool __vectorcall King::Line::Intersects(const Line & lineIn, float3 *intersectionOut) const
{
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    float3 rayA = pt[1] - pt[0];
    float3 rayB = lineIn.pt[1] - lineIn.pt[0];

    float3 pointOnSegA, pointOnSegB;

    // line segment is degenerate (distance between end points is near zero)
    // this is the same as Dot(rayA,rayA) and Dot(rayB,rayB)
    float L11 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayA, float3( 2.f, 2.f ,2.f ))));
    float L22 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayB, float3( 2.f, 2.f, 2.f ))));

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
        float3 AB = lineIn.pt[0] - pt[0];
        // - dot product of A and B
        float L12 = -float3::SumComponents(rayA * rayB);
        float DetL = L11 * L22 - L12 * L12;

        // Lines/Segments A and B are parallel
        if (fabsf(DetL) < FLT_EPSILON)
        {
            return false;
        }
        // The general case
        else
        {
            float ra = float3::SumComponents(rayA * AB);
            float rb = -float3::SumComponents(rayB * AB);

            float t = (L11 * rb - ra * L12) / DetL;
            float s = (ra - L12 * t) / L11;

            pointOnSegA = rayA * s + pt[0];
            pointOnSegB = rayB * t + lineIn.pt[0];
        }
    
    }
    // point of intersection is the midpoint of the two
    if(intersectionOut != nullptr)
        *intersectionOut = (pointOnSegA + pointOnSegB) * 0.5f;

    float3 nearestVector = pointOnSegB - pointOnSegA;
    nearestVector.Absolute();

    return float3::SumComponents(nearestVector) < FLT_EPSILON ? true : false;
}
/******************************************************************************
*    Line::FindNearestPointOnLineSegment
*        Desc:       Given a line (segment) and a point in 3-dimensional space,
*                    find the point on the line (segment) that is closest to the
*                    point.
*        Input:      a point in 3-dimensional space
*        Output:     nearest point on line to input point
*        Returns:    none
*        Remarks:    Adapted from:
*                         Book Title: Game Programming Gems II
*                         Chapter Title: Fast, Robust Intersection of 3D Line Segments
*                         Author: Graham Rhodes
*                         Revisions: 05-Apr-2001 - GSR. Original.
******************************************************************************/
King::float3 King::Line::FindNearestPointOnLineSegment(const float3 & pointIn) const
{
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    float3 ray = pt[1] - pt[0];

    // Line/Segment is degenerate (distance between them is near zero)
    float d = float3::SumComponents(DirectX::XMVectorPow(ray, { 2.f,2.f ,2.f ,2.f }));
    if (d < epsilon_squared)
        return pt[0];

    float3 ab = pointIn - pt[0];

    // parametric coordinate of the nearest point along the line (infinite in direction of ray)
    float parameter = float3::SumComponents(ray * ab) / d;
    parameter = std::fmaxf(0.0f, std::fminf(1.0f, parameter)); // limit to the line segment (with range 0.0 to 1.0)

    float3 nearestPoint = ray * parameter + pt[0];
    
    return nearestPoint;
}
/******************************************************************************
*    Method:    Line::GetPerpendicular
*       Note: my original code.  Wow, this was hard!
******************************************************************************/
float3 King::Line::GetPerpendicular() const
{
    // any perpendicular vector to the line rotated about the line remains perpendicular (infinite solutions)
    // we chose a solution that minimizes the component that is the smallest and maximizes the largest
    float3 dir(pt[1] - pt[0]);
    XMFLOAT3 d = dir;
    dir.Absolute();
    XMFLOAT3 a = dir;
    // Ax + By + Cz = 0 ; from the dot product

    // test
    //d = a;
    if (a.z > FLT_EPSILON)
    {
        float m = max(a.x, a.y);
        if (m < FLT_EPSILON)
            dir.Set(1.0f, 1.0f, -(d.x + d.y) / d.z);
        else
            dir.Set(a.x / m, a.y / m, -(d.x + d.y) / d.z);
    }
    else
    {
        if (a.x > a.y)
        {
            float m = max(a.z, a.y);
            if (m < FLT_EPSILON)
                dir.Set(-(d.z + d.y) / d.x, 1.0f, 1.0f);
            else
                dir.Set(-(d.z + d.y) / d.x, a.y / m, a.z / m);
        }
        else
        {
            float m = max(a.x, a.z);
            if (m < FLT_EPSILON)
                dir.Set((1.0f, -(d.x + d.z) / d.y, 1.0f));
            else
                dir.Set((a.x / m, -(d.x + d.z) / d.y, a.z / m));
        }
    }
    dir.MakeNormalize();
    return dir;
}
/******************************************************************************
*    Method:    Line::GetPerpendicular
*       Input
*           firstPlaneIn    Narrow the infinite perpendicular normals to the
*                           line to within a plane. The x',y', z' portion of
*                           sx'+ sy' + sz' + s = 0 ; homogeneous plane equation
*                           excluding w component storing the scalar s since we
*                           have two points already anchoring the plane.
* ******************************************************************************/
float3 __vectorcall King::Line::GetPerpendicular(float3 firstPlaneIn) const 
{ 
    auto rtn = GetPerpendicular() * firstPlaneIn; 
    rtn.MakeNormalize(); 
    return rtn;
}
/******************************************************************************
*    Method:    Line::Collision
******************************************************************************/
inline bool King::Line::Collision(Point const & pointIn) const { return Intersects(pointIn); }
inline bool King::Line::Collision(Ray const& rayIn) const { return rayIn.Collision(*this); }
inline bool King::Line::Collision(Line const & lineIn) const { return Intersects(lineIn); }
inline bool King::Line::Collision(Sphere const & sphereIn) const { return sphereIn.Intersects(*this); }
inline bool King::Line::Collision(Box const& boxIn) const { float3 ptOut; return boxIn.Intersects(*this, &ptOut); }
inline bool King::Line::Collision(Plane const& planeIn) const { return planeIn.Collision(*this); }
inline bool King::Line::Collision(Capsule const& capsuleIn) const { return capsuleIn.Collision(*this); }
inline bool King::Line::Collision(Pyramid const& pyramidIn) const { return false; }// { return pyramidIn.Collision(*this); }
/******************************************************************************
*    Method:    Quad::Intersects
******************************************************************************/
bool King::Quad::Intersects(const Ray& ray, float3* intersectPointOut)
{
    return King::Quad::Intersects(ray.GetOrigin(), ray.GetDirection(), intersectPointOut);
}

bool King::Quad::Intersects(const float3 & rayOriginIn, const float3 & rayDirectionIn, float3 * intersectPointOut)
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
std::vector<Quad> King::Quad::SubDivide()
{
    std::vector<Quad> quadsOut;
    
    float3 c = GetCenter();
    float3 c0 = 0.5f * (pt[1] + pt[0]);
    float3 c1 = 0.5f * (pt[2] + pt[1]);
    float3 c2 = 0.5f * (pt[3] + pt[2]);
    float3 c3 = 0.5f * (pt[0] + pt[3]);

    Quad q;

    // 0
    q.pt[0].Set(pt[0]);
    q.pt[1].Set(c0);
    q.pt[2].Set(c);
    q.pt[3].Set(c3);
    quadsOut.push_back(q);
    // 1
    q.pt[0].Set(c0);
    q.pt[1].Set(pt[1]);
    q.pt[2].Set(c1);
    q.pt[3].Set(c);
    quadsOut.push_back(q);
    // 2
    q.pt[0].Set(c);
    q.pt[1].Set(c1);
    q.pt[2].Set(pt[2]);
    q.pt[3].Set(c2);
    quadsOut.push_back(q);
    // 3
    q.pt[0].Set(c3);
    q.pt[1].Set(c);
    q.pt[2].Set(c2);
    q.pt[3].Set(pt[3]);
    quadsOut.push_back(q);

    return quadsOut;
}
/******************************************************************************
*    Method:    MomentsOfInertia
*       Returns the momement of inertia of our Sphere
******************************************************************************/
DirectX::XMMATRIX King::Sphere::MomentsOfInertia(const float& densityIn)
{
    float r = GetRadius();
    float v = GetVolume();
    float m = densityIn * v;

    auto r_sq = r * r;
    auto i = 2.0f / 5.0f * m * r_sq;

    // Inertia tensor
    DirectX::XMMATRIX I(i, 0, 0, 0,
        0, i, 0, 0,
        0, 0, i, 0,
        0, 0, 0, 1);

    return I;
}
/******************************************************************************
*    Method:    Intersects
*       Sphere vs. Point
******************************************************************************/
bool King::Sphere::Contains(const Point & ptIn) const
{
    float3 d = ptIn - GetCenter();
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
inline bool King::Sphere::Intersects(const Ray & ray, float3 *ptOut) const
{
    float3 r(GetRadius());
    float3 r2 = DirectX::XMVectorMultiply(r, r);
    float3 ray2( GetCenter() - ray.GetOrigin());
    float3 s(DirectX::XMVector3Dot(ray2, ray.GetDirection())); // projection
    float3 l2(DirectX::XMVector3Dot(ray2, ray2) );

    // m2 is squared distance from the center of the sphere to the projection.
    float3 m2 = DirectX::XMVectorNegativeMultiplySubtract(s, s, l2);

    XMVECTOR NoIntersection;
    // If the ray origin is outside the sphere and the center of the sphere is
    // behind the ray origin there is no intersection.
    NoIntersection = DirectX::XMVectorAndInt(DirectX::XMVectorLess(s, DirectX::XMVectorZero()), DirectX::XMVectorGreater(l2, r2));
    // If the squared distance from the center of the sphere to the projection
    // is greater than the radius squared the ray will miss the sphere.
    NoIntersection = DirectX::XMVectorOrInt(NoIntersection, DirectX::XMVectorGreater(m2, r2));

    if (DirectX::XMVector4NotEqualInt(NoIntersection, DirectX::XMVectorTrueInt()))
    {
        // The ray hits the sphere, compute the nearest intersection point.
        float3 q( Sqrt(r2 - m2) );
        float3 t1( s - q );
        float3 t2( s + q );

        l2 <= r2;

        XMVECTOR OriginInside = DirectX::XMVectorLessOrEqual(l2, r2);
        XMVECTOR t = DirectX::XMVectorSelect(t1, t2, OriginInside);
        *ptOut = t;
        return true;
    }
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
bool King::Sphere::Intersects(const Triangle& triIn, float3* intersectionOut) const
{
    float3 sphereCenter = GetCenter();
    float radius = GetRadius();

    float3 closestPoint = triIn.FindNearestPoint(sphereCenter);

    float distance = (sphereCenter - closestPoint).GetMagnitude();

    if (distance <= radius) {
        if (intersectionOut != nullptr) {
            *intersectionOut = closestPoint;
        }
        return true;
    }

    return false;
}
bool King::Sphere::Intersects(const Plane& planeIn) const
{
    auto nPt = planeIn.FindNearestPointOnPlane(GetCenter());

    return Contains(nPt);
}
/******************************************************************************
*    Method:    Intersects Sphere
*       Sphere vs. Sphere
******************************************************************************/
bool King::Sphere::Intersects(const Sphere & rhs) const
{
    float3 d( rhs.GetCenter() - GetCenter() );
    float3 dsq( DirectX::XMVector3Dot(d, d) );

    float3 r3A(GetRadius());
    float3 r3B(rhs.GetRadius());
    auto rsq = r3A + r3B;
    rsq = rsq * rsq;

    return  dsq <= rsq;
}
/******************************************************************************
*    Method:    Intersects Orientated Box
******************************************************************************/
bool King::Sphere::Intersects(const Box& boxIn, const Quaternion& orientationIn) const
{
    if (DirectX::XMQuaternionEqual(orientationIn, DirectX::XMQuaternionIdentity()))
        return boxIn.Intersects(*this);

    //*** TO DO ***
    // https://gamedev.stackexchange.com/questions/163873/separating-axis-theorem-obb-vs-sphere
    return false;
}
/******************************************************************************
*    Method:    FindNearestPointFromSphere
******************************************************************************/
float3 __vectorcall King::Sphere::FindNearestPointFromSphere(const float3& pt3In) const
{
    float3 sphereToPoint = pt3In - GetCenter();
    sphereToPoint.MakeNormalize();
    sphereToPoint *= GetRadius();
    float3 worldPoint = GetCenter() + sphereToPoint;

    return float3();
}
/******************************************************************************
*    Method:    Collision
******************************************************************************/
bool King::Sphere::Collision(Point const & pointIn) const { return Contains(pointIn); }
bool King::Sphere::Collision(Ray const & rayIn) const { float3 ptOut; return Intersects(rayIn, &ptOut); }
bool King::Sphere::Collision(Line const & lineIn) const { return Intersects(lineIn); }
inline bool King::Sphere::Collision(Plane const& planeIn) const { return Intersects(planeIn); }
bool King::Sphere::Collision(Sphere const & sphereIn) const { return Intersects(sphereIn); }
inline bool King::Sphere::Collision(Capsule const& capsuleIn) const { return capsuleIn.Collision(*this); }
bool King::Sphere::Collision(Box const & boxIn) const { return boxIn.Intersects(*this); }
inline bool King::Sphere::Collision(Frustum const& frustumIn) const { return frustumIn.Collision(*this); }
//inline bool King::Sphere::Collision(Pyramid const& pyramidIn) const { return pyramidIn.Collision(*this); } // does not seem to work, linker does not see this definiton for some reason

/******************************************************************************
*    Method:    Merge Sphere
*       grow the sphere to match the min and max extents of each sphere
******************************************************************************/
void King::Sphere::Merge(const Sphere& sIn)
{
    const auto& r1 = GetRadius();
    const auto& c1 = GetCenter();

    const auto& r2 = sIn.GetRadius();
    const auto& c2 = sIn.GetCenter();

    assert(r1 >= 0.f);
    assert(r2 >= 0.f);

    float3 min1 = c1 - r1;
    float3 min2 = c2 - r2;
    float3 pt_min = Min(min1, min2);

    float3 max1 = c1 + r1;
    float3 max2 = c2 + r2;
    float3 pt_max = Max(max1, max2);

    SetCenter((pt_min + pt_max) * 0.5f);
    SetRadius(((pt_max - pt_min) * 0.5f).GetMagnitude());
}
/******************************************************************************
*    Method:    Merge Box
******************************************************************************/
inline void King::Box::Merge(const Box & bIn)
{
    assert(pt_min <= pt_max);

    pt_min = Min(pt_min, bIn.pt_min);
    pt_max = Max(pt_max, bIn.pt_max);
}
/******************************************************************************
*    Method:    Merge Point
******************************************************************************/
inline void __vectorcall King::Box::Merge(const float3 ptIn)
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
    auto center = GetCenter();
    auto extents = GetExtents();

    // Compute the dot product againt each axis of the box.
    // Since the axii are (1,0,0), (0,1,0), (0,0,1) no computation is necessary.
    float3 TOrigin = center - rayIn.GetOrigin();
    float3 AxisDotDirection = rayIn.GetDirection();

    // if (fabs(AxisDotDirection) <= Epsilon) the ray is nearly parallel to the slab.
    XMVECTOR IsParallel = DirectX::XMVectorLessOrEqual(DirectX::XMVectorAbs(AxisDotDirection), g_RayEpsilon);

    // Test against all three axii simultaneously.
    XMVECTOR InverseAxisDotDirection = DirectX::XMVectorReciprocal(AxisDotDirection);
    XMVECTOR t1 = DirectX::XMVectorMultiply(DirectX::XMVectorSubtract(TOrigin, extents), InverseAxisDotDirection);
    XMVECTOR t2 = DirectX::XMVectorMultiply(DirectX::XMVectorAdd(TOrigin, extents), InverseAxisDotDirection);

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
    XMVECTOR ParallelOverlap = DirectX::XMVectorInBounds(TOrigin, extents);
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
inline bool King::Box::Intersects(const Line & lineIn, float3* intersectionOut) const
{ 
    const float3& c = GetCenter();
    auto l_nearest = lineIn.FindNearestPointOnLineSegment(c);

    if (DirectX::XMVector3InBounds(l_nearest - c, GetExtents()))
    {
        if (intersectionOut)
            *intersectionOut = FindNearestPointOnBox(l_nearest);
        return true;
    }

    return false;
}
bool King::Box::Intersects(const Triangle& rhs, float3* intersectionOut) const 
{
    // Get the vertices of the triangle
    float3 v0 = rhs.GetVertex(0);
    float3 v1 = rhs.GetVertex(1);
    float3 v2 = rhs.GetVertex(2);

    // Box corners
    float3 corners[8] = {
        pt_min,
        { pt_min.GetX(), pt_min.GetY(), pt_max.GetZ() },
        { pt_min.GetX(), pt_max.GetY(), pt_min.GetZ() },
        { pt_min.GetX(), pt_max.GetY(), pt_max.GetZ() },
        { pt_max.GetX(), pt_min.GetY(), pt_min.GetZ() },
        { pt_max.GetX(), pt_min.GetY(), pt_max.GetZ() },
        { pt_max.GetX(), pt_max.GetY(), pt_min.GetZ() },
        pt_max
    };

    // Check each edge of the triangle against the box

    if (Intersects(Line(v0, v1), intersectionOut) || Intersects(Line(v1, v2), intersectionOut) || Intersects(Line(v2, v0), intersectionOut)) {
        if (intersectionOut) {
            // Find intersection point(s) if needed
            // (For simplicity, this part can be more complex depending on your specific needs)
            *intersectionOut = (v0 + v1 + v2) / 3.0f; // Use the centroid for now
        }
        return true; // Triangle edge intersects with the box
    }

    // Check if any of the box corners are inside the triangle
    for (const auto& corner : corners) {
        if (rhs.Contains(corner)) {
            if (intersectionOut) {
                *intersectionOut = corner; // Store the corner point as intersection
            }
            return true; // Box corner is inside the triangle
        }
    }

    return false; // No intersection detected
}


inline bool King::Box::Intersects(const Plane& planeIn) const 
{ 
    const float3& c = GetCenter();
    auto p_nearest = planeIn.FindNearestPointOnPlane(c);

    return DirectX::XMVector3InBounds(p_nearest - c, GetExtents());
}
/******************************************************************************
*    Method:    Intersects Box
*
*   |------|*this     pt_min|------|      boxIn.pt_min|------|
*   |      |                |      |                  |      |
*   |      |                |      |                  |      |
*   | |----|-| boxIn        |      |                  |      |
*   |------| |              |------|pt_max            |------|boxIn.pt_max
*     |      |              
*     |      |
*     |------|
* 
******************************************************************************/
bool King::Box::Intersects(const Box & boxIn) const
{
    XMVECTOR d = DirectX::XMVectorOrInt(DirectX::XMVectorGreater(pt_min, boxIn.pt_max), DirectX::XMVectorGreater(boxIn.pt_min, pt_max));
    
    // if any component is false, return true
    return !DirectX::Internal::XMVector3AnyTrue(d);
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
    auto c(sphereIn.GetCenter());
    float r(sphereIn.GetRadius());

    auto delta_dist = FindNearestPointOnBox(c) - c;

    // sum of the squares
    auto distSq = Dot(delta_dist, delta_dist);
    // compare to a square inlieu of sqrt and compare
    auto rSq = r * r;

    return distSq < rSq;
}
bool King::Box::Intersects(const Pyramid& pyramidIn, float3* intersectionOut) const
{
    return pyramidIn.Intersects(*this, Quaternion(), intersectionOut);
}
/******************************************************************************
*    Method:    Find Nearest Point On Box
*   |------|
*   |      |
*   |      |*<--------------------* pt3In
*   |      |return
*   |------|
******************************************************************************/
float3 __vectorcall King::Box::FindNearestPointOnBox(const float3 &pt3In) const
{
    return Max(pt_min, Min(pt_max, pt3In));
}
/******************************************************************************
*    Method:    Find Nearest Point On Box (transformed)
*   \-----\               ------*  pt3In (world)
*    \     \ return ------
*     \     \*<-----
*      \     \  *<---------------* localSpacePoint (box can be treated as a AABB)
*       \-----\ (npOnBox)  
******************************************************************************/
float3 __vectorcall King::Box::FindNearestPointOnBox(const float3 & pt3In, const DirectX::FXMMATRIX & M) const
{
    // world to box space
    float3 localSpacePoint(pt3In);
    localSpacePoint *= M;

    auto npOnBox = FindNearestPointOnBox(localSpacePoint);
    
    // box to world space
    auto invM = DirectX::XMMatrixInverse(nullptr, M);
    npOnBox *= invM;

    return npOnBox;
}
inline bool King::Box::Collision(Ray const & rayIn) const { float distOut; return Intersects(rayIn, distOut); }
inline bool King::Box::Collision(Point const & pointIn) const { return DirectX::XMVector3InBounds((DirectX::XMVECTOR)(pointIn - GetCenter()), GetExtents()); }
inline bool King::Box::Collision(Line const & lineIn) const { float3 ptOut; return Intersects(lineIn, &ptOut); }
inline bool King::Box::Collision(Plane const& planeIn) const { return Intersects(planeIn); }
inline bool King::Box::Collision(Sphere const & sphereIn) const { return Intersects(sphereIn); }
inline bool King::Box::Collision(Capsule const& capsuleIn) const { return capsuleIn.Intersects(*this); }
inline bool King::Box::Collision(Box const & boxIn) const { return Intersects(boxIn); }
inline bool King::Box::Collision(Frustum const& frustumIn) const { return frustumIn.Collision(*this); }
//inline bool King::Box::Collision(Pyramid const& pyramidIn) const { return pyramidIn.Collision(*this); } // this does not seem to work, linker does not see this definition
/******************************************************************************
*    Method:    CollisionVolume
*       Assumption:  Collision detection has already occured and is true
*       Note: solution could be a plane if they are touching and not overlapping
*       otherwise a box.
******************************************************************************/
Box King::Box::CollisionVolume(const Box& boxIn)
{

    auto temp1 = Min(pt_max, boxIn.pt_max); // smallest maximum
    auto temp2 = Max(pt_min, boxIn.pt_min); // largest minimum
    Box collisionVolume(Min(temp1, temp2), Max(temp1, temp2));

    return collisionVolume;
}
/******************************************************************************
*    Method:    MomentsOfInertia
*       Returns the momement of inertia of our box of uniform density
******************************************************************************/
DirectX::XMMATRIX King::Box::MomentsOfInertia(const float& densityIn)
{
    auto s(GetSize().Get_XMFLOAT3());
    float volume = s.x * s.y * s.z;
    float mass = densityIn * volume;

    auto s_sq_temp(GetSize());
    s_sq_temp *= s_sq_temp;
    auto s_sq = s_sq_temp.Get_XMFLOAT3();

    // for a box
    float3 i;
    i.SetX(s_sq.y + s_sq.z);
    i.SetY(s_sq.x + s_sq.z);
    i.SetZ(s_sq.x + s_sq.y);
    i = i * mass / 12.0f; 
    
    auto Ii = i.Get_XMFLOAT3();

    // Inertia tensor
    DirectX::XMMATRIX I(Ii.x, 0, 0, 0,
                        0, Ii.y, 0, 0,
                        0, 0, Ii.z, 0,
                        0, 0, 0, 1);

    return I;
}
/******************************************************************************
*    Method:    MomentsOfInertiaRotated
*       Rotated about an arbitrary axis
******************************************************************************/
DirectX::XMMATRIX King::Box::MomentsOfInertiaRotated(const DirectX::XMMATRIX& Iin, const DirectX::XMMATRIX& rotationIn)
{
    auto I_inv = DirectX::XMMatrixInverse(nullptr, Iin);
    auto r_inv = DirectX::XMMatrixInverse(nullptr, rotationIn);
    return r_inv * I_inv * rotationIn;
}
/******************************************************************************
*    Method:    IdentifyCorners
*       Returns the list with corner identifiers.
******************************************************************************/
vector<pair<float3, enum King::Box::CornerDescription>> King::Box::IdentifyCorners(const vector<float3>& pointsIn, const Quaternion* quaternionIn) const
{
    vector<pair<float3, CornerDescription>> vectorCorners;
    vectorCorners.reserve(8);

    auto myCorners = GetCorners8(quaternionIn);

    for (const auto& pt : pointsIn)
    {
        pair<float3, CornerDescription> ptWithDescriptor;
        ptWithDescriptor.first = pt;

        // default description
        ptWithDescriptor.second = CornerDescription::INVALID;
        // identify and change the default
        for (unsigned int i=0; i<8; ++i)
        {
            if (pt == myCorners[i])
            {
                ptWithDescriptor.second = static_cast<CornerDescription>(CornerDescription::lbb + i);
                break;
            }
        }
        vectorCorners.push_back(ptWithDescriptor);
    }

    return vectorCorners;
}
/******************************************************************************
*    Method:    Contains Point
*   |------|
*   |      |
*   |  *   |
*   |  ptIn|
*   |------|
******************************************************************************/
bool __vectorcall King::Box::Contains(const float3 ptIn) const
{
    // convert to local coordinate system and check against extents
    return DirectX::XMVector3InBounds(ptIn - GetCenter(), GetExtents());
}
/******************************************************************************
*    Method:    Constrain Point
*   |------|
*   |      |
*   |  *   |
*   |  ptIn|
*   |------|
******************************************************************************/
void King::Box::Constrain(float3* ptOut) const
{
    if (!DirectX::XMVector3InBounds(*ptOut - GetCenter(), GetExtents()))
    {
        XMFLOAT3 p = *ptOut;
        XMFLOAT3 min = GetMin();
        XMFLOAT3 max = GetMax();
        if (p.x < min.x) p.x = min.x;
        else if (p.x > max.x) p.x = max.x;

        if (p.y < min.y) p.y = min.y;
        else if (p.y > max.y) p.y = max.y;

        if (p.z < min.z) p.z = min.z;
        else if (p.z > max.z) p.z = max.z;

        *ptOut = p;
    }
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
    if(boxIn.pt_min >= pt_min && boxIn.pt_max <= pt_max)
        return true;

    return false;
}
/******************************************************************************
*    Method:    SetAABBfromThisTransformedBox
*       Sets the extents outward for an axis aligned box (AAB) that encloses
*       the original AABB that was rotated, scaled, and translated to be
*       a orientated bounding box (OBB)
******************************************************************************/
void King::Box::SetAABBfromThisTransformedBox(FXMMATRIX M)
{
    auto corners = GetCorners8Transformed(M); // need to free this when done

    float3 minC, maxC, corner;
    minC = maxC = corner = float3(corners[0]);

    for (size_t i = 1; i < 8; ++i)
    {
        minC = Min(minC, float3(corners[i]));
        maxC = Max(maxC, float3(corners[i]));
    }
    Set(minC, maxC);

    delete[] corners;
}
/******************************************************************************
*    Method:    SetAABBfromBoundingSphere
*       Sets the extents outward for an axis aligned box (AAB) that encloses
*       the original AABB roated to any rotation based on the bounding sphere
******************************************************************************/
void __vectorcall King::Box::SetAABBfromBoundingSphere(Sphere s)
{
    SetCenterAndExtents(s.GetCenter(), s.GetRadius());
}
/******************************************************************************
*    Method:    GetCorners8Transformed
*       Usage: Transform the box corners (in world space) by another matrix
******************************************************************************/
//std::vector<King::float3> 
DirectX::XMFLOAT4* King::Box::GetCorners8Transformed(const DirectX::FXMMATRIX &M)
{
    DirectX::XMFLOAT4* pArrayOut = new DirectX::XMFLOAT4[8];
    assert(pArrayOut);

    if (!GetExtents())
    {
        // invalid as infinite if really large
        SetExtents(0.5f);
    }

    DirectX::XMFLOAT3* pArray = new DirectX::XMFLOAT3[8];
    assert(pArray);


    GetCorners8ObjectSpace(pArray);
    const float3 a = pArray[0];
    assert(a);

    DirectX::XMVector3TransformStream(pArrayOut, sizeof(XMFLOAT4), pArray, sizeof(XMFLOAT3), 8, M);

    const float4 ao = pArrayOut[0];
    assert(ao);


    delete[] pArray;
    return pArrayOut; // remember to delete[] this array when done
}
/******************************************************************************
*    Method:    GetCorners8ObjectSpace
******************************************************************************/
std::vector<King::float3> King::Box::GetCorners8ObjectSpace(const Quaternion* quaternionIn) 
{
    DirectX::XMFLOAT3 pArrayInOut[8];
    GetCorners8ObjectSpace(pArrayInOut, quaternionIn);

    std::vector<float3> rtn;
    for (int i = 0; i < 8; ++i)    rtn.push_back(float3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCorners8
******************************************************************************/
std::vector<King::float3> King::Box::GetCorners8(const Quaternion * quaternionIn) const
{
    DirectX::XMFLOAT3 pArrayInOut[8];
    GetCorners8(pArrayInOut, quaternionIn);

    std::vector<float3> rtn;
    for (int i = 0; i < 8; ++i)    rtn.push_back(float3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCornersTop4
******************************************************************************/
std::vector<King::float3> King::Box::GetCornersTop4(const Quaternion * quaternionIn) const // scaled, rotated and translated
{
    DirectX::XMFLOAT3 pArrayInOut[4];
    GetCornersTop4(pArrayInOut, quaternionIn);

    std::vector<King::float3> rtn;
    for (int i = 0; i < 4; ++i)    rtn.push_back(King::float3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCornersBottom4
******************************************************************************/
std::vector<King::float3> King::Box::GetCornersBottom4(const Quaternion * quaternionIn) const // scaled, rotated and translated
{
    DirectX::XMFLOAT3 pArrayInOut[4];
    GetCornersBottom4(pArrayInOut, quaternionIn);

    std::vector<King::float3> rtn;
    for (int i = 0; i < 4; ++i)    rtn.push_back(King::float3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCornersLowest4
******************************************************************************/
std::vector<King::float3> King::Box::GetCornersLowest4(const Quaternion * quaternionIn) const // scaled, rotated and translated
{
    DirectX::XMFLOAT3 pArrayInOut[4];
    GetCornersLowest4(pArrayInOut, quaternionIn);

    std::vector<King::float3> rtn;
    for (int i = 0; i < 4; ++i)    rtn.push_back(King::float3(pArrayInOut[i]));

    return rtn;
}
/******************************************************************************
*    Method:    GetCorners8ObectSpace
******************************************************************************/
void King::Box::GetCorners8ObjectSpace(DirectX::XMFLOAT3* pArrayInOut, const Quaternion* quaternionIn) 
{
    assert(pArrayInOut != nullptr);

    King::float3 corn;

    // Ensure the box is not infinite or otherwise too large to handle (leave room more math operators to use extents)
    auto e = GetExtents();
    const auto Extents_Max = FLT_MAX / 10000.f;
    bool f(false);
    if (e.GetX() > Extents_Max) { e.SetX(Extents_Max); f = true; }
    if (e.GetY() > Extents_Max) { e.SetY(Extents_Max); f = true; }
    if (e.GetZ() > Extents_Max) { e.SetZ(Extents_Max); f = true; }
    if (f) SetExtents(e);

    if (quaternionIn == nullptr)
    {
        for (size_t i = 0; i < 8; ++i)
        {
            corn = King::float3(DirectX::XMVectorMultiply(g_boxCorners[i], e)); // scale
            assert((bool)corn);
            pArrayInOut[i] = corn;
        }
    }
    else
    {
        for (size_t i = 0; i < 8; ++i)
        {
            corn = DirectX::XMVectorMultiply(g_boxCorners[i], e); // scale to extents
            corn = DirectX::XMVector3Rotate(corn, *quaternionIn); // rotate
            assert((bool)corn);
            pArrayInOut[i] = corn;
        }
    }
}
/******************************************************************************
*    Method:    GetCorners8
******************************************************************************/
void King::Box::GetCorners8(DirectX::XMFLOAT3 *pArrayInOut, const Quaternion * quaternionIn) const
{
    assert(pArrayInOut != nullptr);

    King::float3 corn;

    auto c = GetCenter();
    auto e = pt_max - c;

    if (quaternionIn == nullptr)
    {
        for (size_t i = 0; i < 8; ++i)
            pArrayInOut[i] = King::float3(DirectX::XMVectorMultiplyAdd(g_boxCorners[i], e, c)); // scale and translate
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
void King::Box::GetCornersBottom4(DirectX::XMFLOAT3 *pArrayInOut, const Quaternion * quaternionIn) const
{
    assert(pArrayInOut != nullptr);

    King::float3 corn;

    King::float3 c = GetCenter();
    King::float3 e = pt_max - c;

    if (quaternionIn == nullptr)
    {
        for (size_t i = 0; i < 4; ++i)
            pArrayInOut[i] = float3(DirectX::XMVectorMultiplyAdd(g_boxCorners[i], e, c)); // scale and translate
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
void King::Box::GetCornersTop4(DirectX::XMFLOAT3 *pArrayInOut, const Quaternion * quaternionIn) const
{
    assert(pArrayInOut != nullptr);

    King::float3 corn;

    King::float3 c = GetCenter();
    King::float3 e = pt_max - c;

    if (quaternionIn == nullptr)
    {
        for (size_t i = 0; i < 4; ++i)
            pArrayInOut[i] = float3(DirectX::XMVectorMultiplyAdd(g_boxCorners[i+4], e, c)); // scale and translate
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
void King::Box::GetCornersLowest4(DirectX::XMFLOAT3 * pArrayOut, const Quaternion * quaternionIn) const
{
    assert(pArrayOut != nullptr);

    DirectX::XMFLOAT3 corn[8];
    GetCorners8(corn, quaternionIn);

    pArrayOut[0] = { 0.f,FLT_MAX,0.f };
    pArrayOut[1] = { 0.f,FLT_MAX,0.f };
    pArrayOut[2] = { 0.f,FLT_MAX,0.f };
    pArrayOut[3] = { 0.f,FLT_MAX,0.f };

    // Sort corners in Y and keep lowest 4
    for(size_t keep = 0; keep < 4; ++keep)
        for (size_t i = keep; i < 8; ++i)
            if (corn[i].y < pArrayOut[keep].y) std::swap(corn[i], pArrayOut[keep]);
    /*
    Ex:
    pArrayOut[0] { x:   3.23799 y:   16.3983 z:  -83.7365 }
    pArrayOut[1] { x:   3.23799 y:   26.6022 z:  -66.7107 }
    pArrayOut[2] { x:  -3.23677 y:   26.5955 z:  -66.7107 }
    pArrayOut[3] { x:  -3.23677 y:   16.2477 z:  -83.7365 }
    */
    // Sort remaining corners in X
    for (size_t j = 0; j < 3; ++j) // 0,1,2
        for (size_t i = j+1; i < 4; ++i) // 1,2,3
            if (pArrayOut[i].x < pArrayOut[j].x) std::swap(pArrayOut[j], pArrayOut[i]);
    
    // now two smallest x are positions 0 & 1
    if (pArrayOut[1].z < pArrayOut[0].z)
        std::swap(pArrayOut[1], pArrayOut[0]);
    // and the two largest in x are positions 2 & 3
    if (pArrayOut[2].z < pArrayOut[3].z)
        std::swap(pArrayOut[2], pArrayOut[3]);
    /*
    pArrayOut[0] { x:  -3.23677 y:   16.2477 z:  -83.7365 }
    pArrayOut[1] { x:  -3.23677 y:   26.5955 z:  -66.7107 }
    pArrayOut[2] { x:   3.23799 y:   26.6022 z:  -66.7107 }
    pArrayOut[3] { x:   3.23799 y:   16.3983 z:  -83.7365 }
    */

    // now we are arranged as RHS:
    // -Z
    // ^
    // |0.---.3
    // | |   |
    // |1.---.2
    // ----------> +X
}
/******************************************************************************
*   Method:    operator= ; copy assignment
 ******************************************************************************/
King::ModelScaffold& King::ModelScaffold::operator= (const King::Line& lineIn)
{
    // position is added as default
    // uses the vertx format already set in class
    const auto& bs = _vertexFormat.GetByteSize();
    _vertexBufferMaster.SetStride(bs);

    // initialize vertex buffer
    _vertexBufferMaster.Initialize(2 * bs);
    auto size = _vertexBufferMaster.GetElements();
    assert(size == 2);

    auto pos = lineIn.GetVertex(0).Get_XMFLOAT3();
    SetVertexElement(0, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&pos));

    pos = lineIn.GetVertex(1).Get_XMFLOAT3();
    SetVertexElement(1, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&pos));

    _indexBufferMaster = { 0,1 };

    CalculateBoundingBox();

    return *this;
}
/******************************************************************************
*    Method:    operator= ; copy assignment
*           4-------7
*           |       |
*           |   T   |
*           |       |
*   4-------5-------6-------7-------4
*   |       |       |       |       |
*   |   L   |   F   |   R   |   Bk  |
*   |       |       |       |       |
*   0-------1-------2-------3-------0
*           |       |
*           |   Bm  |
*           |       |
*           0-------3
*   0 = { -1.0f, -1.0f, -1.0f } // lbb (min)
*   1 = { -1.0f, -1.0f,  1.0f } // lbf
*   2 = { 1.0f, -1.0f,  1.0f } // rbf
*   3 = { 1.0f, -1.0f, -1.0f } // rbb
*   4 = { -1.0f,  1.0f, -1.0f } // ltb
*   5 = { -1.0f, 1.0f, 1.0f } // ltf
*   6 = { 1.0f,  1.0f,  1.0f } // rtf (max)
*   7 = { 1.0f,  1.0f, -1.0f } // rtb
*
* ******************************************************************************/
King::ModelScaffold & King::ModelScaffold::operator= (const King::Box& boxIn)
{
    // position is added as default
    //_vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
    //_vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());

    _boundingBox = boxIn;

    // get points
    DirectX::XMFLOAT3 vp[8];
    _boundingBox.GetCorners8(vp);
    // initialize vertex buffer
    _vertexBufferMaster.Initialize(8 * _vertexBufferMaster.GetStride());
    auto size = _vertexBufferMaster.GetElements();
    SetVertexStream(0, reinterpret_cast<uint8_t*>(vp), size);
    
    // Points, bottom 4 then top 4
    _indexBufferMaster = { 0,1,2,3,4,5,6,7 };

    return *this;
}
/******************************************************************************
*    Method:    operator= ; copy assignment
*           4-------7
*           |       |
*           |   T   |
*           |       |
*   4-------5-------6-------7-------4
*   |       |       |       |       |
*   |   L   |   F   |   R   |   Bk  |
*   |       |       |       |       |
*   0-------1-------2-------3-------0
*           |       |
*           |   Bm  |
*           |       |
*           0-------3
*   0 = { -1.0f, -1.0f, -1.0f } // lbb (min)
*   1 = { -1.0f, -1.0f,  1.0f } // lbf
*   2 = { 1.0f, -1.0f,  1.0f } // rbf
*   3 = { 1.0f, -1.0f, -1.0f } // rbb
*   4 = { -1.0f,  1.0f, -1.0f } // ltb
*   5 = { -1.0f, 1.0f, 1.0f } // ltf
*   6 = { 1.0f,  1.0f,  1.0f } // rtf (max)
*   7 = { 1.0f,  1.0f, -1.0f } // rtb
*
* ******************************************************************************/
King::LineModel& King::LineModel::operator= (const King::Box& boxIn)
{
    // position is added as default
    _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::color, VertexAttrib::enumFormat::format_float32x4);

    _vertexBufferMaster.Initialize((uint32_t)8 * _vertexFormat.GetByteSize()); 
    _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());

    _boundingBox = boxIn;

    // get points
    DirectX::XMFLOAT3 vp[8];
    _boundingBox.GetCorners8(vp);

    // initialize vertex buffer
    //auto size = _vertexBufferMaster.GetElements(); // 8
    auto color = float4(0.f, 1.0f, 1.0f, 1.0f).Get_XMFLOAT4(); // yellow opaque

    for (size_t i = 0; i < 8; ++i)
    {
        SetVertexElement(i, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&vp[i]));
        SetVertexElement(i, VertexAttrib::enumDesc::color, reinterpret_cast<uint8_t*>(&color));
    }

    //_vertexBufferMaster.WriteText("lineBoxTest.txt");
    //std::ofstream outfile("lineBoxTest.txt", std::ofstream::trunc);
    //outfile << "Length: " << std::to_string((_vertexBufferMaster.GetLength())) << '\n';
    //outfile << "Stride: " << std::to_string((_vertexBufferMaster.GetStride())) << '\n';
    //outfile << "Elements: " << std::to_string((_vertexBufferMaster.GetElements())) << '\n';
    //outfile << "Data:" << '\n';

    //for (size_t i = 0; i < _vertexBufferMaster.GetElements() * 7; ++i)
    //{
    //    outfile << std::to_string(reinterpret_cast<float*>(&_vertexBufferMaster.GetData())[i]) << "\t";
    //    if (i % 7 == 6) // every 7 values, new line
    //        outfile << '\n';
    //}
    //outfile.close();

    // Lines
    //_indexBufferMaster.Initialize(24);
    _indexBufferMaster = { 0,1, 1,2, 2,3, 3,0, 4,5, 5,6, 6,7, 7,4, 0,4, 1,5, 2,6, 3,7 };
     
    auto mesh = CreateMesh((uint32_t)_indexBufferMaster.GetElements() / 2);
    mesh->Set_name("lineMesh");
    mesh->CalculateBoundingBox();

    // just incase we create more than one mesh or append data to our master we will reset the pointer addresses in each mesh
    for (auto& m : _meshes)
    {
        m->SetVB(_vertexBufferMaster.GetData());
        m->SetIB(_indexBufferMaster.GetData());
    }
    return *this;
}
/******************************************************************************
*    Method:    CreateMeshFrom Line
******************************************************************************/
int King::LineModel::CreateMeshFrom(const King::Line& lineIn, float4 colorIn)
{
    // check that the format is correct
    if (!_vertexFormat.Has(VertexAttrib::enumDesc::position))
        _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
    if (!_vertexFormat.Has(VertexAttrib::enumDesc::color))
        _vertexFormat.SetNext(VertexAttrib::enumDesc::color, VertexAttrib::enumFormat::format_float32x4);
    // encase we changed one above
    _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
    assert(_vertexFormat.GetByteSize() == 12+16);

    auto color = colorIn.Get_XMFLOAT4();

    // Create our new mesh and add to model
    auto vbStart = _vertexBufferMaster.GetElements();
    auto ibStart = _indexBufferMaster.GetElements();

    if (!(bool)(_vertexBufferMaster))
    {
        // empty vertex buffer, so create a new one using the _vertexFormat already set in our model
        // scaffold code only adds position data
        // note, this operator also resets the indexbuffer
        ModelScaffold::operator=(lineIn); 

        auto size = _vertexBufferMaster.GetElements();
        assert(size == 2);

        for (size_t i = 0; i < size; ++i)
        {
            SetVertexElement(i, VertexAttrib::enumDesc::color, reinterpret_cast<uint8_t*>(&color));
        }
    }
    else
    {
        MemoryBlock<uint8_t> vb(2, _vertexFormat.GetByteSize());
        _vertexBufferMaster += vb;

        auto size = vb.GetElements();
        assert(size == 2); // since it is one line
        for (size_t i = 0; i < size; ++i)
        {
            auto pos = lineIn.GetVertex(i).Get_XMFLOAT3();
            // modifies _vertexBufferMaster 
            SetVertexElement(vbStart + i, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&pos));
            SetVertexElement(vbStart + i, VertexAttrib::enumDesc::color, reinterpret_cast<uint8_t*>(&color));
        }

        MemoryBlock<uint32_t> ib;
        ib = { 0,1 }; // refers to vertex buffer start
        _indexBufferMaster += ib;
    }
    auto mesh = CreateMesh(1, vbStart, ibStart);
    auto index = _meshes.size() - 1;
    mesh->Set_name("lineMesh"s + std::to_string(index));

    HelperSetAllMeshesVBandIBtoMaster();

    // model
    CalculateBoundingBox();

    return index;
}
/******************************************************************************
*    Method:    CreateMeshFrom Triangle
******************************************************************************/
int King::Model::CreateMeshFrom(const Triangle& tri)
{
    vector<float3>      positions(3);
    vector<uint32_t>    indicies(3);
    vector<float3>      normals(3);

    indicies = { 0, 1, 2 };
    auto normal = tri.GetNormalCCW();

    for (auto i = 0ul; i < 3; ++i)
    {
        positions.push_back(tri.GetVertex(i));
        normals.push_back(normal);
    }

    auto textureCoordinates = HelperCreateTextureCoordinatesForTriOrQuad(positions, 1);

    return HelperCreateMeshFrom(positions, indicies, textureCoordinates, normals);
}
/******************************************************************************
*    Method:    CreateMeshFrom Quad
******************************************************************************/
int King::Model::CreateMeshFrom(const Quad& q)
{
    vector<float3>      positions(4);
    vector<uint32_t>    indicies(6);
    vector<float3>      normals(4);

    indicies = { 0, 1, 2, 2, 1, 3 };
    auto normal = q.GetNormalCCW();

    for (auto i = 0ul; i < 4; ++i)
    {
        positions.push_back(q.GetVertex(i));
        normals.push_back(normal);
    }

    auto textureCoordinates = HelperCreateTextureCoordinatesForTriOrQuad(positions, 1);

    return HelperCreateMeshFrom(positions, indicies, textureCoordinates, normals);
}

/******************************************************************************
*    Method:    CreateMeshFor GridXZ 
*       Input:
*           numWidth        number of cells (verticies - 1) in the X direction
*           numHeight       number of cells (verticies - 1) in the z direction
*       Output: 
*           returns the mesh index number created
******************************************************************************/
int King::Model::CreateMeshForGridXZ(const uint32_t numWidth, const uint32_t numHeight, const float2 cellSize)
{
    const auto numVerticies = (numWidth + 1) * (numHeight + 1);
    const auto numIndicies = numWidth * numHeight * 6;

    vector<float3>      positions; positions.reserve(numVerticies);
    vector<uint32_t>    indicies; indicies.reserve(numIndicies);
    vector<float3>      normals; normals.reserve(numVerticies);

    // RHS 
    float3 center(0.f, 0.f);
    float3 start = center - float3(cellSize.GetX() * numWidth, 0.f, cellSize.GetY() * numHeight) * 0.5f;

    for (uint32_t j = 0; j <= numHeight; ++j)
        for (uint32_t i = 0; i <= numWidth; ++i)
        {
            positions.push_back(start + float3((float)i * cellSize.GetX(), 0.f, (float)j * cellSize.GetY()));
            normals.emplace_back(0.f, 1.f, 0.f);
        }

    /*
    0   1   2
    ---------
    |T5/|\T7|
    | / | \ |
    |/T6|T8\|
    ---------
    3   4   5
    ---------
    |\T2|T3/|
    | \ | / |
    |T1\|/T4|
    ---------
    6   7   8
    */

    // cells
    bool alternate;
    bool alternateCols(true);
    for (uint32_t j = 0; j < numHeight; ++j)
    {
        alternateCols = !alternateCols;
        alternate = alternateCols;
        uint32_t rowStart = j * (numWidth + 1);
        for (uint32_t i = 0; i < numWidth; ++i)
        {
            // create quad
            if (alternate)
            {
                // LL to UR (3 to 1)
                indicies.push_back(i + 0 + rowStart);
                indicies.push_back(i + (numWidth + 1) + rowStart);
                indicies.push_back(i + 1 + rowStart);

                indicies.push_back(i + 1 + rowStart);
                indicies.push_back(i + (numWidth + 1) + rowStart);
                indicies.push_back(i + (numWidth + 2) + rowStart);
            }
            else
            {
                // UL to LR (3 to 7)
                indicies.push_back(i + 0 + rowStart);
                indicies.push_back(i + (numWidth + 1) + rowStart);
                indicies.push_back(i + (numWidth + 2) + rowStart);

                indicies.push_back(i + 0 + rowStart);
                indicies.push_back(i + (numWidth + 2) + rowStart);
                indicies.push_back(i + 1 + rowStart);
            }
            alternate = !alternate;
        }
    }

    auto textureCoordinates = HelperCreateTextureCoordinatesForTriOrQuad(positions, numWidth + 1);

    return HelperCreateMeshFrom(positions, indicies, textureCoordinates, normals);
}

/******************************************************************************
*    Method:    HelperCreateTextureCoordinatesForTriOrQuad
******************************************************************************/
std::vector<King::float2> King::Model::HelperCreateTextureCoordinatesForTriOrQuad(const std::vector<King::float3>& pt, uint32_t ptsWide)
{
    assert(pt.size() > 2);
    vector<float2> textureCoordinatesOut;
    textureCoordinatesOut.reserve(pt.size());

    // RHS CCW
    auto normal = Cross(pt[pt.size() - ptsWide] - pt[0], pt[ptsWide - 1] - pt[0]);
    normal.MakeNormalize();

    // transform from global space to texture space
    Rectangle2DF texture;
    const float3& from(normal);
    const float3 to(0.f, 0.f, 1.f); // normal to xy plane
    Quaternion toTextureSpace(from, to);
    // for a unit square, 1.41421356237 = diagonal * scale
    float d1 = (pt[pt.size()-1] - pt[0]).GetMagnitude();
    float scale(1.41421356237f);
    scale /= d1;

    // find the center of the object
    auto c = float3::Average(pt);
    c = toTextureSpace * c;
    // rotate and scale each point, discarding z which is zero
    // translate from c to 0,0 for center of scale
    for (auto i = 0ul; i < pt.size(); ++i)
    {
        auto uv = toTextureSpace * pt[i];
        uv -= c;
        uv *= scale;
        auto textureCoord = float2(uv);
        // texture y space is reversed
        textureCoord.SetY(-textureCoord.GetY());
        textureCoord += 0.5f;

        textureCoord = Abs(textureCoord);

        if (textureCoord.GetX() < 1.e-7)
            textureCoord.SetX(0.f);
        if (textureCoord.GetY() < 1.e-7)
            textureCoord.SetY(0.f);

        textureCoordinatesOut.push_back(textureCoord);
    }

    return textureCoordinatesOut;
}
/******************************************************************************
*    Method:    HelperCreateTextureCoordinatesForSphere
*          \ /
*    TOP:---*---
*          / \
*         |   |
*         |   |
*          \ /
*        ---*---  :BOTTOM
*          / \
*   Uses the cube map method
******************************************************************************/
void King::Model::HelperCreateTextureCoordinatesForSphere()
{
    auto c = GetBoundingBox().GetCenter();
    c = DirectX::XMVectorSelect(g_XMZero, c, DirectX::XMVectorNearEqual(c, g_XMZero, float3(0.1e-7)));

    cout << "Sphere center: " << c << "\n";
    auto pts = GetPositions();
    cout << "pts: " << pts.size() << "\n";
    assert(pts.size() > 2);
    vector<vector<float>> uvs(pts.size());

    float3 uV, uv;
    for (auto& pt : pts)
    {
        int index = &pt - &pts[0];
        
        uv = uV = float3(pt) - c;
        uV.Normalize();
        //uV.Absolute();
        auto t = max(uV.GetX(), uV.GetY());
        t = max(uV.GetZ(), t);

        uv = uv / (2.f * t) + 0.5f;
        // one coordinate will then be 0 or 1 indicating which texture of the cube map it maps to
        // the other two will be in the range of 0 to +1
        auto d = uv.Get_XMFLOAT3();
        auto ct = uV.Get_XMFLOAT3();
        if (ct.x == 1.f) { uvs[index].push_back(d.y); uvs[index].push_back(d.z); }
        else if (ct.y == 1.f) { uvs[index].push_back(d.x); uvs[index].push_back(d.z); }
        else if (ct.z == 1.f) { uvs[index].push_back(d.x); uvs[index].push_back(d.y); }
        else
        {
            // should not hit this!!!!
            uvs[index].push_back(d.x); uvs[index].push_back(d.y);
        }
        // *** TO DO *** Untested
        //cout << "UV: " << index << " " << float2(uvs[index]) << "\n";
    }

    SetDataAttribute(uvs, VertexAttrib::enumDesc::textureCoord, true);

    return;
}
/******************************************************************************
*    Method:    HelperCreateMeshFrom
*       Input buffers with positionsIn and indicies being of non-zero size
*       Input buffers uvIn and normalsIn are optional and added if that format
*           is already defined in the model vertex format. Note that these
*           two buffers must be size zero OR size of positionsIn.
******************************************************************************/
int King::Model::HelperCreateMeshFrom(const vector<float3>& positionsIn, const vector<uint32_t>& indiciesIn, const vector<float2>& uvIn, const vector<float3>& normalsIn)
{
    // Input validation
    assert(positionsIn.size());
    assert(indiciesIn.size());
    assert((uvIn.size() == 0 ? true : uvIn.size() == positionsIn.size()));
    assert((normalsIn.size() == 0 ? true : normalsIn.size() == positionsIn.size()));

    // is the existing vertex format sufficient?
    if (!_materials.count("default"))
        AddMaterial(Material::Create("default"));
    if (_vertexFormat.GetByteSize() < 56)
    {
        if (!_vertexFormat.Has(VertexAttrib::enumDesc::position))
            _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);

        _vertexFormat.SetNext(VertexAttrib::enumDesc::textureCoord, VertexAttrib::enumFormat::format_float32x2);
        _vertexFormat.SetNext(VertexAttrib::enumDesc::normal, VertexAttrib::enumFormat::format_float32x3);
        _vertexFormat.SetNext(VertexAttrib::enumDesc::tangent, VertexAttrib::enumFormat::format_float32x3);
        _vertexFormat.SetNext(VertexAttrib::enumDesc::bitangent, VertexAttrib::enumFormat::format_float32x3);
        _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
    }

    // format offsets
    auto indexPos = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position); // UINT16_MAX did not find
    auto indexUV = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::textureCoord); // UINT16_MAX did not find
    auto indexNorm = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::normal); // UINT16_MAX did not find
    // must have position data in format, if not, reset
    if (indexPos == UINT16_MAX)
    {
        _vertexBufferMaster.Destroy();
        _indexBufferMaster.Destroy();
        _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
        _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
    }
    // start where the last mesh left off
    const uint32_t vbStart(_vertexBufferMaster.GetElements()); 
    const uint32_t ibStart(_indexBufferMaster.GetElements()); 
    // create our mesh and copy the indicies
    {
        // Expand index memory block
        MemoryBlock<uint32_t> ib(indiciesIn.size());
        size_t numTriangles = indiciesIn.size() / 3;
        // indicies, RH CCW
        ib.Copy(0, indiciesIn.data(), indiciesIn.size());
        // store our data into the model master buffers
        AddMasterIndexData(ib);
        // our mesh!
        auto tm = CreateMesh(numTriangles, vbStart, ibStart);
        tm->Set_name("CreateMeshFrom");
        tm->Set_materialName("default");
    }
    // vertex
    {
        // Expand vertex memory block
        {
            MemoryBlock<uint8_t> vb(positionsIn.size(), GetVertexStride());
            AddMasterVertexData(vb);
        }
        // build the vertex data
        const auto offsetPosBytes = _vertexFormat.GetAttributeByteStart(indexPos);
        const auto offsetUVBytes = _vertexFormat.GetAttributeByteStart(indexUV);
        const auto offsetNormBytes = _vertexFormat.GetAttributeByteStart(indexNorm);

        for (size_t i = 0; i < positionsIn.size(); ++i)
        {
            // positions
            assert(indexPos != UINT16_MAX);
            {
                auto src = static_cast<const uint8_t*>(positionsIn[i]);
                auto dest = GetVertexAddr(i, vbStart) + offsetPosBytes;
                std::copy(src, src + 12, dest);
            }

            // texture coordinates
            if (indexUV && uvIn.size())
            {
                auto src = static_cast<const uint8_t*>(uvIn[i]);
                auto dest = GetVertexAddr(i, vbStart) + offsetUVBytes;
                std::copy(src, src + 8, dest);
            }

            // normals
            if (indexNorm && normalsIn.size())
            {
                auto src = static_cast<const uint8_t*>(normalsIn[i]);
                auto dest = GetVertexAddr(i, vbStart) + offsetNormBytes;
                std::copy(src, src + 12, dest);
            }
        }

    }

    //GetIndexBufferMaster().WriteText("ib.txt");

    // normals
    if (normalsIn.empty())
        CalculateNormals(); //***TO DO*** draw the vertex normals so these can be checked

    // This is a bad idea, as calculated normals are smoothed
    //ReverseWindingsToMatchNormals();

    if (uvIn.empty())
        CalculateTextureCoordinates();

    // tangents and bitangents
    CalculateTangentsAndBiTangents();

    // update the bounding geometry
    CalculateBoundingBox();

    // return the mesh index
    return _meshes.size() - 1;
}
/******************************************************************************
*    Method:    operator= ; copy assignment
*       Defines a box with normals facing outward and vertex unwrapped as:
*
*   u ->     <--w-->
*          12------13                           v
*           |       |                          |
*           |   T   |                    d     v
*    <--d-->|       |<--d--> <--w-->
*   4-------5-------6-------7-------9   
*   |       |       |       |       |
*   |   L   |   F   |   R   |   Bk  |    h
*   |       |       |       |       |
*   0-------1-------2-------3-------8   
*           |       |
*           |   Bm  |                    d
*           |       |
*          10------11                   
*   0 = { -1.0f, -1.0f, -1.0f }, // lbb (min)
*   1 = { -1.0f, -1.0f,  1.0f }, // lbf
*   2 = { 1.0f, -1.0f,  1.0f },  // rbf
*   3 = { 1.0f, -1.0f, -1.0f },  // rbb
*   4 = { -1.0f,  1.0f, -1.0f }, // ltb
*   5 = { -1.0f, 1.0f, 1.0f },   // ltf
*   6 = { 1.0f,  1.0f,  1.0f },  // rtf (max)
*   7 = { 1.0f,  1.0f, -1.0f },  // rtb
*   8 = { -1.0f, -1.0f, -1.0f }, // lbb (min)
*   9 = { -1.0f,  1.0f, -1.0f }, // ltb
*  10 = { -1.0f, -1.0f, -1.0f }, // lbb (min)
*  11 = { 1.0f, -1.0f, -1.0f },  // rbb
*  12 = { -1.0f,  1.0f, -1.0f }, // ltb
*  13 = { 1.0f,  1.0f, -1.0f },  // rtb
******************************************************************************/
int King::Model::CreateMeshFrom(const King::Box& boxIn)
{
    // Model() does not get called if Model m = Box(); since Box is not a model and the reference moves the model
    // but, if you Model m(Box()); it will be called first
    if (!_materials.count("default"))
        AddMaterial(Material::Create("default"));
    if (!_vertexFormat.Has(VertexAttrib::enumDesc::position))
    {
        _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
    }
    _vertexFormat.SetNext(VertexAttrib::enumDesc::textureCoord, VertexAttrib::enumFormat::format_float32x2);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::normal, VertexAttrib::enumFormat::format_float32x3);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::tangent, VertexAttrib::enumFormat::format_float32x3);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::bitangent, VertexAttrib::enumFormat::format_float32x3);    
    _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
    assert(_vertexFormat.GetByteSize() == 56);

    _boundingBox = boxIn;
    
    // uv coordinates normalize to (0-1, 0-1) except v can be < 1 depending on proportions (if a box of whd = (1,1,1) then v_max = 0.75
    float3 whd = _boundingBox.GetDiagonal();
    float2 wd(whd.GetX(),whd.GetZ());
    wd.MakeNormalize(); wd *= 0.5f;

    float w = wd.GetX();
    float d = wd.GetY();
    float h = whd.GetY();
    float hTod = h / whd.GetZ();
    if (hTod > 2.0f) hTod = 2.0f;
    h = d * hTod;

    // get points
    DirectX::XMFLOAT3 vp[8];
    _boundingBox.GetCorners8(vp);

    // initialize vertex buffer
    _vertexBufferMaster.Initialize(14 * _vertexFormat.GetByteSize());
    _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
    auto size = _vertexBufferMaster.GetElements(); // 14

    // first eight
    for (size_t i = 0; i<8; ++i)
        SetVertexElement(i, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&vp[i]));
    // next six duplicates with different uv coordinates and normals
    SetVertexElement(8, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&vp[0]));
    SetVertexElement(9, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&vp[4]));
    SetVertexElement(10, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&vp[0]));
    SetVertexElement(11, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&vp[3]));
    SetVertexElement(12, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&vp[4]));
    SetVertexElement(13, VertexAttrib::enumDesc::position, reinterpret_cast<uint8_t*>(&vp[7]));

    // texture coordinates
    XMFLOAT2 uv;
    uv = { 0.f , d + h }; SetVertexElement(0, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { d , d + h }; SetVertexElement(1, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { d + w, d + h }; SetVertexElement(2, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { 1.0f - w, d + h }; SetVertexElement(3, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { 0.f , d }; SetVertexElement(4, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { d , d }; SetVertexElement(5, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { d + w, d }; SetVertexElement(6, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { 1.0f - w, d }; SetVertexElement(7, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { 1.f , d + h }; SetVertexElement(8, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { 1.f , d }; SetVertexElement(9, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { d, d + h + d }; SetVertexElement(10, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { d + w, d + h + d }; SetVertexElement(11, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { d, 0.f}; SetVertexElement(12, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));
    uv = { d + w, 0.f}; SetVertexElement(13, VertexAttrib::enumDesc::textureCoord, reinterpret_cast<uint8_t*>(&uv));

    // Triangles using RHS convention, normals outward
    _indexBufferMaster = { 0,1,4, 4,1,5, 5,1,2, 5,2,6, 6,2,3, 6,3,7, 7,3,8, 7,8,9, 1,10,11, 1,11,2, 12,5,6, 12,6,13 };

    // add normals
    CalculateNormals();
    // add tangents and bitangents
    CalculateTangentsAndBiTangents();
    // re-order verticies into the order they are accessed first to last in the index buffer
    OptimizeVertexBuffer();

    // Store the mesh
    auto mesh = TriangleMesh::Create(12, _vertexFormat, 0, 0, _vertexBufferMaster.GetData(), _indexBufferMaster.GetData());
    mesh->Set_name("triangleMesh");
    mesh->CalculateBoundingBox();
    _meshes.push_back(mesh);

    // just incase we create more than one mesh or append data to our master we will reset the pointer addresses in each mesh
    for (auto& m : _meshes)
    {
        m->SetVB(_vertexBufferMaster.GetData());
        m->SetIB(_indexBufferMaster.GetData());
    }

    return (int)_meshes.size() - 1;
}
/******************************************************************************
*    Method:    CreateMeshFrom
******************************************************************************/
int King::Model::CreateMeshFrom(const King::Sphere& sphereIn, const uint32_t segmentsIn)
{
    if (!_materials.count("default"))
        AddMaterial(Material::Create("default"));
    if (!_vertexFormat.Has(VertexAttrib::enumDesc::position))
    {
        _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
    }
    _vertexFormat.SetNext(VertexAttrib::enumDesc::textureCoord, VertexAttrib::enumFormat::format_float32x2);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::normal, VertexAttrib::enumFormat::format_float32x3);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::tangent, VertexAttrib::enumFormat::format_float32x3);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::bitangent, VertexAttrib::enumFormat::format_float32x3);
    _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
    assert(_vertexFormat.GetByteSize() == 56);

    // create our sphere by rotating a path around an axis
    Model &m = *this;

    const float radius = sphereIn.GetRadius();
    float theta = XM_2PI / (float)segmentsIn;
    Path p;

    const auto to = segmentsIn / (uint32_t)2 + (uint32_t)1;
    for (size_t j = 0; j < to; ++j)
    {
        auto a = theta * (float)j;
        const float cosTheta = cos(a);
        const float sinTheta = sin(a);
        float x = (sinTheta) * radius;
        float y = (cosTheta) * radius;
        p.push_back(float3(x, y, 0.f));
    }

    auto firstIndex = _meshes.size();

    for (size_t i = 0; i < segmentsIn; ++i)
    {
        quat rotQ(float3(0.f, 1.f, 0.f), theta);
        Path p2 = p * rotQ;

        auto index = m.CreateMeshFrom(p, p2);
        p = p2;

        m.MergeMeshes(firstIndex, index);

        //auto a = _indexBufferMaster.Max();
        //auto b = _vertexBufferMaster.GetElements();
        //assert(a < b);
    }

    // optimize
    auto rtn = RemoveDuplicateVerticies();

    auto vert0 = _meshes[firstIndex]->GetVertexPosition(0);
    assert(vert0.GetX() > -FLT_EPSILON);

    _meshes[firstIndex]->CalculateBoundingBox();

    Translate(sphereIn.GetCenter());

    // texture coordinates
    if (_vertexFormat.Has(VertexAttrib::enumDesc::textureCoord))
        HelperCreateTextureCoordinatesForSphere();

    return (int)firstIndex;
}

int King::Model::CreateMeshFrom(const King::Pyramid& pyramidIn)
{
    // Check and set up material
    if (!_materials.count("default"))
        AddMaterial(Material::Create("default"));

    // Set up vertex format if not already done
    if (!_vertexFormat.Has(VertexAttrib::enumDesc::position))
    {
        _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
    }
    _vertexFormat.SetNext(VertexAttrib::enumDesc::textureCoord, VertexAttrib::enumFormat::format_float32x2);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::normal, VertexAttrib::enumFormat::format_float32x3);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::tangent, VertexAttrib::enumFormat::format_float32x3);
    _vertexFormat.SetNext(VertexAttrib::enumDesc::bitangent, VertexAttrib::enumFormat::format_float32x3);
    _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
    assert(_vertexFormat.GetByteSize() == 56); // Ensure byte size matches expectations

    // Create our pyramid geometry
    Model& m = *this;

    // Get the base vertices and apex of the pyramid
    const std::vector<float3>& baseVertices = pyramidIn.GetBaseVertices();
    const float3& apex = pyramidIn.GetApex();

    // Indices for creating the mesh
    std::vector<uint32_t> indices;
    std::vector<float3> vertices;
    std::vector<float3> normals;

    // Add the base vertices to the vertex buffer
    vertices.insert(vertices.end(), baseVertices.begin(), baseVertices.end());

    // Add the apex to the vertex buffer
    vertices.push_back(apex);

    // Generate indices for the base of the pyramid (as a fan)
    const size_t baseVertexCount = baseVertices.size();
    const uint32_t apexIndex = static_cast<uint32_t>(vertices.size() - 1); // Last index is the apex

    // Create the base of the pyramid
    for (size_t i = 0; i < baseVertexCount; ++i)
    {
        uint32_t nextIndex = (i + 1) % baseVertexCount;
        indices.push_back(i);               // Current base vertex
        indices.push_back(nextIndex);       // Next base vertex
        indices.push_back(apexIndex);       // Apex
    }

    // Create the mesh from the generated vertices, indices, and normals
    auto meshIndex = HelperCreateMeshFrom(vertices, indices, {}, {});

    // Set the bounding box of the pyramid mesh
    _meshes[meshIndex]->CalculateBoundingBox();

    // Translate the mesh to the pyramid's position
    Translate(pyramidIn.GetCenter() - float3(0.f, pyramidIn.GetHeight() * 0.5f, 0.f));

    // Set texture coordinates if applicable
    if (_vertexFormat.Has(VertexAttrib::enumDesc::textureCoord))
        HelperCreateTextureCoordinatesForTriOrQuad(vertices, baseVertexCount);

    return static_cast<int>(meshIndex);
}

/******************************************************************************
*    Method:    operator= ; copy assignment
******************************************************************************/
King::LineModel & King::LineModel::operator= (const King::LineModel &in)
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

    HelperSetAllMeshesVBandIBtoMaster();
    return *this;
}
/******************************************************************************
*    Method:    operator= ; copy assignment
******************************************************************************/
inline King::Model & King::Model::operator= (const King::Model &in)
{
    Destroy();

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
    _materials = in._materials;

    for (auto & m : _meshes)
    {
        m->SetVB(_vertexBufferMaster.GetData());
        m->SetIB(_indexBufferMaster.GetData());
    }

    if (in._boneHierarchy) 
    { 
        _boneHierarchy = std::make_unique<BoneHierarchy>(); 
        *_boneHierarchy = *in._boneHierarchy; 
    }

    return *this;
}
/******************************************************************************
*    Method:    Load
******************************************************************************/
bool King::Model::Load(std::string fileNameIN)
{    
    std::filesystem::path p(fileNameIN);

    // native file format in binary
    if ((p.extension() == ".kng" || p.extension() == ".KNG" || p.extension() == ".Kng"))
    {
        std::vector<std::shared_ptr<Model>> models;
        ifstream infile(p, std::ios_base::binary);

        if (infile.is_open())
        {
            uint32_t magic;
            uint32_t version;
            uint32_t numModels;

            infile.read(reinterpret_cast<char*>(&magic), sizeof(magic));

            if (magic != 1971001974)
            {
                cout << "File: " << fileNameIN << " is not a .KNG native file" << '\n';
                return false;
            }
            else
            {
                //cout << "native .KNG file " << fileNameIN << '\n';

                infile.read(reinterpret_cast<char*>(&version), sizeof(version));
                infile.read(reinterpret_cast<char*>(&numModels), sizeof(numModels));

                if (infile.fail())
                {
                    cout << "  ERROR: Bad read from .KNG file format\n";
                    assert(0);
                    return false;
                }

                uint32_t maxVersion = 2;
                if (version > maxVersion)
                {
                    cout << "Version " << version << " of .KNG file format is not supported. Max is version " << maxVersion << "\n";
                    return false;
                }

                for (uint32_t i = 0; i < numModels; ++i)
                {
                    auto model = Model::Create();
                    if (version == 1) model->Read_v1(infile);
                    else if (version == 2) model->Read_v2(infile);

                    models.push_back(model);
                }

                if (infile.fail())
                {
                    cout << "  ERROR: Bad read from .KNG file format\n";
                    assert(0);
                    return false;
                }

                if (models.size())// just want one
                {
                    *this = *models.front(); // copies model contents to our model
                }
                if (models.size()) return true;
                else return false;
            }
        }
        infile.close();

    }
    // simple models and a very common file format for inter-operability
    else if ((p.extension() == ".obj" || p.extension() == ".OBJ" || p.extension() == ".Obj"))
    {
        Model_IO m;
        auto models = m.Load_OBJ(fileNameIN, &_vertexFormat); // returns a standard model
        if (models.size())// just want one
        {
            *this = *models.front(); // copies model contents to our model
        }
        if (models.size()) return true;
    }
    // complex models with bones and animation clips with a good readable file format but poorly supported (quake 3 arena format)
    else if ((p.extension() == ".m3d" || p.extension() == ".M3D" || p.extension() == ".M3d" || p.extension() == ".m3D"))
    {
        Model_IO m;
        //auto models = m.Load_M3D(fileNameIN, &_vertexFormat); // returns a standard model
        //if (models.size())// just want one
        //{
        //    *this = *models.front(); // model will ignore the bones and animation clips on copy
        //}
        //if (models.size()) return true;
    }

    return false;
}
/******************************************************************************
*    Method:    Save
******************************************************************************/
bool King::Model::Save(std::string fileNameIN)
{ 
    bool rtn(false);

    std::filesystem::path p(fileNameIN);

    // native file format in binary
    if ((p.extension() == ".kng" || p.extension() == ".KNG" || p.extension() == ".Kng"))
    {
        ofstream outfile;
        outfile.open(fileNameIN, std::ios_base::binary | std::ios_base::trunc);
        //cout << "Saving file: " << fileNameIN << '\n';

        if (outfile.is_open())
        {
            uint32_t magic = 1971001974; // file type identifier
            uint32_t version = 2;
            uint32_t numModels = 1;

            outfile.write(reinterpret_cast<char*>(&magic), sizeof(magic));
            outfile.write(reinterpret_cast<char*>(&version), sizeof(version));
            outfile.write(reinterpret_cast<char*>(&numModels), sizeof(numModels));

            for (uint32_t i = 0; i < numModels; ++i)
            {
                if (version == 1) Write_v1(outfile);
                else if (version == 2) Write_v2(outfile);
            }

            if (outfile.fail()) return false;
        }
        outfile.close();

        return true;
    }
    else if ((p.extension() == ".obj" || p.extension() == ".OBJ" || p.extension() == ".Obj"))
    {
        Model_IO m;
        std::vector<std::shared_ptr<ModelScaffold>> models;
        models.push_back(std::static_pointer_cast<Model>(shared_from_this()));

        rtn = m.Save_OBJ(fileNameIN, models);
        if (rtn)
            rtn = m.Save_MTL(fileNameIN, models);
        return rtn;
    }

    return rtn;
}
/******************************************************************************
*    Method:    Read_v1
*       Read from an open binary file
******************************************************************************/
bool King::Model::Read_v1(ifstream& dataFileIn)
{
    if (!dataFileIn.is_open())
    {
        assert(0);
        return false;
    }
    // lambda
    auto ReadString = [&dataFileIn](string& strOut)
    {
        size_t len;
        dataFileIn.read(reinterpret_cast<char*>(&len), sizeof(len));
        char* temp = new char[len + 1];
        dataFileIn.read(temp, len);
        temp[len] = '\0';
        strOut = temp;
        delete[] temp;
    };
    // read contents
    uint32_t numMaterials;
    dataFileIn.read(reinterpret_cast<char*>(&numMaterials), sizeof(numMaterials));

    uint32_t numMeshes;
    dataFileIn.read(reinterpret_cast<char*>(&numMeshes), sizeof(numMeshes));

    bool readRtn = ModelScaffold::Read_v1(dataFileIn);
    if (dataFileIn.fail() || !readRtn)
    {
        assert(0);
        return false;
    }

    bool good = true;
    string key;

    while (dataFileIn && good && numMaterials)
    {
        auto mtl = Material::Create();
        
        ReadString(key);
        good = mtl->Read_v1(dataFileIn);

        // store
        _materials[key] = mtl;

        --numMaterials;
    }
    if (dataFileIn.fail() || !good)
    {
        assert(0);
        return false;
    }

    while (dataFileIn && good && numMeshes)
    {
        auto mesh = TriangleMesh::Create();

        good = mesh->Read_v1(dataFileIn);

        // set reference data
        mesh->SetVertexFormat(GetVertexFormat());
        mesh->SetVB(GetVertexBufferMaster().GetData());
        mesh->SetIB(GetIndexBufferMaster().GetData());
        // store
        _meshes.push_back(mesh);

        --numMeshes;
    }

    if (dataFileIn.fail() || !good)
    {
        return false;
    }

    return true;
}
/******************************************************************************
*    Method:    Write_v1
*       Write to an open binary file
******************************************************************************/
bool King::Model::Write_v1(ofstream& outfileIn)
{
    if (!outfileIn.is_open()) return false;
    // lambda
    auto WriteString = [&outfileIn](const string& str)
    {
        size_t len = str.size();
        outfileIn.write(reinterpret_cast<const char*>(&len), sizeof(len));
        outfileIn.write(str.c_str(), len);
    };
    // write contents
    uint32_t numMaterials = (uint32_t)_materials.size();
    outfileIn.write(reinterpret_cast<const char*>(&numMaterials), sizeof(numMaterials));

    uint32_t numMeshes = (uint32_t)_meshes.size();
    outfileIn.write(reinterpret_cast<const char*>(&numMeshes), sizeof(numMeshes));

    ModelScaffold::Write_v1(outfileIn);

    for (const auto &mtl : _materials)
    {
        WriteString(mtl.first);
        mtl.second->Write_v1(outfileIn);
    }

    for (const auto &mesh : _meshes)
    {
        mesh->Write_v1(outfileIn);
    }

    if (outfileIn.fail()) return false;

    return true;
}
/******************************************************************************
*    Method:    Read_v2
*       Read from an open binary file
******************************************************************************/
bool King::Model::Read_v2(ifstream& dataFileIn)
{
    if (!dataFileIn.is_open())
    {
        return false;
    }
    // lambda
    auto ReadString = [&dataFileIn](string& strOut)
    {
        size_t len;
        dataFileIn.read(reinterpret_cast<char*>(&len), sizeof(len));
        char* temp = new char[len + 1];
        dataFileIn.read(temp, len);
        temp[len] = '\0';
        strOut = temp;
        delete[] temp;
    };
    // read contents
    uint32_t numMaterials;
    dataFileIn.read(reinterpret_cast<char*>(&numMaterials), sizeof(numMaterials));

    uint32_t numMeshes;
    dataFileIn.read(reinterpret_cast<char*>(&numMeshes), sizeof(numMeshes));

    bool readRtn = ModelScaffold::Read_v1(dataFileIn);
    if (dataFileIn.fail() || !readRtn)
    {
        assert(0);
        return false;
    }

    bool good = true;
    string key;

    while (dataFileIn && good && numMaterials)
    {
        auto mtl = Material::Create();

        ReadString(key);
        good = mtl->Read_v1(dataFileIn);

        // store
        _materials[key] = mtl;

        --numMaterials;
    }
    if (dataFileIn.fail() || !good)
    {
        assert(0);
        return false;
    }

    while (dataFileIn && good && numMeshes)
    {
        auto mesh = TriangleMesh::Create();

        good = mesh->Read_v1(dataFileIn);

        // set reference data
        mesh->SetVertexFormat(GetVertexFormat());
        mesh->SetVB(GetVertexBufferMaster().GetData());
        mesh->SetIB(GetIndexBufferMaster().GetData());
        // store
        _meshes.push_back(mesh);

        --numMeshes;
    }

    if (dataFileIn.fail() || !good)
    {
        return false;
    }

    // optional bone hierarchy data
    {
        _boneHierarchy = BoneHierarchy::CreateUnique();
        good = _boneHierarchy->Read_v1(dataFileIn);

        if (dataFileIn.fail() || !good)
            _boneHierarchy.reset();
    }

    return true;
}
/******************************************************************************
*    Method:    Write_v2
*       Write to an open binary file
*       Version 2: includes optional bone hierarchy data
******************************************************************************/
bool King::Model::Write_v2(ofstream& outfileIn)
{
    if (!outfileIn.is_open()) return false;
    // lambda
    auto WriteString = [&outfileIn](const string& str)
    {
        size_t len = str.size();
        outfileIn.write(reinterpret_cast<const char*>(&len), sizeof(len));
        outfileIn.write(str.c_str(), len);
    };
    // write contents
    uint32_t numMaterials = (uint32_t)_materials.size();
    outfileIn.write(reinterpret_cast<const char*>(&numMaterials), sizeof(numMaterials));

    uint32_t numMeshes = (uint32_t)_meshes.size();
    outfileIn.write(reinterpret_cast<const char*>(&numMeshes), sizeof(numMeshes));

    ModelScaffold::Write_v1(outfileIn);

    for (const auto& mtl : _materials)
    {
        WriteString(mtl.first);
        mtl.second->Write_v1(outfileIn);
    }

    for (const auto& mesh : _meshes)
    {
        mesh->Write_v1(outfileIn);
    }

    if (_boneHierarchy)
        if (!_boneHierarchy->Write_v1(outfileIn)) return false;

    if (outfileIn.fail()) return false;

    return true;
}
/******************************************************************************
*    Method:    CalculateBoundingBox
******************************************************************************/
Box King::LineModel::CalculateBoundingBox()
{
    if (!_meshes.size())
    {
        _boundingBox.SetZero();
        return _boundingBox;
    }

    _boundingBox = _meshes[0]->CalculateBoundingBox();
    for (auto& mesh : _meshes)
    {
        _boundingBox.Merge(mesh->CalculateBoundingBox());
    }

    return _boundingBox;
}
/******************************************************************************
*    Method:    CreateMesh
******************************************************************************/
inline std::shared_ptr<LineMesh> King::LineModel::CreateMesh(uint32_t numLinesIn, uint32_t vbStartIn, uint32_t ibStartIn)
{
    assert((bool)GetVertexBufferMaster);
    assert((bool)GetIndexBufferMaster);
    auto ptr = LineMesh::Create(numLinesIn, GetVertexFormat(), vbStartIn, ibStartIn, GetVertexBufferMaster().GetData(), GetIndexBufferMaster().GetData());
    _meshes.push_back(ptr);
    return _meshes.back();
}
/******************************************************************************
*    Method:    CalculateBoundingBox
******************************************************************************/
Box King::Model::CalculateBoundingBox() 
{
    if (!_meshes.size())
    {
        _boundingBox.SetZero();
        return _boundingBox;
    }

    _boundingBox = _meshes[0]->CalculateBoundingBox();
    for (auto & mesh : _meshes)
    {
        _boundingBox.Merge(mesh->CalculateBoundingBox());
    }

    return _boundingBox;
}
/******************************************************************************
*    Method:    CalculateNormals
*       For the entire model, not per mesh
******************************************************************************/
void King::Model::CalculateNormals()
{
    if (!(_vertexFormat.Has(VertexAttrib::position) && _vertexFormat.Has(VertexAttrib::normal)))
        return;

    auto& ib = _indexBufferMaster;
    auto& vb = _vertexBufferMaster;
    assert(ib);
    assert(vb);
    const auto positionAttribute = _vertexFormat.GetAttribute(_vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::position));
    const auto normalAttribute = _vertexFormat.GetAttribute(_vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::normal));
    // zero out normals
    for (size_t i = 0; i < ib.GetElements(); ++i)
    {
        auto n = reinterpret_cast<float*>(&vb[ib[i]] + normalAttribute.GetOffset());
        n[0] = 0.0f; n[1] = 0.0f; n[2] = 0.0f;
    }
    // add face normal to vertex
    for (size_t i = 0; i < ib.GetElements(); i += 3)
    {
        /*
            5   4   3      4    5    4     3    4    3
            ---------           -----           -----
            |  /|  /|     /|    |  /      /|    |  /
            | / | / | =  / |  + | /   +  / |  + | /
            |/  |/  |   /  |    |/      /  |    |/
            ---------  -----           -----
            0   1   2  0   1    0      1   2    1

            f 0 1 2 3 4 5
         */
        auto addr0 = &vb[ib[i + 0]];
        auto addr1 = &vb[ib[i + 1]];
        auto addr2 = &vb[ib[i + 2]];
        const float3 pt0(reinterpret_cast<float*>(addr0 + positionAttribute.GetOffset()));
        const float3 pt1(reinterpret_cast<float*>(addr1 + positionAttribute.GetOffset()));
        const float3 pt2(reinterpret_cast<float*>(addr2 + positionAttribute.GetOffset()));
        // #1f = 0 1 4, #2f = 0 4 5
        // face normal
        auto fnFaceNormal = [](float3 a, float3 b, float3 c) {return Cross((b - a), (c - a)); };

        float3 c = fnFaceNormal(pt0, pt1, pt2);
        // reverse normal if not outward from the center of the object
        //if (c.DotProduct(pt0 - center) < 0.f)
        //{
        //    c *= -1.0f;
        //    // reverse winding to match normal
        //    auto temp = ib[i + 1];
        //    ib[i + 1] = ib[i + 2];
        //    ib[i + 2] = temp;
        //}
        {
            float* n0 = reinterpret_cast<float*>(addr0 + normalAttribute.GetOffset());
            float3 n(n0); n += c;
            n0[0] = n.GetX(); n0[1] = n.GetY(); n0[2] = n.GetZ();
        }
        {
            float* n1 = reinterpret_cast<float*>(addr1 + normalAttribute.GetOffset());
            float3 n(n1); n += c;
            n1[0] = n.GetX(); n1[1] = n.GetY(); n1[2] = n.GetZ();
        }
        {
            float* n2 = reinterpret_cast<float*>(addr2 + normalAttribute.GetOffset());
            float3 n(n2); n += c;
            n2[0] = n.GetX(); n2[1] = n.GetY(); n2[2] = n.GetZ();
        }
    }
    // normalize normals (effectively smoothing the normals of verticies that are shared with multiple faces
    for (size_t i = 0; i < ib.GetElements(); ++i)
    {
        float* ni = reinterpret_cast<float*>(&vb[ib[i]] + normalAttribute.GetOffset());
        float3 n(ni);
        n.MakeNormalize();
        ni[0] = n.GetX(); ni[1] = n.GetY(); ni[2] = n.GetZ();

//        cout << "Normal " << to_string(i) << ": " << n << '\n';
    }

    // validate they are smooth and not reversing
    if (false)
    {
        float* n0 = reinterpret_cast<float*>(&vb[ib[0]] + normalAttribute.GetOffset());
        float3 prev(n0);
        for (size_t i = 1; i < ib.GetElements(); ++i)
        {
            float* ni = reinterpret_cast<float*>(&vb[ib[i]] + normalAttribute.GetOffset());
            float3 n(ni);
            assert(n.DotProduct(prev) >= 0.f);
            prev = ni;
        }
    }
}
void King::Model::CalculateTextureCoordinates()
{
    if (!(_vertexFormat.Has(VertexAttrib::position) &&
        _vertexFormat.Has(VertexAttrib::textureCoord)))
        return;

    const auto& ib = _indexBufferMaster;
    auto& vb = _vertexBufferMaster;
    const auto textureCoordAttribute = _vertexFormat.GetAttribute(_vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::textureCoord));

    auto temp = GetPositions();
    size_t i;
    assert(temp.size() > 3);
    for (i = 0; i < temp.size() - 3; i += 4)
    {
        std::vector<King::float3> pts;
        pts.push_back(float3(temp[i]));
        pts.push_back(float3(temp[i + 1]));
        pts.push_back(float3(temp[i + 2]));
        pts.push_back(float3(temp[i + 3]));

        auto uvQuad = King::Model::HelperCreateTextureCoordinatesForTriOrQuad(pts, 1);

        auto uv0 = reinterpret_cast<float*>(&vb[ib[i + 0]] + textureCoordAttribute.GetOffset());
        auto uv1 = reinterpret_cast<float*>(&vb[ib[i + 1]] + textureCoordAttribute.GetOffset());
        auto uv2 = reinterpret_cast<float*>(&vb[ib[i + 2]] + textureCoordAttribute.GetOffset());
        auto uv3 = reinterpret_cast<float*>(&vb[ib[i + 3]] + textureCoordAttribute.GetOffset());

        uv0[0] = uvQuad[0].GetX();
        uv0[1] = uvQuad[0].GetY();

        uv1[0] = uvQuad[1].GetX();
        uv1[1] = uvQuad[1].GetY();

        uv2[0] = uvQuad[2].GetX();
        uv2[1] = uvQuad[2].GetY();

        uv3[0] = uvQuad[3].GetX();
        uv3[1] = uvQuad[3].GetY();
    }
    // do we have a triangle at the end and not a full quad?
    // 0 1 2 3
    // 0 1 2 3 4 5 6
    if (i < temp.size() - 3)
    {
        std::vector<King::float3> pts;
        pts.push_back(float3(temp[i]));
        pts.push_back(float3(temp[i + 1]));
        pts.push_back(float3(temp[i + 2]));

        auto uvTri = King::Model::HelperCreateTextureCoordinatesForTriOrQuad(pts, 1); // Does this really support tris?

        auto uv0 = reinterpret_cast<float*>(&vb[ib[i + 0]] + textureCoordAttribute.GetOffset());
        auto uv1 = reinterpret_cast<float*>(&vb[ib[i + 1]] + textureCoordAttribute.GetOffset());
        auto uv2 = reinterpret_cast<float*>(&vb[ib[i + 2]] + textureCoordAttribute.GetOffset());

        uv0[0] = uvTri[0].GetX();
        uv0[1] = uvTri[0].GetY();

        uv1[0] = uvTri[1].GetX();
        uv1[1] = uvTri[1].GetY();

        uv2[0] = uvTri[2].GetX();
        uv2[1] = uvTri[2].GetY();
    }
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

    for (size_t i = 0; i < ib.GetElements(); i+=3)
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
*    Method:    MergeMeshes
*       Re-order indicies such that the two buffers are next to one another
*       and then set their start position (zero base) the same, changing the
*       index values as necessary.
*       0   1   2...5   6   7   0   1   2   0   1   0   1   2
*       -----------------------------------------------------
*       |       mesh0        |    mesh1   | mesh2 |  mesh3  |
*
*       Ex: Merge mesh1 & mesh 3 with elements in mesh2 < mesh3
*       -----------------------------------------------------
*       |       mesh0        |    mesh1+mesh3       | mesh2 |
*       Procedure is this then:
* /*      -----------------------------------------------------
*       |                  _indexBufferMaster               |
*       | _indexBufferMaster |              split           |
*       |       mesh0        |    meshF   | mesh2 |  meshS  |
*       |       mesh0        |    meshF   |   tempBuffer1   |
*       |       mesh0        |    meshF   |  meshS  | mesh2 | 
*       |       mesh0        |    mesh1 + mesh3     | mesh2 |
*       |                  _indexBufferMaster               | 
******************************************************************************/
void King::Model::MergeMeshes(const size_t meshIndex1, const size_t meshIndex2)
{
    if (meshIndex1 == meshIndex2) return;
    assert(meshIndex1 < _meshes.size());
    assert(meshIndex2 < _meshes.size());
    
    size_t firstMesh = min(meshIndex1, meshIndex2);
    size_t secondMesh = max(meshIndex1, meshIndex2);
    auto meshF = GetMesh(firstMesh);
    auto meshS = GetMesh(secondMesh);

    // starting with the first mesh and ending with the last index of the model
    // copy the data into a temporary buffer
    uint32_t splitSizeLHS(0);
    for (size_t i = 0; i < firstMesh; ++i)
        splitSizeLHS += _meshes[i]->GetNumIndicies();

    // merge the meshes by moving the indicies around
    {
        // split [_indexBufferMaster] into [_indexBufferMaster] + [split]
        MemoryBlock<uint32_t> split;
        _indexBufferMaster.Split(splitSizeLHS, &split);

        if (split)
        {
            // the split might reallocate, adjust mesh references
            HelperSetAllMeshesVBandIBtoMaster();

            // copy data occurring after firstMesh
            uint32_t splitSizeRHS(0);
            for (size_t i = firstMesh + 1; i < _meshes.size(); ++i)
                splitSizeRHS += _meshes[i]->GetNumIndicies();

            MemoryBlock<uint32_t> tempBuffer1(splitSizeRHS);
            tempBuffer1.Copy(0, split.GetData() + meshF->GetNumIndicies(), splitSizeRHS); // copy into tempBuffer1

            // how many elements are between meshF and meshS? 
            const uint32_t meshesToSkip = secondMesh - firstMesh - 1;
            uint32_t elementsToSkipIntempBuffer1(0);
            for (size_t i = 0; i < meshesToSkip; ++i)
                elementsToSkipIntempBuffer1 += _meshes[firstMesh + 1 + i]->GetNumIndicies();

            // copy from tempBuffer1 into split to move meshS next to meshF
            split.Copy(meshF->GetNumIndicies(), tempBuffer1.GetData() + elementsToSkipIntempBuffer1, meshS->GetNumIndicies());

            // copy from tempBuffer1 the skipped meshs to after meshS
            if (elementsToSkipIntempBuffer1)
                split.Copy((size_t)meshF->GetNumIndicies() + (size_t)meshS->GetNumIndicies(), tempBuffer1.GetData(), elementsToSkipIntempBuffer1);
            
            _indexBufferMaster.Merge(split);
            // the merge reallocates, adjust mesh references
            HelperSetAllMeshesVBandIBtoMaster();
        }
    }
    // add the vertex start delta offset (between the first and second mesh) to the moved second mesh indicies
    int adj(0);
    if (meshF->GetVBStart() != meshS->GetVBStart())
        adj += meshS->GetVBStart() - meshF->GetVBStart();
    const auto offset = splitSizeLHS + meshF->GetNumIndicies();
    const auto items = meshS->GetNumIndicies();
    for (uint32_t i = offset; i < offset + items; ++i)
        _indexBufferMaster[i] += adj;

    // adjust combined mesh to match the merge
    auto mesh1 = GetMesh(meshIndex1);
    mesh1->SetNumIndicies(meshF->GetNumIndicies() + meshS->GetNumIndicies());
    mesh1->SetIBStart(meshF->GetIBStart());
    mesh1->SetVBStart(meshF->GetVBStart());
    mesh1->Set_materialName(meshF->Get_materialName());
    mesh1->Set_name(meshF->Get_name());
    mesh1->CalculateBoundingBox();
    // remove meshS to finish
    _meshes.erase(std::next(_meshes.begin(), meshIndex2));

    HelperSetAllMeshesVBandIBtoMaster();
}
/******************************************************************************
*    Method:    MergeMeshes
*       Re-order indicies such that the two buffers are next to one another
*       and then set their start position (zero base) the same, changing the
*       index values as necessary.
*       0   1   2...5   6   7   0   1   2   0   1   0   1   2
*       -----------------------------------------------------
*       |       mesh0        |    mesh1   | mesh2 |  mesh3  |
*
*       Ex: Merge mesh1 & mesh 3 with elements in mesh2 < mesh3
*       -----------------------------------------------------
*       |       mesh0        |    mesh1+mesh3       | mesh2 |
*       Procedure is this then:
* /*      -----------------------------------------------------
*       |                  _indexBufferMaster               |
*       | _indexBufferMaster |              split           |
*       |       mesh0        |    meshF   | mesh2 |  meshS  |
*       |       mesh0        |    meshF   |   tempBuffer1   |
*       |       mesh0        |    meshF   |  meshS  | mesh2 |
*       |       mesh0        |    mesh1 + mesh3     | mesh2 |
*       |                  _indexBufferMaster               |
* 
* NOTE: THIS IS IDENTICAL TO Model::MergeMeshes except no materials to handle
******************************************************************************/
void King::LineModel::MergeMeshes(const size_t meshIndex1, const size_t meshIndex2)
{
    if (meshIndex1 == meshIndex2) return;
    assert(meshIndex1 < _meshes.size());
    assert(meshIndex2 < _meshes.size());

    size_t firstMesh = min(meshIndex1, meshIndex2);
    size_t secondMesh = max(meshIndex1, meshIndex2);
    auto meshF = GetMesh(firstMesh);
    auto meshS = GetMesh(secondMesh);

    // starting with the first mesh and ending with the last index of the model
    // copy the data into a temporary buffer
    uint32_t splitSizeLHS(0);
    for (size_t i = 0; i < firstMesh; ++i)
        splitSizeLHS += _meshes[i]->GetNumIndicies();

    // merge the meshes by moving the indicies around
    {
        // split [_indexBufferMaster] into [_indexBufferMaster] + [split]
        MemoryBlock<uint32_t> split;
        _indexBufferMaster.Split(splitSizeLHS, &split);

        if (split)
        {
            // the split reallocates, adjust mesh references
            HelperSetAllMeshesVBandIBtoMaster();

            // copy data occurring after firstMesh
            uint32_t splitSizeRHS(0);
            for (size_t i = firstMesh + 1; i < _meshes.size(); ++i)
                splitSizeRHS += _meshes[i]->GetNumIndicies();

            MemoryBlock<uint32_t> tempBuffer1(splitSizeRHS);
            tempBuffer1.Copy(0, split.GetData() + meshF->GetNumIndicies(), splitSizeRHS);

            // how many elements are between meshF and meshS? 
            const uint32_t meshesToSkip = secondMesh - firstMesh - 1;
            uint32_t elementsToSkipIntempBuffer1(0);
            for (size_t i = 0; i < meshesToSkip; ++i)
                elementsToSkipIntempBuffer1 += _meshes[firstMesh + 1 + i]->GetNumIndicies();

            // copy from tempBuffer1 into split to move meshS next to meshF
            split.Copy(meshF->GetNumIndicies(), tempBuffer1.GetData() + elementsToSkipIntempBuffer1, meshS->GetNumIndicies());

            // copy from tempBuffer1 the skipped meshs to after meshS
            if (elementsToSkipIntempBuffer1)
                split.Copy(meshF->GetNumIndicies() + meshS->GetNumIndicies(), tempBuffer1.GetData(), elementsToSkipIntempBuffer1);

            _indexBufferMaster.Merge(split);
            // the merge reallocates, adjust mesh references
            for (auto ea : _meshes)
            {
                ea->SetIB(_indexBufferMaster.GetData());
                ea->SetVB(_vertexBufferMaster.GetData());
            }
        }
    }
    // add the vertex start delta offset (between the first and second mesh) to the moved second mesh indicies
    int adj(0);
    if (meshF->GetVBStart() != meshS->GetVBStart())
        adj += meshS->GetVBStart() - meshF->GetVBStart();
    const auto offset = splitSizeLHS + meshF->GetNumIndicies();
    const auto items = meshS->GetNumIndicies();
    for (uint32_t i = offset; i < offset + items; ++i)
        _indexBufferMaster[i] += adj;

    // adjust cobined mesh to match the merge
    auto mesh1 = GetMesh(meshIndex1);
    mesh1->SetNumIndicies(meshF->GetNumIndicies() + meshS->GetNumIndicies());
    mesh1->SetIBStart(meshF->GetIBStart());
    mesh1->SetVBStart(meshF->GetVBStart());
    mesh1->Set_name(meshF->Get_name());
    mesh1->CalculateBoundingBox();
    // remove meshS to finish
    _meshes.erase(std::next(_meshes.begin(), meshIndex2));

    // performed above
    //HelperSetAllMeshesVBandIBtoMaster();
}
/******************************************************************************
*    Method:    OptimizeMeshVertexBuffer
*       Re-order vertex to match index buffer ordering from first encountered
*       to last encountered.
*       2   3   5   7   9   11
*       ---------------------
*       |\  |\  |\  |\  |\  |
*       | \ | \ | \ | \ | \ |
*       |  \|  \|  \|  \|  \|
*       ---------------------
*       0   1   4   6   8   10
*       
*       CCW index order then is: 0,1,2 2,1,3 1,4,3 3,4,5 4,6,5 5,6,7
*       Vertex order would be optimized to: 0,1,2,3,4,5,6, etc.
******************************************************************************/
void King::Model::OptimizeMeshVertexBuffer(const TriangleMesh & forMeshIn)
{
    vector<uint8_t> new_verts;
    vector<uint32_t> newIndexBuffer;
    auto ib = forMeshIn.GetIB();
    const auto & ib_start = forMeshIn.GetIBStart();
   
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
            auto value = *(ib + i + ib_start); // added ib_start 7/16/2021 and untested.  Before it was assumed always zero.  Copy below adjusted too.

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
        copy(new_verts.begin(), new_verts.end(), forMeshIn.GetVB() + stride * forMeshIn.GetVBStart());
    // copy remapped index buffer
    if (meshNumIndex == newIndexBuffer.size())
        copy(newIndexBuffer.begin(), newIndexBuffer.end(), ib + forMeshIn.GetIBStart());

    //cout << "OptimizeVertexBuffer" << '\n';
    //cout << "  " << "Verticies before: " << meshNumVerts << '\n';
    //cout << "  " << "Verticies after:  " << new_verts.size()/stride << '\n';
    //cout << "  " << "Verticies reordered: " << vertexSwapped << '\n';
    //cout << "  " << "Indicies before: " << meshNumIndex << '\n';
    //cout << "  " << "Indicies after: " << newIndexBuffer.size() << '\n';
    //cout << "  " << "Indicies remapped: " << indexRemapped << '\n';
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
            file << "# " << "  " << "Name: " << mesh->Get_name() << '\n' ;
            file << "# " << "  " << "Material: " << mesh->Get_materialName() << '\n' ;
            file << "# " << "  " << "Triangles: " << std::to_string(mesh->GetNumTriangles()) << '\n' ;
            file << "# " << "  " << "VB start: " << std::to_string(mesh->GetVBStart()) << '\n' ;
            file << "# " << "  " << "IB start: " << std::to_string(mesh->GetIBStart()) << '\n' ;
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
*    Method:    ReverseNormals
******************************************************************************/
void King::Model::ReverseNormals()
{
    if (!HasNormals()) return;

    auto stride = GetVertexBufferMaster().GetStride();
    size_t numElements = GetVertexBufferMaster().GetElements();

    auto start = GetVertexFormat().GetAttributeByteStart(GetVertexFormat().GetAttributeIndexFromDescription(VertexAttrib::enumDesc::normal));
    auto* n = reinterpret_cast<XMFLOAT3*>(reinterpret_cast<char*>(GetVertexBufferMaster().GetData()) + start);

    float3 intrinsic;
    for (size_t i = 0; i < numElements; i++)
    {
        intrinsic.Set(*n);
        *n = -intrinsic;
        n = reinterpret_cast<XMFLOAT3*>((reinterpret_cast<char*>(n) + stride));
    }
}
/******************************************************************************
*    Method:    ReverseWindingsToMatchNormals
******************************************************************************/
void King::Model::ReverseWindingsToMatchNormals()
{

    auto fnFaceNormal = [](float3 a, float3 b, float3 c) {return Cross((b - a), (c - a)); };
    auto fnAveNormals = [](float3 a, float3 b, float3 c) {return Normalize(a + b + c); };

    auto startP = GetVertexFormat().GetAttributeByteStart(GetVertexFormat().GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position));
    auto startN = GetVertexFormat().GetAttributeByteStart(GetVertexFormat().GetAttributeIndexFromDescription(VertexAttrib::enumDesc::normal));
        
    auto stride = GetVertexBufferMaster().GetStride();
    auto* p = (reinterpret_cast<char*>(GetVertexBufferMaster().GetData()) + startP);
    auto* n = (reinterpret_cast<char*>(GetVertexBufferMaster().GetData()) + startN);

    auto* indicies = reinterpret_cast<uint32_t*>(reinterpret_cast<char*>(GetIndexBufferMaster().GetData()));
    const uint32_t numElements = (uint32_t)GetIndexBufferMaster().GetElements();

    uint32_t numReversed = 0;

    for (uint32_t i = 0; i < numElements; i += 3)
    {
        float3 v0 = *reinterpret_cast<XMFLOAT3*>(p + indicies[i] * stride);
        float3 v1 = *reinterpret_cast<XMFLOAT3*>(p + indicies[i + 1] * stride);
        float3 v2 = *reinterpret_cast<XMFLOAT3*>(p + indicies[i + 2] * stride);
        auto faceNormal = fnFaceNormal(v0, v1, v2);

        float3 n0 = *reinterpret_cast<XMFLOAT3*>(n + indicies[i] * stride);
        float3 n1 = *reinterpret_cast<XMFLOAT3*>(n + indicies[i + 1] * stride);
        float3 n2 = *reinterpret_cast<XMFLOAT3*>(n + indicies[i + 2] * stride);
        auto aveNormal = fnAveNormals(n0, n1, n2);

        float mustBePositive = Dot(faceNormal, aveNormal);
        if (fabsf(mustBePositive) > mustBePositive + FLT_EPSILON)
        {
            ++numReversed;
            auto temp = indicies[i + 1];
            indicies[i + 1] = indicies[i + 2];
            indicies[i + 2] = temp;
        }
    }
    //cout << "ReverseWindingsToMatchNormals: " << numReversed << '\n';

}
/******************************************************************************
*    Method:    ReverseWindings
*       Flip the winding order so that CCW becomes CW and CW becomes CCW
*       Operates on the indicies of the index buffer
******************************************************************************/
void King::Model::ReverseWindings()
{
    if (HasPositions())
    {
        auto* indicies = GetIndexBufferMaster().GetData();
        const auto numElements = (uint32_t)GetIndexBufferMaster().GetElements();

        // Triangles
        assert(GetIndexBufferMaster().GetElements() % 3 == 0);

        for (uint32_t i = 0; i < numElements; i += 3)
        {
            auto temp = indicies[i + 1];
            indicies[i + 1] = indicies[i + 2];
            indicies[i + 2] = temp;
        }
    }
}
/******************************************************************************
*    Method:    RemoveUnusedVerticies
******************************************************************************/
size_t King::Model::RemoveUnusedVerticies()
{
    size_t rtn(0);
    // pre-pass
    // makes all indicies to the unique buffer the same by removing the vertex start offset of each mesh
    for (auto& mesh : _meshes)
    {
        for (uint32_t f = 0; f < mesh->GetNumIndicies(); ++f)
        {
            auto indexAddr = mesh->GetIB() + f;
            auto indexValue = *(indexAddr);

            indexValue += mesh->GetVBStart();

            assert(indexValue < _vertexBufferMaster.GetElements());
            *(indexAddr) = indexValue;
        }
        mesh->SetVBStart(0);
    }

    // create a list of unique verticies
    auto stride = _vertexBufferMaster.GetStride();
    MemoryBlock<uint8_t> uniqueVB(_vertexBufferMaster.GetElements(), stride);
    uint32_t numUniqueVerts(0);

    // main comparison pass
    uint32_t numIndicies(_indexBufferMaster.GetElements());
    uint32_t numOfVerticies(_vertexBufferMaster.GetElements());

    for (uint32_t k = 0; k < numOfVerticies; ++k)
        for (uint32_t s = 0; s < numIndicies; ++s)
        {
            if (_indexBufferMaster[s] == k)
            {
                // in use, keep it
                uniqueVB.Copy(numUniqueVerts, &_vertexBufferMaster[k]);

                // remap this index, k, into our unique buffer
                for (uint32_t f = 0; f < numIndicies; ++f)
                {
                    //if (f == 767) __debugbreak();
                    if (_indexBufferMaster[f] == k)
                        _indexBufferMaster[f] = numUniqueVerts;
                }
                ++numUniqueVerts;
                break;
            }
        }

    rtn = _vertexBufferMaster.GetElements() - uniqueVB.GetElements();
    _vertexBufferMaster = uniqueVB;

    HelperSetAllMeshesVBandIBtoMaster();

    return rtn;
}
/******************************************************************************
*    Model:    RemoveDuplicateVerticies
*       Returns: number of vierticies removed
******************************************************************************/
size_t King::Model::RemoveDuplicateVerticies()
{
    size_t rtn(0);

    // attributes that must match to be equal (exclude parameters like normals)
    bool hasP(HasPositions());
    bool hasT(HasTextureCoordinates());
    bool hasC(HasColors());

    auto startP = GetVertexFormat().GetAttributeByteStart(GetVertexFormat().GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position));
    auto startT = GetVertexFormat().GetAttributeByteStart(GetVertexFormat().GetAttributeIndexFromDescription(VertexAttrib::enumDesc::textureCoord));
    auto startC = GetVertexFormat().GetAttributeByteStart(GetVertexFormat().GetAttributeIndexFromDescription(VertexAttrib::enumDesc::color));

    auto stride = _vertexBufferMaster.GetStride();
    auto p = _vertexBufferMaster.GetData() + startP;
    auto t = _vertexBufferMaster.GetData() + startT;
    auto c = _vertexBufferMaster.GetData() + startC;

    // create a list of unique verticies
    MemoryBlock<uint8_t> uniqueVB(_vertexBufferMaster.GetElements(), stride);

    auto pu = uniqueVB.GetData() + startP;
    auto tu = uniqueVB.GetData() + startT;
    auto cu = uniqueVB.GetData() + startC;

    // build a matching lamda
    auto VerticiesMatch = [&uniqueVB, &p, &t, &c, &pu, &tu, &cu, &stride, &hasP, &hasT, &hasC](uint32_t i1, uint32_t i2)
    {
        float3 epsilon(1.e-12f);
        float3 v1 = reinterpret_cast<float*>(p + i1 * stride);
        float3 v2 = reinterpret_cast<float*>(pu + i2 * stride);

        if (hasP && DirectX::XMVector3NearEqual(v1, v2, epsilon))
        {
            bool equalForT;
            bool equalForC;
            if (hasT)
            {
                float2 t1 = reinterpret_cast<float*>(t + i1 * stride);
                float2 t2 = reinterpret_cast<float*>(tu + i2 * stride);
                if (DirectX::XMVector3NearEqual(t1, t2, epsilon))
                    equalForT = true;
                else
                    equalForT = false;
            }
            else
                equalForT = true;

            if (hasC)
            {
                float3 c1 = reinterpret_cast<float*>(c + i1 * stride);
                float3 c2 = reinterpret_cast<float*>(cu + i2 * stride);
                if (DirectX::XMVector3NearEqual(c1, c2, epsilon))
                    equalForC = true;
                else
                    equalForC = false;
            }
            else
                equalForC = true;

            if (equalForT && equalForC)
                return true;
            else
                return false;
        }
        return false;
    };


    //GetIndexBufferMaster().WriteText("before replacement.txt");

    // pre-pass
    // makes all indicies to the unique buffer the same by removing the vertex start offset of each mesh
    for (auto& mesh : _meshes)
    {
        for (uint32_t f = 0; f < mesh->GetNumIndicies(); ++f)
        {
            auto indexAddr = mesh->GetIB() + f;
            auto indexValue = *(indexAddr);

            indexValue += mesh->GetVBStart();

            assert(indexValue < _vertexBufferMaster.GetElements());
            *(indexAddr) = indexValue;
        }
        mesh->SetVBStart(0);
    }
    //GetIndexBufferMaster().WriteText("after absolute.txt");

    // first always unique and no remap as index zero is still zero
    uint32_t numUniqueVerts(0);
    uniqueVB.Copy(numUniqueVerts, _vertexBufferMaster.GetData());
    ++numUniqueVerts;

    // main comparison pass
    // loop through all verticies
    uint32_t numOfVerticies(_vertexBufferMaster.GetElements());

    for (uint32_t k = 1; k < numOfVerticies; ++k)
    {
        auto d = reinterpret_cast<float*>(&_vertexBufferMaster[k]);
        cout << "   " << (int)((float)k / numOfVerticies * (int)100) << "% " << '\r';// << "Vertex " << k << ": " << float3(d) << '\n';

        int newIndex(-1);
        // back search to identify as a duplicate against the unique buffer
        // doing a reverse search because duplicates are often close to each
        // other in our data set, and we can stop when we find one in unique
        for (int s = (int)numUniqueVerts - 1; s >= 0; --s)
        {
            if (VerticiesMatch(k, s))
            {
                //cout << "matches " << s << ',';
                newIndex = s;
                // since the s buffer is unique, we can stop our search for more
                break;
            }
        }

        if (newIndex < 0)
        {
            // new unique
            newIndex = numUniqueVerts;
            uniqueVB.Copy(newIndex, &_vertexBufferMaster[k]);
            ++numUniqueVerts;
        }

        //cout << k << " maps to " << newIndex << '\n';

        // remapped this index, k, into our unique buffer
        for (uint32_t f = 0; f < GetIndexCount(); ++f)
        {
            if (_indexBufferMaster[f] == k)
                _indexBufferMaster[f] = newIndex;
        }
    }
    //GetIndexBufferMaster().WriteText("after replacement.txt");
    if (numUniqueVerts < numOfVerticies)
    {
        MemoryBlock<uint8_t> discard;
        uniqueVB.Split(numUniqueVerts, &discard);
    }

    rtn = _vertexBufferMaster.GetElements() - uniqueVB.GetElements();
    _vertexBufferMaster = uniqueVB;

    HelperSetAllMeshesVBandIBtoMaster();

    //for (uint32_t k = 0; k < _vertexBufferMaster.GetElements(); ++k)
    //{
    //    auto d = reinterpret_cast<float*>(&_vertexBufferMaster[k]);
    //    cout << "Unique " << k << ": " << float3(d) << '\n';
    //}

    //cout << "REMOVED: " << rtn << '\n';
    return rtn;
}
/******************************************************************************
*    Method:    Scale
* // modifies the underlying data
******************************************************************************/
void King::ModelScaffold::Scale(const float3 scaleIn)
{
    if (!_vertexFormat.IsFirst(King::VertexAttrib::enumDesc::position)) return;

    auto matrix = DirectX::XMMatrixAffineTransformation(scaleIn, float3(), float3(), float3());
    Transform(matrix);
}
/******************************************************************************
*    Method:    Rotate
* // modifies the underlying data
******************************************************************************/
void King::ModelScaffold::Rotate(const float3 orignIn, const quat rIn)
{
    if (!_vertexFormat.IsFirst(King::VertexAttrib::enumDesc::position)) return;

    auto matrix = DirectX::XMMatrixAffineTransformation(float3(1.0f), orignIn, rIn, float3());
    Transform(matrix);
}
/******************************************************************************
*    Method:    Translate
* // modifies the underlying data
******************************************************************************/
void King::ModelScaffold::Translate(const float3 transIn)
{
    if (!_vertexFormat.IsFirst(King::VertexAttrib::enumDesc::position)) return;

    auto in = reinterpret_cast<XMFLOAT3*>(GetVertexBufferMaster().GetData());
    auto stride = GetVertexBufferMaster().GetStride();
    size_t numElements = GetVertexBufferMaster().GetElements();

    float3 intrinsic;
    for (size_t i = 0; i < numElements; i++)
    {
        intrinsic.Set(*in);
        *in = intrinsic + transIn;
        in = reinterpret_cast<XMFLOAT3*>((reinterpret_cast<char*>(in) + stride));
    }
    CalculateBoundingBox();
}
/******************************************************************************
*    Method:    Read_v1
*       Read and interpret data from an open file into the model definition
******************************************************************************/
bool King::ModelScaffold::Read_v1(ifstream& dataFileIn)
{
    bool rtn = true;

    if (!dataFileIn.is_open())
    {
        assert(0);
        return false;
    }
    // lambda
    auto ReadString = [&dataFileIn](string& strOut)
    {
        size_t len;
        dataFileIn.read(reinterpret_cast<char*>(&len), sizeof(len));
        char* temp = new char[len + 1];
        dataFileIn.read(temp, len);
        temp[len] = '\0';
        strOut = temp;
        delete[] temp;
    };
    // text
    ReadString(_modelName);
    // binary vertex format
    for (int i = 0; i < 8; ++i)
    {
        dataFileIn.read(reinterpret_cast<char*>(&_vertexFormat.attributes[i]._offset), sizeof(uint16_t));
        if (dataFileIn.fail())
        {
            assert(0);
            return false;
        }

        int temp;
        dataFileIn.read(reinterpret_cast<char*>(&temp), sizeof(temp));
        if (dataFileIn.fail())
        {
            assert(0);
            return false;
        }
        _vertexFormat.attributes[i]._format = (enum VertexAttrib::enumFormat)temp;

        dataFileIn.read(reinterpret_cast<char*>(&temp), sizeof(temp));
        if (dataFileIn.fail())
        {
            assert(0);
            return false;
        }
        _vertexFormat.attributes[i]._desc = (enum VertexAttrib::enumDesc)temp;;
    }
    dataFileIn.read(reinterpret_cast<char*>(&_vertexFormat.nextAttribute), sizeof(_vertexFormat.nextAttribute));
    if (dataFileIn.fail())
    {
        assert(0);
        return false;
    }
    dataFileIn.read(reinterpret_cast<char*>(&_indexFormat), sizeof(_indexFormat));
    if (dataFileIn.fail())
    {
        assert(0);
        return false;
    }
    // binary model bounding box
    dataFileIn.read(reinterpret_cast<char*>(&_boundingBox), sizeof(_boundingBox));
    if (dataFileIn.fail())
    {
        assert(0);
        return false;
    }
    // binary vetex data
    rtn = _vertexBufferMaster.Read(dataFileIn);
    if (!rtn)
    {
        assert(0);
    }
    // binary index data
    if (rtn) rtn = _indexBufferMaster.Read(dataFileIn);  
    if (!rtn)
    {
        assert(0);
    }

    if (dataFileIn.fail() || !rtn) return false;
    return true;
}
/******************************************************************************
*    Method:    Write_v1
*       Write class data to an open file
******************************************************************************/
bool King::ModelScaffold::Write_v1(ofstream& outfileIn)
{
    if (!outfileIn.is_open()) return false;
    // lambda
    auto WriteString = [&outfileIn](const string& str)
    {
        size_t len = str.size();
        outfileIn.write(reinterpret_cast<const char*>(&len), sizeof(len));
        outfileIn.write(str.c_str(), len);
    };
    WriteString(_modelName);

    // lambda
    auto WriteInt = [&outfileIn](const int& in)
    {
        outfileIn.write(reinterpret_cast<const char*>(&in), sizeof(in));
    };

    for (int i = 0; i < 8; ++i)
    {
        outfileIn.write(reinterpret_cast<char*>(&_vertexFormat.attributes[i]._offset), sizeof(uint16_t));

        int temp = _vertexFormat.attributes[i]._format;
        WriteInt(temp);
        temp = _vertexFormat.attributes[i]._desc;
        WriteInt(temp);
    }
    outfileIn.write(reinterpret_cast<char*>(&_vertexFormat.nextAttribute), sizeof(_vertexFormat.nextAttribute));

    outfileIn.write(reinterpret_cast<char*>(&_indexFormat), sizeof(_indexFormat));
    outfileIn.write(reinterpret_cast<char*>(&_boundingBox), sizeof(_boundingBox));

    _vertexBufferMaster.Write(outfileIn);
    _indexBufferMaster.Write(outfileIn);
    
    if (outfileIn.fail()) return false;
    return true;
}
/******************************************************************************
//	WorldToObject - Converter
//		Helper method for a common task in picking and collision detection.  Generally
//		to reduce the number of calculations, a point will be converted to object space
//		for collision detection rather than converting all of the meshes verticies to
//		world space.  Included here for convience and completeness.
******************************************************************************/
Position King::ModelScaffold::ConvertWorldPointToObjectSpace(const Position& worldPointIn, const DirectX::XMMATRIX worldMatrixIn)
{
    DirectX::XMMATRIX invWorldMatrix = XMMatrixInverse(nullptr, worldMatrixIn);
    Position rtn = worldPointIn * invWorldMatrix;
    return rtn;
}
Ray King::ModelScaffold::ConvertWorldRayToObjectSpace(const Ray& worldRayIn, const DirectX::XMMATRIX worldMatrixIn)
{
    DirectX::XMMATRIX invWorldMatrix = XMMatrixInverse(nullptr, worldMatrixIn);
    Ray rtn = worldRayIn * invWorldMatrix;
    return rtn;
}
/******************************************************************************
*    Method:    Transform
******************************************************************************/
void King::ModelScaffold::Transform(const DirectX::XMMATRIX transformIn)
{
    if (!_vertexFormat.IsFirst(King::VertexAttrib::enumDesc::position)) return;

    auto in = reinterpret_cast<XMFLOAT3*>(GetVertexBufferMaster().GetData());
    auto out = in;
    auto stride = GetVertexBufferMaster().GetStride();
    size_t numElements = GetVertexBufferMaster().GetElements();

    DirectX::XMVector3TransformCoordStream(out, stride, in, stride, numElements, transformIn); // stride should be in bytes
    CalculateBoundingBox();
}
/******************************************************************************
*    Method:    GetVertexAddr
*       return the point to start of the vextex data
*       by vertex number or by indirection through the index buffer
******************************************************************************/
uint8_t* King::ModelScaffold::GetVertexAddr(const uint32_t vertexNumIn, uint32_t vbStartIn) 
{
    // works becuase _vertexBufferMaster._stride is set to _vertexFormat.GetByteSize()
    uint8_t* vb = &_vertexBufferMaster.Get(vertexNumIn + vbStartIn);
    //uint8_t* vb = &_vertexBufferMaster.Get(vertexNumIn) + vbStartIn; // changed on 6/26/2022 CHK such that vbStartIn is in uint8_t, not element number
    return vb;
}
uint8_t* King::ModelScaffold::GetVertexAddr(const uint32_t indexIn, uint32_t vbStartIn, uint32_t ibStartIn)
{ 
    assert(indexIn < _indexBufferMaster.GetElements());

    auto bs = _vertexFormat.GetByteSize(); 
    uint8_t* vb = _vertexBufferMaster.GetData() + (size_t)vbStartIn * bs;
    //uint8_t* vb = _vertexBufferMaster.GetData() + (size_t)vbStartIn; // changed on 6/26/2022 CHK such that vbStartIn is in uint8_t, not element number (this needs to be changed in many places as well!!)
    uint32_t* ib = _indexBufferMaster.GetData() + ibStartIn;
    uint32_t vertNum = *(ib + indexIn);
    //assert(vertNum < _vertexBufferMaster.GetElements() - vbStartIn);

    return vb + vertNum * (size_t)bs;
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
    if (mtl_IN == nullptr) return;

    const auto & mtlName = mtl_IN->Get_name();

    _materials[mtlName] = mtl_IN;

    return;
}
/******************************************************************************
*    Method:    CreateMesh
******************************************************************************/
inline std::shared_ptr<TriangleMesh> King::Model::CreateMesh(const uint32_t numTrianglesIn, const uint32_t vbStartIn, const uint32_t ibStartIn)
{
    assert((bool)GetVertexBufferMaster);
    assert((bool)GetIndexBufferMaster);
    auto ptr = TriangleMesh::Create(numTrianglesIn, GetVertexFormat(), vbStartIn, ibStartIn, GetVertexBufferMaster().GetData(), GetIndexBufferMaster().GetData());
    _meshes.push_back(ptr);
    return _meshes.back();
}
/******************************************************************************
*    Method:    CreateMeshFrom
*       Inputs:
*           Line            List of two points for each new vertex
*           Distance        length and direction to extrude Path to form new
*                           verticies and join as a mesh (forms a quad)
*       Output:
*           Adds a new TriangleMesh to _meshes. At minimum, it will create
*           master data in our model for positions.  If you predefine the
*           vertex format, it will add informational in as well
*       Returns:
*           Index into _meshes of the TriangleMesh created
******************************************************************************/
int King::Model::CreateMeshFrom(const Line& l, Distance d)
{
    if (!_materials.count("default"))
        AddMaterial(Material::Create("default"));
    // used existing format
    auto stride = _vertexFormat.GetByteSize();
    // is a model vertex buffer already setup?
    if (stride != GetVertexBufferMaster().GetStride())
    {
        if (_vertexFormat.GetByteSize() < 56)
        {
            if (!_vertexFormat.Has(VertexAttrib::enumDesc::position))
                _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);

            _vertexFormat.SetNext(VertexAttrib::enumDesc::textureCoord, VertexAttrib::enumFormat::format_float32x2);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::normal, VertexAttrib::enumFormat::format_float32x3);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::tangent, VertexAttrib::enumFormat::format_float32x3);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::bitangent, VertexAttrib::enumFormat::format_float32x3);
            _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
        }
    }
    auto indexPos = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position); // UINT16_MAX did not find
    auto indexUV = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::textureCoord); // UINT16_MAX did not find
    auto indexNorm = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::normal); // UINT16_MAX did not find

    if (indexPos == UINT16_MAX)
    {
        // minimum is not present, reset the master and add position as minimum
        _vertexBufferMaster.Destroy();
        _indexBufferMaster.Destroy();
        _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
        stride = _vertexFormat.GetByteSize();
        _vertexBufferMaster.SetStride(stride);
    }

    // define size of elements
    const size_t numVertex = 4;
    const size_t numTriangles = 2;

    // start where the last mesh left off
    const uint32_t vbStart(_vertexBufferMaster.GetElements()); // zero based to this start position for mesh
    const uint32_t ibStart(_indexBufferMaster.GetElements()); // zero based to this start position for mesh
    // create our mesh and allocate memory
    {
        // total vertex and indicies for our mesh matching our model's master format
        MemoryBlock<uint8_t> vb(numVertex, GetVertexStride());
        MemoryBlock<uint32_t> ib(3 * numTriangles);
        // store our data into the model master buffers
        AddMasterVertexData(vb);
        AddMasterIndexData(ib);
        // our mesh!
        auto tm = CreateMesh(numTriangles, vbStart, ibStart);
        tm->Set_name("CreateMeshFrom(line,distance)");
        tm->Set_materialName("default");
    }

    // indicies, RH CCW
    {
        MemoryBlock<uint32_t> ib; // (3 * numTriangles);
        ib = { 0, 1, 2, 2, 1, 3 };
        assert((_indexBufferMaster.GetElements() - ibStart) >= ib.GetElements());
        _indexBufferMaster.Copy(ibStart, ib.GetData(), ib.GetElements());
    }

    Quad q = l * d;

    // positions
    assert(indexPos != UINT16_MAX);
    const auto offsetPosBytes = _vertexFormat.GetAttributeByteStart(indexPos);
    for (int i = 0; i < 4; ++i)
    {
        auto src = static_cast<uint8_t*>(q.GetVertex(i));
        auto dest = GetVertexAddr(i, vbStart) + offsetPosBytes;
        std::copy(src, src + 12, dest);
        // this is built in, but slower in a loop
        //SetVertexElement(i + vbStart, VertexAttrib::enumDesc::position, src);
    }
    // texture coordinates
    if (indexUV)
    {
        auto offsetUVBytes = _vertexFormat.GetAttributeByteStart(indexUV);
        float2 uv(0.f, 0.f);

        for (int i = 0; i < 4; ++i)
        {
            auto src = static_cast<uint8_t*>(uv);
            auto dest = GetVertexAddr(i, vbStart) + offsetUVBytes;
            std::copy(src, src + 8, dest);
        }
    }
    // normals
    if (indexNorm != UINT16_MAX)
    {
        vector<float3> pos;
        for (auto i = 0; i < 4; ++i)
        {
            pos.emplace_back(float3(static_cast<uint8_t*>(q.GetVertex(i)) + offsetPosBytes));
        }
        float3 norm = Cross((pos[1] - pos[0]), (pos[2] - pos[0]));
        norm.MakeNormalize(); // unit vector

        const auto offsetNormBytes = _vertexFormat.GetAttributeByteStart(indexNorm);
        for (int i = 0; i < 4; ++i)
        {
            auto src = static_cast<uint8_t*>(norm);
            auto dest = GetVertexAddr(i, vbStart) + offsetNormBytes;
            std::copy(src, src + 12, dest);
        }
    }
    // tangents and bitangents
    CalculateTangentsAndBiTangents();

    // update the bounding geometry
    CalculateBoundingBox();
    // OptimizeMeshVertexBuffer(*tm); // commented out only because it is not tested (modified to accomodate non-zero based indicies and vertex)
    // return the mesh index
    return (int)_meshes.size() - 1;
}
/******************************************************************************
*    Method:    CreateMeshFrom
*       Inputs:
*           Path p          List of points for each new vertex that encloses an
*                               area in CCW rotation.
*       Output:
*           Adds a new TriangleMesh to _meshes which forms a closed mesh of the
*           path.
*       Returns:
*           Index into _meshes of the TriangleMesh created
******************************************************************************/
int King::Model::CreateMeshFrom(const Path& p)
{
    assert(p.size() > 2);
    auto numPathVerts = p.size();

    vector<float3> verticies;
    vector<uint32_t> indicies;
    vector<float2> uvs;
    vector<float3> normals;

    Triangle t;
    // first two of triangle strip;
    // make sure first and last are not the same
    if (p[0] == p[numPathVerts - 1])
        --numPathVerts;
    
    assert(numPathVerts > 3); // min
    /* 
     path ordering:
     15  14  12  11  10  8   7

     vertex ordering desired:
     0---2---5---6---9---10--13
     |  /|\  |  /|\  |  /|\  |
     | / | \ | / | \ | / | \ |
     |/  |  \|/  |  \|/  |  \|
     1---3---4---7---8---11--12

     0   1   2   3   4   5   6
             ^
             | loop starts
*/
    // Re-order the path into verticies 
    /*  (verified) 5/15/2022 CHK
        Input of 5, CCW, bottom first:
        { path 0: { x:         0 y:         0 z:         0 w:         1 }
        { path 1: { x:         1 y:         0 z:         0 w:         1 }
        { path 2: { x:         2 y:         0 z:         0 w:         1 }
        { path 3: { x:         1 y:         1 z:         0 w:         1 }
        { path 4: { x:         0 y:         1 z:         0 w:         1 }
    */
    /*
        Input of 6, CCW, bottom first:
        path 0: { x:         0 y:         0 z:         0 w:         1 }
        path 1: { x:         1 y:         0 z:         0 w:         1 }
        path 2: { x:         2 y:         0 z:         0 w:         1 }
        path 3: { x:         2 y:         1 z:         0 w:         1 }
        path 4: { x:         1 y:         1 z:         0 w:         1 }
        path 5: { x:         0 y:         1 z:         0 w:         1 }
        
    */
    // if p[0] is too far left of both points, the tri is inverted, we would need to reverse the winding order
    {
        if (p[numPathVerts - 1].GetY() > p[0].GetY())
        {
            verticies.push_back(p[numPathVerts - 1]);
            verticies.push_back(p[0]);
            verticies.push_back(p[numPathVerts - 2]);
            /* // this is an error, I think the only way to detect for sure is to valid and fix before passed to this method
            { x:  -858.622 y : 754.389 z : 0 }
            { x:  -1000.18 y : -963.679 z : 0 } // p[0]
            { x:  -590.304 y : 867.352 z : 0 }
            */
        }
        else
        {
            // attempt at fixing the input (but the case above still unsolved)
            // Error in y reverses:
            //{ x:  -840.297 y : -200.086 z : 0 }
            //{ x:  -1002.52 y : 206.8 z : 0 }
            //{ x:  -661.723 y : 1007.02 z : 0 }
            // p[last].y < p[first].y !!!
            if (p[numPathVerts - 2].GetY() < p[numPathVerts - 1].GetY())
            {
                // p[numPathVerts - 2] has the smallest y
                verticies.push_back(p[0]);
                verticies.push_back(p[numPathVerts - 2]);
                verticies.push_back(p[numPathVerts - 1]);
            }
            else
            {
                // p[numPathVerts - 1] has the smallest y
                verticies.push_back(p[0]);
                verticies.push_back(p[numPathVerts - 1]);
                verticies.push_back(p[numPathVerts - 2]);
            }

        }
    }
    /*
        Output of 5, CCW, top first then alternating top and bottom:
        v0 { path 4: { x:         0 y:         1 z:         0 w:         1 } } // t
        v1 { path 0: { x:         0 y:         0 z:         0 w:         1 } } // b
        v2 { path 3: { x:         1 y:         1 z:         0 w:         1 } } // t
        v3 { path 1: { x:         1 y:         0 z:         0 w:         1 } } // b
        
        v4 { path 2: { x:         2 y:         0 z:         0 w:         1 } } // b
    */
    /*
        Output of 6, CCW, top first then alternating top and bottom:
        v0 { path 5: { x:         0 y:         1 z:         0 w:         1 } } // t
        v1 { path 0: { x:         0 y:         0 z:         0 w:         1 } } // b
        v2 { path 4: { x:         1 y:         1 z:         0 w:         1 } } // t
        v3 { path 1: { x:         1 y:         0 z:         0 w:         1 } } // b
        
        v4 { path 2: { x:         2 y:         0 z:         0 w:         1 } } // b
        v5 { path 3: { x:         2 y:         1 z:         0 w:         1 } } // t
    */

    // Vertex buffer creation
    bool bottom(true);
    bool reverse(false);
    size_t i, end, bottomIndex(0), topIndex(numPathVerts - 2);
    end = (numPathVerts - 3);
    for (i = 0; i < end; ++i)
    {
        if (i == numPathVerts - 2)
            __debugbreak();
        // pattern is b b t t b b t t b b t t ....
        if (bottom) verticies.push_back(p[++bottomIndex]);
        else verticies.push_back(p[--topIndex]);

        if (reverse) bottom = !bottom;
        reverse = !reverse;
    }
    cout << "Verticies:\n";
    for (const auto& i : verticies)
        cout << i << '\n';
    
    assert(numPathVerts == verticies.size());

    // Index buffer creation
    indicies.push_back(0); // tri1
    indicies.push_back(1);
    indicies.push_back(2);

    indicies.push_back(2); // tri2
    indicies.push_back(1);
    indicies.push_back(3);

    float3 normalPrev; // used in validation below that we are facing in the same general direction (by use of dot product)
    {
        Triangle t1(verticies[indicies[0]], verticies[indicies[1]], verticies[indicies[2]]);
        Triangle t2(verticies[indicies[2]], verticies[indicies[1]], verticies[indicies[3]]);
        normalPrev = t2.GetNormalCCW();
        assert(float3::DotProduct(t1.GetNormalCCW(), normalPrev) >= 0.f);
    }
    bottom = false;
    reverse = false;
    for (i = 4; i < numPathVerts; ++i)
    {
        /*  
             vertex ordering:
                +2  +3  +1  +3  +1   +3
             0---2---5---6---9---10--13
             | 1/|\4 |5 /|\8 |9 /|\  |
             | / | \ | / | \ | / | \ |
             |/ 2|3 \|/6 |7 \|/  |  \|
             1---3---4---7---8---11--12
                +2  +1  +3  +1  +3   +1
        */
        /* 6 points is 4 tris
            0,1,2, 2,1,3 ,2,3,4, 2,4,5, (verified) 5/15/2022 CHK
        */
        auto indexCount = indicies.size();

        bottom = !bottom;

        if (bottom)
        {
            // i = 6, 10
            if (reverse)
            {
                indicies.push_back(i - 1);
                indicies.push_back(i - 2);
                indicies.push_back(i);
            }
            else // i = 4, 8
            {
                indicies.push_back(i - 2);
                indicies.push_back(i - 1);
                indicies.push_back(i);
            }
        }
        else
        {
            // i = 7, 11
            if (reverse)
            {
                indicies.push_back(i - 1);
                indicies.push_back(i - 3);
                indicies.push_back(i);

            }
            else // i = 5, 9
            {
                indicies.push_back(i - 3);
                indicies.push_back(i - 1);
                indicies.push_back(i);
            }
            reverse = !reverse;
        }
        // validate the triangle
        t.Set(verticies[indicies[indexCount + 0]], verticies[indicies[indexCount + 1]], verticies[indicies[indexCount + 2]]);
        auto nCCW = t.GetNormalCCW();
        if (float3::DotProduct(nCCW, normalPrev) >= 0.f)
        {
            normalPrev = nCCW;
        }
        else
        {
            // reverse
            auto temp = indicies[indexCount + 1];
            indicies[indexCount + 1] = indicies[indexCount + 2];
            indicies[indexCount + 2] = temp;

            t.Set(verticies[indicies[indexCount + 0]], verticies[indicies[indexCount + 1]], verticies[indicies[indexCount + 2]]);
            nCCW = t.GetNormalCCW();
            if(float3::DotProduct(nCCW, normalPrev) >= 0.f)
                normalPrev = nCCW;
            else
            {
                // something is wrong, the triangle needs some help, just get rid of it for now
                indicies.pop_back();
                indicies.pop_back();
                indicies.pop_back();
            }
        }
    }

    for (const auto& i : indicies)
        cout << i << ',';
    cout << '\n';

    return HelperCreateMeshFrom(verticies, indicies, uvs, normals);
}
/******************************************************************************
*    Method:    CreateMeshFrom
*       Inputs:
*           Path p          List of points for each new vertex
*           Distance d      length and direction to extrude Path to form new
*                           verticies and join as a mesh
*       Output:
*           Adds a new TriangleMesh to _meshes
* *       Returns:
*           Index into _meshes of the TriangleMesh created
******************************************************************************/
int King::Model::CreateMeshFrom(const Path& p, Distance d)
{
    assert(p.size() > 2);

    // Form a series of quad meshes joined by one common edge between pairs.
    const auto& numPathVerts = p.size();
    vector<Quad> quads;
    vector<float3> normals;

    // prime the loop by setting up the first quad
    Line l(p[0], p[1]);
    {
        Quad q = l * d;
        normals.emplace_back(q.GetNormalCCW());
    }
    float3 firstPlane = float3(1.0f) - normals.back();
    float3 l1n = l.GetPerpendicular(firstPlane);
    quat prev_rot; // unity default

    for (size_t i = 1; i < numPathVerts; ++i)
    {
        // line formed
        l.Set(p[i - 1], p[i]);
        //*** SPECIAL CASE  ***
        // change in height not part of rotation
        auto y = l.pt[1].GetY() - l.pt[0].GetY();
        l.pt[1].SetY(l.pt[0].GetY());
        // *** END ***
        // perpendicular within the specified plane
        float3 l2n = l.GetPerpendicular(firstPlane); // my line
        // our rotation with the firstPlaneIn (from/to)
        quat rot(l1n, l2n); // from/to special cases might be an error in rotations... need to test more *** TO DO ***
        // save for next loop iteration
        l1n = l2n;

        d = rot * d; // first time through loop, no rotation applied

        // *** SPECIAL CASE RESTORE ***
        l.pt[1].SetY(l.pt[1].GetY() + y);

        // our extruded quad
        Quad q = l * d;
        quads.push_back(q);
        // and its normal
        normals.emplace_back(q.GetNormalCCW());
    }

    if (!_materials.count("default"))
        AddMaterial(Material::Create("default"));
    auto stride = _vertexFormat.GetByteSize();
    // is a model vertex buffer already setup?
    if (stride != GetVertexBufferMaster().GetStride())
    {
        if (_vertexFormat.GetByteSize() < 56)
        {
            if (!_vertexFormat.Has(VertexAttrib::enumDesc::position))
                _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);

            _vertexFormat.SetNext(VertexAttrib::enumDesc::textureCoord, VertexAttrib::enumFormat::format_float32x2);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::normal, VertexAttrib::enumFormat::format_float32x3);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::tangent, VertexAttrib::enumFormat::format_float32x3);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::bitangent, VertexAttrib::enumFormat::format_float32x3);
            _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
        }
    }
    auto indexPos = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position); // UINT16_MAX did not find
    auto indexUV = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::textureCoord); // UINT16_MAX did not find
    auto indexNorm = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::normal); // UINT16_MAX did not find

    if (indexPos == UINT16_MAX)
    {
        // minimum is not present, reset the master and add position as minimum
        _vertexBufferMaster.Destroy();
        _indexBufferMaster.Destroy();
        _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
        stride = _vertexFormat.GetByteSize();
        _vertexBufferMaster.SetStride(stride);
    }

    // define size of elements
    const size_t numVertex = quads.size() * 4; // *** TO DO *** optimize to 2 shared per pair of quads (except gaps) by removing verticies *** TO DO ***
    size_t numTriangles = quads.size() * 2; 

    // do we need to add triangles to fill gaps?
    for (size_t i = 1; i < quads.size(); ++i)
    {
        auto e4 = quads[i].GetEdge4();
        auto e2 = quads[i - 1].GetEdge2();

        e4.MakeNormalize();
        e2.MakeNormalize();
        float dot = Dot(e4, e2);
        //dot = (dot < -1.0 ? -1.0 : (dot > 1.0 ? 1.0 : dot));
        //float angle = acos(dot);

        //cout << "Dot(e4, e2) = " << to_string(dot) << '\n';
        if (dot > .1f)
        {
            // *** TO DO *** we need to now test if it is opening the faces or closing the faces (overlap).  If opening add a triangle to close it. *** TO DO ***
            //++numTriangles;
        }
    }

    // start where the last mesh left off
    const uint32_t vbStart(_vertexBufferMaster.GetElements()); // zero based to this start position for mesh
    const uint32_t ibStart(_indexBufferMaster.GetElements()); // zero based to this start position for mesh
    // create our mesh and allocate memory
    {
        // total vertex and indicies for our mesh matching our model's master format
        MemoryBlock<uint8_t> vb(numVertex, GetVertexStride());
        MemoryBlock<uint32_t> ib(3 * numTriangles);
        // store our data into the model master buffers
        AddMasterVertexData(vb);
        AddMasterIndexData(ib);
        // our mesh!
        auto tm = CreateMesh(numTriangles, vbStart, ibStart);
        tm->Set_name("CreateMeshFrom(path,distance)");
        tm->Set_materialName("default");
    }

    // indicies, RH CCW
    MemoryBlock<uint32_t> ib(6);
    for (size_t i = 0; i < quads.size(); ++i)
    {
        auto c = i * 4;
        ib[0] = c + 0ull; ib[1] = c + 1ull; ib[2] = c + 2ull;
        ib[3] = c + 2ull; ib[4] = c + 1ull; ib[5] = c + 3ull;

        _indexBufferMaster.Copy(ibStart + i * 6, ib.GetData(), ib.GetElements());
    }
    //cout << "_indexBufferMaster { length:" << to_string(_indexBufferMaster.GetLength()) << " stride:" << to_string(_indexBufferMaster.GetStride()) << " data: ";
    //for (size_t i = 0; i < _indexBufferMaster.GetLength(); ++i)
    //{
    //    cout << to_string(_indexBufferMaster.Get(i)) << " ";
    //}
    //cout << " }\n";
    // verticies
    const auto offsetPosBytes = _vertexFormat.GetAttributeByteStart(indexPos);
    const auto offsetUVBytes = _vertexFormat.GetAttributeByteStart(indexUV);
    const auto offsetNormBytes = _vertexFormat.GetAttributeByteStart(indexNorm);

    uint32_t count(0);
    const size_t increment = 4;
    for (const auto& q : quads)
    {
        // positions
        assert(indexPos != UINT16_MAX);
        for (uint32_t i = 0; i < 4; ++i)
        {
            auto src = static_cast<uint8_t*>(q.GetVertex(i));
            auto dest = GetVertexAddr(count + i, vbStart) + offsetPosBytes;
            std::copy(src, src + 12, dest);
            // this is built in, but slower in a loop
            //SetVertexElement(i + vbStart, VertexAttrib::enumDesc::position, src);
        }
        // texture coordinates
        if (indexUV)
        {
            float2 uv(0.f, 0.f);

            for (int i = 0; i < 4; ++i)
            {
                auto src = static_cast<uint8_t*>(uv);
                auto dest = GetVertexAddr(count + i, vbStart) + offsetUVBytes;
                std::copy(src, src + 8, dest);
            }
        }
        // normals
        if (indexNorm != UINT16_MAX)
        {
            vector<float3> pos;
            for (auto i = 0; i < 4; ++i)
            {
                pos.emplace_back(float3(static_cast<uint8_t*>(q.GetVertex(i)) + offsetPosBytes));
            }

            auto& norm = normals[count / increment];

            for (int i = 0; i < 4; ++i)
            {
                auto src = static_cast<uint8_t*>(norm);
                auto dest = GetVertexAddr(count + i, vbStart) + offsetNormBytes;
                std::copy(src, src + 12, dest);
            }
        }
        count += increment;
    }
    // normals where checked for general direction match as adjacent and then averaged
    // so let make the winding order match
    ReverseWindingsToMatchNormals();

    // tangents and bitangents
    CalculateTangentsAndBiTangents();

    // update the bounding geometry
    CalculateBoundingBox();
    // OptimizeMeshVertexBuffer(*tm); // commented out only because it is not tested (modified to accomodate non-zero based indicies and vertex)
    // return the mesh index
    return (int)_meshes.size() - 1;
}

int King::Model::CreateMeshFromExp(const Path& path, Distance d)
{
    vector<float3> positions;
    vector<uint32_t> indicies;
    // these are optional, HelperCreateMeshFrom will create them
    vector<float2> uv;
    vector<float3> normals;

    // distance is outward from consecutive points, CCW
    auto pt0 = path[0];
    positions.push_back(pt0);
    normals.push_back(float3());
    positions.push_back(pt0 + d);
    normals.push_back(float3());

    Distance prevEdge2 = path[1] - pt0;
    quat rot;
    bool tri(false);
    bool quad(false);
    for (size_t i = 1; i < path.size(); ++i) // *** 5/22/2022 just the last point added is messed up (i_n + d)
    {
        auto& pti = path[i];
        Distance edge2 = pti - pt0;

        if (edge2.Get_unit_direction() != prevEdge2.Get_unit_direction())
        {
            // find the rotation from prevEdge2 to edge2
            rot = quat(prevEdge2.Get_unit_direction(), edge2.Get_unit_direction());
            d = rot * d;
            // if the angle is > 270 deg, insert another point and form a quad [-pi, +pi] CW
            if (rot.GetAngleEuler() > UnitOfMeasure::PI_DIV2)
            {
                /*    pt0
                      /|\
                     / | \
                pos0/  |  \pt0 + d
                      edge    
                */
                auto pos0 = positions.back();
                auto edge = (pos0 + pt0 + d);
                edge.MakeNormalize();
                edge *= d.Get_magnitude();
                // edge
                positions.push_back(edge);
                auto n = float3::CrossProduct(pos0 - pt0, edge);
                n.MakeNormalize();
                normals.push_back(n);
                size_t pos = positions.size() - 1;
                indicies.push_back(pos - 2);
                indicies.push_back(pos - 1);
                indicies.push_back(pos);
                // pt0 + d
                positions.push_back(pt0 + d);
                n = float3::CrossProduct(edge, d);
                n.MakeNormalize();
                normals.push_back(n);
                pos = positions.size() - 1;
                indicies.push_back(pos - 3);
                indicies.push_back(pos - 1);
                indicies.push_back(pos);

                quad = true;
            }
            else 
            {
                // close the gap with a triangle
                auto pos0 = positions.back();
                positions.push_back(pt0 + d);
                auto n = float3::CrossProduct(pos0, d);
                n.MakeNormalize();
                normals.push_back(n);

                // close the gap creating an extra tri
            /*
                        Path:       0   1 = i
                          pti = pos-2
                                    |\
                                    | \
                 pti + edge1 = pos-1|  \pos = pt0 + d
            */
                size_t pos = positions.size() - 1;
                indicies.push_back(pos - 2);
                indicies.push_back(pos - 1);
                indicies.push_back(pos);

                tri = true;
            }
        }

        // our quad
        {
            // direction is inline, add two more points
        /*
                    Path:       0   1 = i
                      pt0 = pos0-----pos2 = pti
                                |  /|
                                | / |
              pt0 + edge1 = pos1|/  |pos3 = pti + edge1

        */
            positions.push_back(pti);
            auto n = float3::CrossProduct(d, edge2);
            n.MakeNormalize();
            normals.push_back(n);

            size_t pos = positions.size() - 1;
            
            if (quad)
            {
                quad = false;
                indicies.push_back(pos - 4);
            }
            else if (tri)
            {
                tri = false;
                indicies.push_back(pos - 3);
            }
            else
                indicies.push_back(pos - 2);

            indicies.push_back(pos - 1);
            indicies.push_back(pos);

            positions.push_back(pti + d);
            normals.push_back(n);

            pos = positions.size() - 1;
            indicies.push_back(pos - 1);
            indicies.push_back(pos - 2);
            indicies.push_back(pos);
        }
        // advance the loop base edge
        prevEdge2 = edge2;
        pt0 = pti;
    }

    // close it all up by adding one last triangle at the end
    size_t pos = positions.size() - 1;
    indicies.push_back(0);
    indicies.push_back(pos);
    indicies.push_back(1);

    auto meshIndex = HelperCreateMeshFrom(positions, indicies, uv, normals);

    return meshIndex;
}
/******************************************************************************
*    Method:    CreateMeshFrom
*       Inputs:
*           Path pFrom      List of points for each new vertex
*           Path pTo        List of points for each new vertex
*                           Verticies are joined with triangles to form a mesh
*       Output:
*           Adds a new TriangleMesh to _meshes
* *       Returns:
*           Index into _meshes of the TriangleMesh created
******************************************************************************/
int King::Model::CreateMeshFrom(const Path& pFrom, const Path& pTo)
{
    assert(pFrom.size() > 1 && pTo.size() > 1);

    auto numPathVerts = min(pFrom.size(), pTo.size());
    vector<float3> verts;
    vector<float3> normals;
    vector<bool> quadOrTri; // yes = quad, no = triangle

    // arrange verts in near optimized vert buffer with CCW convention
    // 1   3   5   7 (odd row)(to)
    // -------------
    // |\  |\  |\  |
    // | \ | \ | \ |
    // |  \|  \|  \|
    // -------------
    // 0   2   4   6 (even row)(from)
    // quad1 : (0, 2, 1, 1, 2, 3) ; near optimized because quad1 is not (need to swap 2 & 1)
    // quad2 : (2, 4, 3, 3, 4, 5) ; if we fully optimize, 2 & 1 swap will edit indexing in quad2
    // quad3 : (4, 6, 5, 5, 6, 7) ; starting with quad 3, and everyone after, the vertex order is in order of access
    // since quad1 and quad2 verticies are very close to each other on GPU cache, there is no reason to fully optimize here post loop
    
    verts.push_back(pFrom[0]);
    verts.push_back(pTo[0]);
    for (size_t i = 1; i < numPathVerts; ++i)
    {
        verts.push_back(pFrom[i]);
        
        // if from == to, then we have a triangle and we only need one of them in the buffer
        if (DirectX::XMVector3NearEqual(pFrom[i].GetVecConst(), pTo[i].GetVecConst(), float3(FLT_EPSILON)))
            quadOrTri.push_back(false);
        else
        {
            verts.push_back(pTo[i]);
            quadOrTri.push_back(true);
        }
        
    }
    // normals
    size_t face = 0;
    for (size_t i = 0; i < verts.size() - 2; i += 2)
    {
        if (quadOrTri[face])
        {
            Quad q(verts[i], verts[i + 2], verts[i + 1], verts[i + 3]); // (0,2,1,3)
            normals.emplace_back(q.GetNormalCCW());
        }
        else
        {
            Triangle t(verts[i], verts[i + 2], verts[i + 1]); // (0,2,1)
            normals.emplace_back(t.GetNormalCCW());
            // we did not use the i+1 vertex, it was not pushed to the verts buffer
            // and as it does not exist, we need to reduce our counter
            --i;
        }
        ++face;
    }
    assert(normals.size() == quadOrTri.size());

    if (!_materials.count("default"))
        AddMaterial(Material::Create("default"));
    auto stride = _vertexFormat.GetByteSize();
    // is a model vertex buffer already setup?
    if (stride != GetVertexBufferMaster().GetStride())
    {
        if (_vertexFormat.GetByteSize() < 56)
        {
            if (!_vertexFormat.Has(VertexAttrib::enumDesc::position))
                _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);

            _vertexFormat.SetNext(VertexAttrib::enumDesc::textureCoord, VertexAttrib::enumFormat::format_float32x2);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::normal, VertexAttrib::enumFormat::format_float32x3);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::tangent, VertexAttrib::enumFormat::format_float32x3);
            _vertexFormat.SetNext(VertexAttrib::enumDesc::bitangent, VertexAttrib::enumFormat::format_float32x3);
            _vertexBufferMaster.SetStride(_vertexFormat.GetByteSize());
        }
    }
    auto indexPos = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position); // UINT16_MAX did not find
    auto indexUV = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::textureCoord); // UINT16_MAX did not find
    auto indexNorm = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::normal); // UINT16_MAX did not find

    if (indexPos == UINT16_MAX)
    {
        // minimum is not present, reset the master and add position as minimum
        _vertexBufferMaster.Destroy();
        _indexBufferMaster.Destroy();
        _vertexFormat.SetNext(VertexAttrib::enumDesc::position, VertexAttrib::enumFormat::format_float32x3);
        stride = _vertexFormat.GetByteSize();
        _vertexBufferMaster.SetStride(stride);
    }

    // define size of elements
    const size_t numVertex = verts.size();
    size_t numTriangles = 0;
    for (const auto& ea : quadOrTri)
    {
        if (ea)
            numTriangles += 2;
        else
            ++numTriangles;
    }
    
    // start where the last mesh left off
    const uint32_t vbStart((uint32_t)_vertexBufferMaster.GetElements()); // zero based to this start position for mesh
    const uint32_t ibStart((uint32_t)_indexBufferMaster.GetElements()); // zero based to this start position for mesh
    // create our mesh and allocate memory
    {
        // total vertex and indicies for our mesh matching our model's master format
        MemoryBlock<uint8_t> vb(numVertex, GetVertexStride());
        MemoryBlock<uint32_t> ib(3 * numTriangles);

        // store our data into the model master buffers
        AddMasterVertexData(vb);
        AddMasterIndexData(ib);
        // our mesh!
        auto tm = CreateMesh(numTriangles, vbStart, ibStart);
        tm->Set_name("CreateMeshFrom(path,path)");
        tm->Set_materialName("default");
    }

    // indicies, RH CCW, to create a quad of 6 or triangle of 3 indicies
    {
        MemoryBlock<uint32_t> ib(6);
        size_t c = 0;
        size_t indicies = 0;
        // for each face
        for (size_t i = 0; i < quadOrTri.size(); ++i)
        {
            if (quadOrTri[i])
            {
                // four verticies, six indicies
                ib[0] = c; ib[1] = c + 2; ib[2] = c + 1;
                ib[3] = c + 1; ib[4] = c + 2; ib[5] = c + 3;
                _indexBufferMaster.Copy(ibStart + indicies, ib.GetData(), 6);
                indicies += 6;
                // quad strip, we reused our first two and two more new
                c += 2;
            }
            else // triangle
            {
                // three verticies, three indicies
                ib[0] = c; ib[1] = c + 2; ib[2] = c + 1;
                _indexBufferMaster.Copy(ibStart + indicies, ib.GetData(), 3);
                indicies += 3;
                // triangle strip, we reused our first two and one more new
                c += 1;
            }
        }
    }
    // verticies
    const auto offsetPosBytes = _vertexFormat.GetAttributeByteStart(indexPos);
    const auto offsetUVBytes = _vertexFormat.GetAttributeByteStart(indexUV);
    const auto offsetNormBytes = _vertexFormat.GetAttributeByteStart(indexNorm);

    for (uint32_t i = 0; i < verts.size(); ++i)
    {
        // positions
        assert(indexPos != UINT16_MAX);
        {
            auto src = static_cast<uint8_t*>(verts[i]);
            auto dest = GetVertexAddr(i, vbStart) + offsetPosBytes;
            std::copy(src, src + 12, dest);
        }

        // texture coordinates
        if (indexUV)
        {
            float2 uv(0.f, 0.f);
            auto src = static_cast<uint8_t*>(uv);
            auto dest = GetVertexAddr(i, vbStart) + offsetUVBytes;
            std::copy(src, src + 8, dest);
        }
    }
    // check
    //auto vert0 = float3(GetVertexAddr(0, vbStart) + offsetPosBytes);
    //assert(vert0.GetX() > -FLT_MIN);

    // normals
    if (indexNorm != UINT16_MAX)
    {
        uint32_t c = 2; // vertex count
        // quadOrTri size is number of faces
        const auto& iSize = quadOrTri.size();
        for (uint32_t i = 0; i < iSize; ++i)
        {
            auto nin = i + 1;
            if (nin >= iSize)
                nin = i;
            // make sure normals are in the same general direction as the one before
            float check = normals[i].DotProduct(normals[nin]);
            if (check < 0.f)
                normals[nin] = -normals[nin];
            // average the two faces so they are smooth
            float3 norm = (normals[i] + normals[nin]) * 0.5f;
            auto src = static_cast<uint8_t*>(norm);

            if (quadOrTri[i]) // quad
            {
                // verticies copy normal
                for (int j = -2; j < 2; ++j)
                {
                    auto dest = GetVertexAddr(j + c, vbStart) + offsetNormBytes;
                    std::copy(src, src + 12, dest);
                }
                // quad strip, we reused our first two and then two new
                c += 2;
            }
            else // triangle
            {
                for (int j = -2; j < 1; ++j)
                {
                    auto dest = GetVertexAddr(j + c, vbStart) + offsetNormBytes;
                    std::copy(src, src + 12, dest);
                }
                // triangle strip, we reused our first two and then one new
                c += 1;
            }
        }
    }
    //auto a = _indexBufferMaster.Max();
    //_indexBufferMaster.WriteText("CreateMeshFromPathPath.txt");
    //auto b = _vertexBufferMaster.GetElements();
    //assert(a < b);
    
    // normals where checked for general direction match as adjacent and then averaged
    // so let make the winding order match
    ReverseWindingsToMatchNormals();

    CalculateTangentsAndBiTangents();
    CalculateBoundingBox();

    // return the mesh index
    return (int)_meshes.size() - 1;
}
/******************************************************************************
*    Method:    SetIndex
******************************************************************************/
void King::ModelScaffold::SetIndex(const size_t & indexIn, const uint32_t& value)
{
    assert((bool)_indexBufferMaster);
    assert(indexIn < _indexBufferMaster.GetElements());
    _indexBufferMaster[indexIn] = value;
}
/******************************************************************************
*    Method:    SetVertex
******************************************************************************/
void King::ModelScaffold::SetVertex(const size_t & vertexIndexIn, uint8_t * dataIn)
{
    assert((bool)_vertexBufferMaster);
    auto length = _vertexFormat.GetByteSize();
    std::copy(dataIn, dataIn + length, &_vertexBufferMaster[vertexIndexIn]);
}
/******************************************************************************
*    Method:    SetVertexStream
******************************************************************************/
void King::ModelScaffold::SetVertexStream(const size_t& vertexStartIndexIn, uint8_t* dataIn, size_t numVertex)
{
    assert((bool)_vertexBufferMaster);
    auto length = _vertexFormat.GetByteSize() * numVertex;
    std::copy(dataIn, dataIn + length, &_vertexBufferMaster[vertexStartIndexIn]);
}
/******************************************************************************
*    Method:    SetIndexStream
******************************************************************************/
void King::ModelScaffold::SetIndexStream(const size_t& startIndexIn, uint8_t* dataIn, size_t numIndicies)
{
    assert((bool)_indexBufferMaster);
    auto length = GetIndexStride() * numIndicies;
    std::copy(dataIn, dataIn + length, &_indexBufferMaster[startIndexIn]);
}
/******************************************************************************
*    Method:    SetIndexStream
******************************************************************************/
void King::ModelScaffold::SetIndexStream(const size_t& startIndexIn, uint32_t* dataIn, size_t numIndicies)
{
    assert((bool)_indexBufferMaster);
    auto length = numIndicies;
    std::copy(dataIn, dataIn + length, &_indexBufferMaster[startIndexIn]);
}
/******************************************************************************
*    Method:    SetVertexElement
******************************************************************************/
void King::ModelScaffold::SetVertexElement(const size_t & vertexIndexIn, const VertexAttrib::enumDesc & propertyIn, const uint8_t * dataIn)
{
    assert((bool)_vertexBufferMaster);
    auto index = _vertexFormat.GetAttributeIndexFromDescription(propertyIn);
    assert(index != UINT16_MAX); // error
    auto offset = _vertexFormat.GetAttribute(index).GetOffset();
    assert(index < 8);

    auto length = _vertexFormat.GetAttribute(index).GetByteSize();
    std::copy(dataIn, dataIn + length, (&_vertexBufferMaster[vertexIndexIn]) + offset);
}

uint16_t King::VertexFormat::SetNext(VertexAttrib::enumDesc descIn, VertexAttrib::enumFormat formatIn)
{
    assert(nextAttribute < 8);
    attributes[nextAttribute]._offset += GetByteSize();
    attributes[nextAttribute]._desc = descIn;
    attributes[nextAttribute]._format = formatIn;
    ++nextAttribute;

    return nextAttribute - 1;
}

uint32_t King::VertexFormat::GetByteSize() const
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

uint32_t King::VertexFormat::GetAttributeByteStart(uint16_t indexIn) const
{
    if (indexIn == UINT16_MAX || indexIn >= nextAttribute) return 0;
    uint16_t i = 0;
    uint32_t s = 0;
    while (i < indexIn)
    {
        s += attributes[i].GetByteSize();
        ++i;
    }
    return s;
}

/******************************************************************************
*    Method:    Has VertexAttrib
******************************************************************************/
bool King::VertexFormat::Has(VertexAttrib::enumDesc descIn) const
{
    if (GetAttributeIndexFromDescription(descIn) != UINT16_MAX)  // ! error code
        return true;
    else
        return false;
}
bool King::VertexFormat::IsFirst(VertexAttrib::enumDesc descIn) const 
{ 
    if (nextAttribute > 0 && attributes[0]._desc == descIn) 
        return true; 
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

    if (_vb && _vertexFormat.Has(VertexAttrib::enumDesc::position))
    {
        rtn.Set(GetVertexPosition(0), GetVertexPosition(0));

        const auto to = GetNumIndicies();
        for (uint32_t i = 1; i < to; ++i)
        {
            rtn.Merge(GetVertexPosition(i));
        }
    }
    else
    {
        rtn.SetZero();
    }
    _boundingBox = rtn;
    return rtn;
}
/******************************************************************************
*    Method:    CalculateBoundingBox
******************************************************************************/
Box King::TriangleMesh::CalculateBoundingBox() 
{
    if (!_vb || !_vertexFormat.Has(VertexAttrib::enumDesc::position))
    {
        _boundingBox = Box();
        return _boundingBox;
    }

    // NOTE: if the line below fails, did the memory get reallocated with a Merge, Split, or other reallocation for a Mesh?
    // if so, go through the model and reset the references to VB and IB for all Meshes

    // optimize GetVertexPosition() for a loop
    const auto attrI = _vertexFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    const auto offset = _vertexFormat.GetAttribute(attrI).GetOffset();
    const auto vb = GetVB();
    const auto ib = GetIB();
    const auto bs = _vertexFormat.GetByteSize();
    float3 pos;

    Box rtn(reinterpret_cast<float*>(vb + *(ib + 0) * bs + offset));

    const auto to = GetNumVerticies();
    for (uint32_t i = 1; i < to; ++i)
    {
        pos = float3(reinterpret_cast<float*>(vb + *(ib + i) * bs + offset));
        rtn.Merge(pos);
    }

    _boundingBox = rtn;
    return rtn;
}
/******************************************************************************
*    Method:    Read_v1
*       Binary read
******************************************************************************/
bool King::TriangleMesh::Read_v1(ifstream& dataFileIn)
{
    if (!dataFileIn.is_open()) return false;
    // lambda
    auto ReadString = [&dataFileIn](string& strOut)
    {
        size_t len;
        dataFileIn.read(reinterpret_cast<char*>(&len), sizeof(len));
        char* temp = new char[len + 1];
        dataFileIn.read(temp, len);
        temp[len] = '\0';
        strOut = temp;
        delete[] temp;
    };

    ReadString(_name);
    ReadString(_materialName);
    dataFileIn.read(reinterpret_cast<char*>(&_vbStart), sizeof(_vbStart));
    dataFileIn.read(reinterpret_cast<char*>(&_ibStart), sizeof(_ibStart));
    dataFileIn.read(reinterpret_cast<char*>(&_numTriangles), sizeof(_numTriangles));
    dataFileIn.read(reinterpret_cast<char*>(&_boundingBox), sizeof(_boundingBox));

    // does not load referenced data which must be set outside the class

    if (dataFileIn.fail()) return false;
    return true;
}
/******************************************************************************
*    Method:    Write_v1
*       Binary write
******************************************************************************/
bool King::TriangleMesh::Write_v1(ofstream& outfileIn)
{
    if (!outfileIn.is_open()) return false;
    // lambda
    auto WriteString = [&outfileIn](const string& str)
    {
        size_t len = str.size();
        outfileIn.write(reinterpret_cast<const char*>(&len), sizeof(len));
        outfileIn.write(str.c_str(), len);
    };

    WriteString(_name);
    WriteString(_materialName);
    outfileIn.write(reinterpret_cast<char*>(&_vbStart), sizeof(_vbStart));
    outfileIn.write(reinterpret_cast<char*>(&_ibStart), sizeof(_ibStart));
    outfileIn.write(reinterpret_cast<char*>(&_numTriangles), sizeof(_numTriangles));
    outfileIn.write(reinterpret_cast<char*>(&_boundingBox), sizeof(_boundingBox));

    if (outfileIn.fail()) return false;
    return true;
}
/******************************************************************************
*    Class:    HeightGrid
*    Method:   InitializeXZ
*    Description: Generates a memory block of height values using interpolation
*        between verts of input mesh for fast lookup later on.  XZ is a reminder
*        that it operates in the XZ plane with Y as the height field.
*    Inputs:     triMeshIn        source of height data
*                densityIn        density of grid
*    Outputs:    heightGrid       re-initializes memory block 
*    Returns:    IntPoint2        X,Z size of height grid in vertex
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
    // points are 1 more than the number of cells
    long densityXPts = densityXIn + 1;
    long densityZPts = densityZIn + 1;
    _gridMem.Initialize((size_t)(densityXPts * densityZPts));
    _gridMem.Fill(0.0f);
    
    assert(_gridMem);

    // calculate height values from mesh
    // start with patch subcell size (strideX & Z)
    float strideX = boundingSize.GetX() / (float)densityXIn; // width
    float strideZ = boundingSize.GetZ() / (float)densityZIn; // depth
    float above = _boundingBox.GetMax().GetY() + 1.0f; // a little above for ray casting down
    Ray projectDown; 
    projectDown.SetDirection(float3(0.f, -1.f, 0.f));

    // test  each triangle in mesh until found
    uint32_t primeJ = 0;
    TriangleIndexed t;
    float3 p0, p1, p2;
    float3 ptOut;
    Triangle2DF triXZ;
    float epsilon = 0.01f; // small amount added to height to be above the terrain

    for (long z = 0; z < densityZPts; ++z)
    {
        // grid coordinates
        float3 testPt(_boundingBox.GetMin().GetX() + 0.0f, above, (float)z * strideZ + _boundingBox.GetMin().GetZ());
        bool repeat = false;
        for (long x = 0; x < densityXPts; ++x)
        {
            projectDown.SetOrigin(testPt);
            const float3 & origin = projectDown.GetOrigin();
            float2 originXZ(origin.GetX(), origin.GetZ());

            uint32_t numTri = triMeshIn.GetNumTriangles();
            do {
                if (repeat)
                {
                    repeat = false;
                }
                uint32_t j;
                for (j = primeJ; j < numTri; ++j) // all triangles
                {
                    t = triMeshIn[(uint32_t)j]; // individual triangle
                    p0 = t.GetVertexPosition(0);
                    p1 = t.GetVertexPosition(1);
                    p2 = t.GetVertexPosition(2);
                    triXZ = Triangle2DF(float2(p0.f[0], p0.f[2]), float2(p1.f[0], p1.f[2]), float2(p2.f[0], p2.f[2]));

                    // rectangle test first which is much cheaper and fact that most tests fail
                    Rectangle2DF bb(triXZ);
                    if (bb.Intersects(originXZ) && t.Intersects(projectDown, &ptOut))
                    {
                        _gridMem[((uint32_t)z * densityXPts + (uint32_t)x)] = ptOut.GetY() + epsilon;
                        repeat = false;
                        primeJ = j; // chances are high that the next hit is the same triangle
                        break; // triangle found, there can be only one
                    }
                }
                if (j >= numTri)
                {
                    // we started mid loop and did not find a triangle that contains the point, so start over
                    numTri = primeJ;
                    primeJ = 0;
                    if (numTri == primeJ)
                        repeat = false;
                    else
                        repeat = true;
                }
            } while (repeat);

            testPt.SetX(testPt.GetX() + strideX);
        }
    }
    SetXZSize(int2(densityXPts, densityZPts));
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
        return 0.f;
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
        |origin      |
        |(0,0)       |
        ------------>(+)w_x
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
        checkDiag = true; // why not here????? added but not tested CHK 11/12/2020
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

bool King::HeightGrid::Load(const string& nameIn)
{
    std::ifstream infile(nameIn + ".mb", std::ifstream::binary);
    if (infile.bad()) return false;

    _gridMem.Read(infile);

    infile.read(reinterpret_cast<char*>(&(_xzSize.i[0])), sizeof(int));
    infile.read(reinterpret_cast<char*>(&(_xzSize.i[1])), sizeof(int));

    infile.read(reinterpret_cast<char*>(&_boundingBox.pt_min.f[0]), sizeof(float));
    infile.read(reinterpret_cast<char*>(&_boundingBox.pt_min.f[1]), sizeof(float));
    infile.read(reinterpret_cast<char*>(&_boundingBox.pt_min.f[2]), sizeof(float));

    infile.read(reinterpret_cast<char*>(&_boundingBox.pt_max.f[0]), sizeof(float));
    infile.read(reinterpret_cast<char*>(&_boundingBox.pt_max.f[1]), sizeof(float));
    infile.read(reinterpret_cast<char*>(&_boundingBox.pt_max.f[2]), sizeof(float));

    if (infile.fail()) return false;

    //cout << "Loaded: " << nameIn << '\n';
    //cout << "  _gridMem length " << _gridMem.GetLength() << '\n';
    //cout << "  _xzSize " << _xzSize << '\n';
    //cout << "  _boundingBox " << _boundingBox << '\n';

    return true;
}

void King::HeightGrid::Save(const string& nameIn)
{
    std::ofstream outfile(nameIn + ".mb", std::ofstream::binary | std::ofstream::trunc);
    if (outfile.bad()) return;

    _gridMem.Write(outfile);

    outfile.write(reinterpret_cast<const char*>(&(_xzSize.i[0])), sizeof(int));
    outfile.write(reinterpret_cast<const char*>(&(_xzSize.i[1])), sizeof(int));

    outfile.write(reinterpret_cast<const char*>(&_boundingBox.pt_min.f[0]), sizeof(float));
    outfile.write(reinterpret_cast<const char*>(&_boundingBox.pt_min.f[1]), sizeof(float));
    outfile.write(reinterpret_cast<const char*>(&_boundingBox.pt_min.f[2]), sizeof(float));

    outfile.write(reinterpret_cast<const char*>(&_boundingBox.pt_max.f[0]), sizeof(float));
    outfile.write(reinterpret_cast<const char*>(&_boundingBox.pt_max.f[1]), sizeof(float));
    outfile.write(reinterpret_cast<const char*>(&_boundingBox.pt_max.f[2]), sizeof(float));

    //cout << "Saved: " << nameIn << '\n';
    //cout << "  _gridMem length " << _gridMem.GetLength() << '\n';
    //cout << "  _xzSize " << _xzSize << '\n';
    //cout << "  _boundingBox " << _boundingBox << '\n';
}

/******************************************************************************
*    Method:    ConstructPerspectiveFrustum
******************************************************************************/
/*
         ltb---------rtb
           /|      _/|
          / |   _/   |
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
    _FrustumCorners[lbf] = float3(-NearX, -NearY, -NearClip);    // Near lower left
    _FrustumCorners[ltf] = float3(-NearX, NearY, -NearClip);    // Near upper left
    _FrustumCorners[rbf] = float3(NearX, -NearY, -NearClip);    // Near lower right
    _FrustumCorners[rtf] = float3(NearX, NearY, -NearClip);    // Near upper right
    _FrustumCorners[lbb] = float3(-FarX, -FarY, -FarClip);    // Far lower left
    _FrustumCorners[ltb] = float3(-FarX, FarY, -FarClip);    // Far upper left
    _FrustumCorners[rbb] = float3(FarX, -FarY, -FarClip);    // Far lower right
    _FrustumCorners[rtb] = float3(FarX, FarY, -FarClip);    // Far upper right

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
    _FrustumCorners[lbf] = float3(Left, Bottom, -Front);    // Near lower left
    _FrustumCorners[ltf] = float3(Left, Top, -Front);    // Near upper left
    _FrustumCorners[rbf] = float3(Right, Bottom, -Front);    // Near lower right
    _FrustumCorners[rtf] = float3(Right, Top, -Front);    // Near upper right
    _FrustumCorners[lbb] = float3(Left, Bottom, -Back);    // Far lower left
    _FrustumCorners[ltb] = float3(Left, Top, -Back);    // Far upper left
    _FrustumCorners[rbb] = float3(Right, Bottom, -Back);    // Far lower right
    _FrustumCorners[rtb] = float3(Right, Top, -Back);    // Far upper right

    // Define the bounding planes
    _FrustumPlanes[kNearPlane] = Plane(0.0f, 0.0f, -1.0f, -Front);
    _FrustumPlanes[kFarPlane] = Plane(0.0f, 0.0f, 1.0f, Back);
    _FrustumPlanes[kLeftPlane] = Plane(1.0f, 0.0f, 0.0f, -Left);
    _FrustumPlanes[kRightPlane] = Plane(-1.0f, 0.0f, 0.0f, Right);
    _FrustumPlanes[kTopPlane] = Plane(0.0f, -1.0f, 0.0f, Bottom);
    _FrustumPlanes[kBottomPlane] = Plane(0.0f, 1.0f, 0.0f, -Top);
}

void King::Frustum::ConstructOrthographicFrustum(const Box& in, const quat *rQIn)
{
    auto corners = in.GetCorners8(rQIn);
    // Define the frustum corners
    _FrustumCorners[lbf] = corners[lbf];
    _FrustumCorners[ltf] = corners[ltf];
    _FrustumCorners[rbf] = corners[rbf];
    _FrustumCorners[rtf] = corners[rtf];
    _FrustumCorners[lbb] = corners[lbb];
    _FrustumCorners[ltb] = corners[ltb];
    _FrustumCorners[rbb] = corners[rbb];
    _FrustumCorners[rtb] = corners[rtb];

    // Define the bounding planes with CW ordering (to point normals inward)
    _FrustumPlanes[kNearPlane] = Plane(corners[rtf], corners[rbf], corners[lbf]);
    _FrustumPlanes[kFarPlane] = Plane(corners[lbb], corners[rbb], corners[rtb]);
    _FrustumPlanes[kLeftPlane] = Plane(corners[lbf], corners[lbb], corners[ltb]);
    _FrustumPlanes[kRightPlane] = Plane(corners[rtb], corners[rbb], corners[rbf]);
    _FrustumPlanes[kTopPlane] = Plane(corners[ltb], corners[rtb], corners[rtf]);
    _FrustumPlanes[kBottomPlane] = Plane(corners[lbf], corners[rbf], corners[rbb]);
}

King::Frustum::Frustum(const DirectX::XMMATRIX &projectionMatrixIn)
{
    const float* ProjMatF = (const float*)&projectionMatrixIn;

    const float HTan = 1.0f / ProjMatF[0]; // Horz.Tangent
    const float VTan = 1.0f / ProjMatF[5]; // Vert.Tangent
    const float DTan = 1.0f / ProjMatF[10]; // Depth.Tangent

    // Identify if the projection is perspective or orthographic by looking at the 4th row.
    if (ProjMatF[3] == 0.0f && ProjMatF[7] == 0.0f && ProjMatF[11] == 0.0f && ProjMatF[15] == 1.0f)
    {
        // Orthographic
        float Left = (-1.0f - ProjMatF[12]) * HTan;
        float Right = (1.0f - ProjMatF[12]) * HTan;
        float Top = (1.0f - ProjMatF[13]) * VTan;
        float Bottom = (-1.0f - ProjMatF[13]) * VTan;
        float Front = (0.0f - ProjMatF[14]) * DTan;
        float Back = (1.0f - ProjMatF[14]) * DTan;

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

        if (DTan > 0.0f)    // Reverse Z
        {
            FarClip = ProjMatF[14] * DTan;
            NearClip = FarClip / (DTan + 1.0f);
        }
        else
        {
            NearClip = ProjMatF[14] * DTan;
            FarClip = NearClip / (DTan + 1.0f);
        }
        ConstructPerspectiveFrustum(HTan, VTan, NearClip, FarClip);
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
std::vector<float3> King::Model::CalculateVertexSkinPositions(size_t meshIndex)
{
    assert(meshIndex > _meshes.size() - 1);
    auto& m = _meshes[meshIndex];

    std::vector<float3> skinPositions;
    float4 pos; // position of skinned vertex, w = 1.0f
    uint8_t n; // number of bones influencing the vertex
    uint8_t bi; // absolute bone index
    DirectX::XMMATRIX boneToParentTrans; // bind pose transform of the bone
    DirectX::XMMATRIX M; // transform of the bone
    float w; // weight of the bone influence

    // identify vertex positions
    const auto vb = m->GetVB();
    assert(vb != nullptr);
    const auto ib = m->GetIB();
    const auto& ibStart = m->GetIBStart();
    const auto& vFormat = m->GetVertexFormat();
    const auto& stride = vFormat.GetByteSize();
    const auto& attrP = vFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    const auto& offset = vFormat.GetAttribute(attrP).GetOffset();
    // byte pointer to start of vertex data
    auto GetVertexAddr = [&](const uint32_t indexIn) { return vb + ((size_t) * (ib + ibStart + indexIn) * stride); };

    // copy vertex positions
    const auto numVerticies = m->GetNumVerticies();
    for (uint32_t i = 0; i < numVerticies; ++i)
    {
        skinPositions.push_back(float3(reinterpret_cast<float*>(GetVertexAddr(i) + offset)));
    }
    // transform vertex positions
    size_t boneIndexOffset = 0; // into skin.boneIndex[]
    for (uint32_t i = 0; i < numVerticies; ++i)
    {
        pos = float4(skinPositions[i], 1.0f);
        float3 posSkinned;
        n = _boneHierarchy->_boneInfluenceCount[i]; // number of bones influencing this vertex

        // apply bone influences
        for (uint8_t b = 0; b < n; ++b)
        {
            bi = _boneHierarchy->_boneIndex[boneIndexOffset + b]; // bone that influences this vertex
            w = _boneHierarchy->_boneWeight[boneIndexOffset + b]; // weight of this influence
            M = DirectX::XMLoadFloat4x4(&_boneHierarchy->_skeleton._bones[bi]._transform); // transform of bone (rotation, scale)
            boneToParentTrans = DirectX::XMLoadFloat4x4(&_boneHierarchy->_skeleton._toParentTransform[bi]); // heiarchy

            //auto detBi = DirectX::XMMatrixDeterminant(boneToParentTrans);
            auto BiInverse = DirectX::XMMatrixInverse(nullptr, boneToParentTrans);

            auto r1 = DirectX::XMVector4Transform(pos.GetVecConst(), M); // bone orientation
            auto r2 = DirectX::XMVector4Transform(r1, BiInverse); // relative to parent

            posSkinned += w * float3(r2);
        }
        skinPositions[i] = posSkinned;
        boneIndexOffset += n;
    }

    return skinPositions; // compiler should involk std::move() optimization
}
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
    const auto vb = m->GetVB();
    assert(vb != nullptr);
    const auto ib = m->GetIB();
    const auto & ibStart = m->GetIBStart();
    const auto & vFormat = m->GetVertexFormat();
    const auto & stride = vFormat.GetByteSize();
    const auto & attrP = vFormat.GetAttributeIndexFromDescription(VertexAttrib::enumDesc::position);
    const auto & offset = vFormat.GetAttribute(attrP).GetOffset();
    // byte pointer to start of vertex data
    auto GetVertexAddr = [&](const uint32_t indexIn) { return vb + ((size_t) *(ib + ibStart + indexIn) * stride); };

    // copy vertex positions
    const auto numVerticies = m->GetNumVerticies();
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
            
            //auto detBi = DirectX::XMMatrixDeterminant(boneToParentTrans);
            auto BiInverse = DirectX::XMMatrixInverse(nullptr, boneToParentTrans);

            auto r1 = DirectX::XMVector4Transform(pos.GetVecConst(), M); // bone orientation
            auto r2 = DirectX::XMVector4Transform(r1, BiInverse); // relative to parent

            posSkinned += w * float3(r2);
        }
        skinPositions[i] = posSkinned;
        boneIndexOffset += n;
    }
    
    return skinPositions; // compiler should involk std::move() optimization
}
/******************************************************************************
*    Method:    Read_v1
******************************************************************************/
bool King::SkinnedModel::Read_v1(ifstream& dataFileIn)
{
    bool good = true;

    if (!dataFileIn.is_open()) return false;
    // lambda
    auto ReadString = [&dataFileIn](string& strOut)
    {
        size_t len;
        dataFileIn.read(reinterpret_cast<char*>(&len), sizeof(len));
        char* temp = new char[len + 1];
        dataFileIn.read(temp, len);
        temp[len] = '\0';
        strOut = temp;
        delete[] temp;
    };
    // read contents
    good = Model::Read_v1(dataFileIn);

    if (good)
    {
        _boneHierarchy = BoneHierarchy::CreateUnique();
        good = _boneHierarchy->Read_v1(dataFileIn);
    }

    if (dataFileIn.fail() || !good) return false;
    return true;
}
/******************************************************************************
*    Method:    Write_v1
******************************************************************************/
bool King::SkinnedModel::Write_v1(ofstream& outfileIn)
{
    if (!outfileIn.is_open()) return false;
    // lambda
    auto WriteString = [&outfileIn](const string& str)
    {
        size_t len = str.size();
        outfileIn.write(reinterpret_cast<const char*>(&len), sizeof(len));
        outfileIn.write(str.c_str(), len);
    };
    // write contents
    if (!Model::Write_v1(outfileIn))
        return false;
    if (_boneHierarchy)
        if (!_boneHierarchy->Write_v1(outfileIn)) return false;

    if (outfileIn.fail()) return false;
    return true;
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
    rtn._translation = DirectX::XMVectorLerp(_translation.GetVecConst(), in._translation.GetVecConst(), fractionIn);
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
*       unique              true then retrieve each vertex in order, else use the index buffer and return vertex list with duplicates as referenced in the index buffer
******************************************************************************/
vector<vector<float>> King::ModelScaffold::GetDataAttribute(VertexAttrib::enumDesc attributeEnum, uint32_t vbStartIn, uint32_t ibStartIn, bool unique)
{
    assert(_vertexFormat.Has(attributeEnum));

    vector<vector<float>> v;
    auto & vb = GetVertexBufferMaster(); // uint8_t but with stride indexing
    uint8_t * vbAddr = vb.GetData();
    auto vbStride = vb.GetStride();
    const auto & ib = GetIndexBufferMaster(); // uint32_t
    const auto attribIndex = _vertexFormat.GetAttributeIndexFromDescription(attributeEnum);
    const auto atrribute = _vertexFormat.GetAttribute(attribIndex);
    const auto attribOffset = atrribute.GetOffset();
    const auto atrribFormat = atrribute.GetFormat();

    auto lamda = [&](const uint32_t &i)
    {
        if (atrribFormat == VertexAttrib::enumFormat::format_float32x4)
        {
            {
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
            {
                float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
                vector<float> x(1);
                x[0] = *v_data;
                v.push_back(x);
            }
        }
    };

    // cycle through indicies and store vertex info    
    const auto indexCount = GetIndexCount();
    const auto vertexCount = GetVertexCount();

    if (unique)
        for (uint32_t i = 0; i < vertexCount; ++i)
            lamda(i);
    else
        for (uint32_t i = 0; i < indexCount; ++i)
            lamda(_indexBufferMaster[i]);

    return v;
}
/******************************************************************************
*    Method:    SetDataAttribute
*       Sets the data of one attribute.
*   INPUTS:
*       dataIn              data to copy
*       attributeEnum       attribute data type to retrieve
*       vbStartIn           vertex offset if master is assembled without index modification
*       ibStartIn           index offset if indicies are shifted for a subset
*       unique              true then retrieve each vertex in order, else use the index buffer and return vertex list with duplicates as referenced in the index buffer
******************************************************************************/
void King::ModelScaffold::SetDataAttribute(vector<vector<float>>& dataIn, VertexAttrib::enumDesc attributeEnum, uint32_t vbStartIn, uint32_t ibStartIn, bool unique)
{
    assert(_vertexFormat.Has(attributeEnum));

    vector<vector<float>> v;
    auto& vb = GetVertexBufferMaster(); // uint8_t but with stride indexing
    uint8_t* vbAddr = vb.GetData();
    auto vbStride = vb.GetStride();
    const auto& ib = GetIndexBufferMaster(); // uint32_t
    const auto attribIndex = _vertexFormat.GetAttributeIndexFromDescription(attributeEnum);
    const auto atrribute = _vertexFormat.GetAttribute(attribIndex);
    const auto attribOffset = atrribute.GetOffset();
    const auto atrribFormat = atrribute.GetFormat();

    auto lamda = [&](const uint32_t& i)
    {
        if (atrribFormat == VertexAttrib::enumFormat::format_float32x4)
        {
            float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
            *v_data = dataIn[i][0];
            *(v_data + 1) = dataIn[i][1];
            *(v_data + 2) = dataIn[i][2];
            *(v_data + 3) = dataIn[i][3];
        }
        else if (atrribFormat == VertexAttrib::enumFormat::format_float32x3)
        {
            float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
            *v_data = dataIn[i][0];
            *(v_data + 1) = dataIn[i][1];
            *(v_data + 2) = dataIn[i][2];
        }
        else if (atrribFormat == VertexAttrib::enumFormat::format_float32x2)
        {
            float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
            *v_data = dataIn[i][0];
            *(v_data + 1) = dataIn[i][1];
        }
        else if (atrribFormat == VertexAttrib::enumFormat::format_float32)
        {
            float* v_data = reinterpret_cast<float*>(GetVertexAddr(i, vbStartIn, ibStartIn) + attribOffset);
            *v_data = dataIn[i][0];
        }
    };

    // cycle through indicies and store vertex info    
    const auto indexCount = GetIndexCount();
    const auto vertexCount = GetVertexCount();

    if (unique)
        for (uint32_t i = 0; i < vertexCount; ++i)
            lamda(i);
    else
        for (uint32_t i = 0; i < indexCount; ++i)
            lamda(_indexBufferMaster[i]);
}


bool __vectorcall King::Triangle::Intersects(const Triangle& triIn, float3* intersectionOut) const 
{
    // Möller–Trumbore intersection algorithm
    const float3& A = pt[0];
    const float3& B = pt[1];
    const float3& C = pt[2];
    const float3& P = triIn.pt[0];
    const float3& Q = triIn.pt[1];
    const float3& R = triIn.pt[2];

    // Edge vectors
    float3 AB = B - A;
    float3 AC = C - A;
    float3 PQ = Q - P;
    float3 PR = R - P;

    // Calculate the normal for the triangle (A, B, C)
    float3 N = Cross(AB, AC);

    // Check for zero area triangle
    if (N.IsOrNearZero()) return false; // Degenerate triangle

    // Normalize the normal vector
    N.Normalize();

    // Find the intersection line of the two triangles (P, Q, R) and (A, B, C)
    float3 E1 = Cross(PQ, N);
    float3 E2 = Cross(PR, N);

    // Calculate the determinant
    float det = Dot(AB, E2);
    if (fabs(det) < 1e-8f) return false; // Parallel planes

    // Calculate the distance from point A to the plane
    float invDet = 1.0f / det;
    float3 AP = P - A;

    // Barycentric coordinates
    float u = Dot(AP, E1) * invDet;
    float v = Dot(PQ, E2) * invDet;

    // Check if intersection lies within triangle A-B-C
    if (u < 0.0f || v < 0.0f || u + v > 1.0f) return false;

    // Calculate intersection point
    float3 intersectionPoint = P + u * PQ;

    // Check if the intersection point lies within triangle P-Q-R
    float3 AE = intersectionPoint - A;
    float3 APQ = Cross(AB, AE);
    if (Dot(N, APQ) < 0.0f) return false; // Outside triangle

    // Check for intersection with triangle (P, Q, R)
    float3 APQCross = Cross(E1, PR);
    if (Dot(N, APQCross) < 0.0f) return false; // Outside triangle

    // Output the intersection point if requested
    if (intersectionOut) {
        *intersectionOut = intersectionPoint;
    }

    return true; // Intersection found
}

inline bool __vectorcall King::Triangle::Intersects(const Quad& qIn, float3* intersectionOut) const { return qIn.Intersects(*this, intersectionOut); }

inline bool __vectorcall King::Triangle::Intersects(const Plane& pIn, float3* intersectionOut) const { return pIn.Intersects(*this, intersectionOut); }

inline bool __vectorcall King::Triangle::Intersects(const Sphere& sIn, float3* intersectionOut) const { return sIn.Intersects(*this, intersectionOut); }

float3 __vectorcall Triangle::FindNearestPoint(const float3& pt3In) const 
{
    // Check if the point is inside the triangle
    if (Contains(pt3In)) {
        return pt3In; // Return the point itself if it's inside the triangle
    }

    float3 nearestPoint = pt[0]; // Start by assuming the nearest point is the first vertex
    float minDistSquared = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(pt3In - pt[0])); // Initialize with the distance to the first vertex

    // Step 2: Check each vertex and edge for the nearest point
    for (int i = 0; i < 3; ++i) {
        float3 currentVertex = pt[i];
        float distSquared = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(pt3In - currentVertex));
        if (distSquared < minDistSquared) {
            minDistSquared = distSquared;
            nearestPoint = currentVertex; // Update nearest point if the current vertex is closer
        }

        // Check the edge formed by the current vertex and the next vertex (wrap around)
        int nextIndex = (i + 1) % 3;
        float3 edge = pt[nextIndex] - currentVertex;
        float3 toPoint = pt3In - currentVertex;

        // Project the point onto the edge
        float edgeLengthSquared = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(edge));
        if (edgeLengthSquared > 0) { // Prevent division by zero
            float minor = min(1.0f, Dot(toPoint, edge));
            float t = max(0.0f, minor / edgeLengthSquared);
            float3 projection = currentVertex + t * edge; // The closest point on the edge
            distSquared = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(pt3In - projection));
            if (distSquared < minDistSquared) {
                minDistSquared = distSquared;
                nearestPoint = projection; // Update nearest point if the edge projection is closer
            }
        }
    }

    return nearestPoint; // Return the closest point found
}


bool __vectorcall King::Quad::Intersects(const Triangle& triIn, float3* intersectionOut) const 
{
    // Get the vertices of the quad
    const float3& A = GetVertex(0);
    const float3& B = GetVertex(1);
    const float3& C = GetVertex(2);
    const float3& D = GetVertex(3);

    // Split the quad into two triangles: ABC and ACD
    Triangle triangle1(A, B, C);
    Triangle triangle2(A, C, D);

    // Check intersection with the first triangle (ABC)
    float3 intersectionPoint;
    if (triangle1.Intersects(triIn, &intersectionPoint)) {
        if (intersectionOut) {
            *intersectionOut = intersectionPoint; // Store intersection point
        }
        return true; // Intersection found with triangle1
    }

    // Check intersection with the second triangle (ACD)
    if (triangle2.Intersects(triIn, &intersectionPoint)) {
        if (intersectionOut) {
            *intersectionOut = intersectionPoint; // Store intersection point
        }
        return true; // Intersection found with triangle2
    }

    return false; // No intersection found with either triangle
}

// Helper function to find the intersection point between a plane and an edge of the triangle
bool Plane::FindIntersectionWithEdge(const float3& v0, const float3& v1, float3& intersectionOut) const {
    // Compute the direction of the edge
    float3 edgeDir = v1 - v0;

    // Compute the distance from the starting vertex to the plane
    float d0 = DistanceFromPoint(v0);
    float d1 = DistanceFromPoint(v1);

    // Check if the edge intersects the plane
    if (d0 * d1 < 0) { // One vertex is on one side and the other is on the other side
        // Interpolation factor to find the intersection
        float t = d0 / (d0 - d1);
        intersectionOut = v0 + edgeDir * t; // Linear interpolation to find the intersection point
        return true; // Intersection found
    }

    return false; // No intersection
}

inline King::Plane::Plane(const Position origin, const float3 normalToPlane) 
{ 
    v = DirectX::XMPlaneFromPointNormal(static_cast<DirectX::XMVECTOR>(origin), DirectX::XMPlaneNormalizeEst(normalToPlane)); 
    float3 w = DirectX::XMVector3Dot((float3)origin, normalToPlane);
    w = -w;
    v = DirectX::XMVectorSelect(w, normalToPlane, g_XMSelect1110.v);
}

King::Plane::Plane(const Position& point1, const Position& point2, const Position& point3)
{ 
    // by using 21 inlieu of 12, the LHS cross product in directx becomes a RHS cross product like we want it
    float3 V21 = (float3)point1 - (float3)point2;
    float3 V31 = (float3)point1 - (float3)point3;

    float3 n = Cross(V21, V31); 
    n.MakeNormalize();

    float3 d = Dot(n, (float3)point1);
    d = -d;

    v = DirectX::XMVectorSelect(d, n, g_XMSelect1110.v);
}

// double dispatch

inline bool King::Plane::Collision(Point const& pointIn) const { return Intersects(pointIn); }

inline bool King::Plane::Collision(Line const& lineIn) const { Point ptOut; return Intersects(lineIn, &ptOut); }

inline bool King::Plane::Collision(Ray const& rayIn) const { Point ptOut; return Intersects(rayIn, &ptOut); }

inline bool King::Plane::Collision(Plane const& planeIn) const { Line lineOut; return Intersects(planeIn, &lineOut); }

inline bool King::Plane::Collision(Sphere const& sphereIn) const { return Intersects((Sphere)sphereIn); }

inline bool King::Plane::Collision(Box const& boxIn) const { return Intersects(boxIn); }

inline bool King::Plane::Collision(Pyramid const& pyramidIn) const { return false; }// { return pyramidIn.Collision(*this); }

inline bool __vectorcall King::Plane::Intersects(const float3 pointIn) const
{
    float dist = fabs(DistanceFromPoint(pointIn));
    if (dist < 5.e-5f)
        return true;
    else
        return false;
}

inline bool __vectorcall King::Plane::Intersects(const Point & pointIn) const
{
    float dist = fabs(DistanceFromPoint(pointIn));
    if(dist < 5.e-5f)
        return true;
    else 
        return false;
}

bool __vectorcall King::Plane::Intersects(const Plane& planeIn, Line *lineOut) const
{
    //XMVECTOR* pLinePoint1,
    //    XMVECTOR* pLinePoint2,
    //    FXMVECTOR  P1,
    //    FXMVECTOR  P2
    auto n = GetNormal();
    auto u = planeIn.GetNormal();
    // parallel test
    if (Dot(n, u) == 0.f)
    {
        if (float4(*this) == float4(planeIn))
        {
            // there is no line of intersection (it is the whole plane)
            (*lineOut).Set(Line(GetOrigin(), planeIn.GetOrigin()));
            return true;
        }
        return false;
    }

    XMVECTOR V1 = float3::CrossProduct(planeIn, *this);

    XMVECTOR LengthSq = DirectX::XMVector3LengthSq(V1);

    XMVECTOR V2 = float3::CrossProduct(planeIn, V1);

    XMVECTOR P1W = DirectX::XMVectorSplatW(*this);
    XMVECTOR pt = DirectX::XMVectorMultiply(V2, P1W);

    XMVECTOR V3 = float3::CrossProduct(V1, *this);

    XMVECTOR P2W = DirectX::XMVectorSplatW(planeIn);
    pt = DirectX::XMVectorMultiplyAdd(V3, P2W, pt);

    XMVECTOR LinePoint1 = DirectX::XMVectorDivide(pt, LengthSq);
    XMVECTOR LinePoint2 = DirectX::XMVectorAdd(LinePoint1, V1);
    (*lineOut).SetVertex(0, LinePoint1);
    (*lineOut).SetVertex(1, LinePoint2);

    return true;
}

bool King::Plane::Intersects(const Line& lineIn, float3* ptOut) const
{
    // reference http://geomalgorithms.com/a05-_intersect-1.html
    auto P0 = lineIn.GetVertex(0);
    auto P1 = lineIn.GetVertex(1);
    auto V0 = GetOrigin();
    auto n = GetNormal();
    auto u = lineIn.GetLengthVector();
    // parallel test
    if (Dot(n, u) == 0.f)
    {
        if (Dot(n, P0 - V0) == 0)
        {
            // P0 is on the plane and the line is parallel to the plane
            // so all points intersects, return the mid point of the line
            *ptOut = lineIn.GetCenter();
            return true;
        }
        return false;
    }
    auto s = Dot(n, V0 - P0) / Dot(n, P1 - P0);
    // If the line L is a finite segment from P0 to P1, then one just has to 
    // check that 0 <= s <= 1 to verify that there is an intersection 
    // between the segment and the plane.
    if (s >= 0.f && s <= 1.f)
    {
        *ptOut = u * s + P0;
        return true;
    }
    else
        return false;
}

inline bool __vectorcall King::Plane::Intersects(const Line& lineIn, Point* ptOut) const
{
    // reference http://geomalgorithms.com/a05-_intersect-1.html
    auto P0 = lineIn.GetVertex(0);
    auto P1 = lineIn.GetVertex(1);
    auto V0 = GetOrigin();
    auto n = GetNormal();
    auto u = lineIn.GetLengthVector();
    // parallel test
    if (Dot(n, u) == 0.f)
    {
        if (Dot(n, P0 - V0) == 0)
        {
            // P0 is on the plane and the line is parallel to the plane
            // so all points intersects, return the mid point of the line
            *ptOut = lineIn.GetCenter();
            return true;
        }
        return false;
    }
    auto s = Dot(n, V0 - P0) / Dot(n, P1 - P0);
    // If the line L is a finite segment from P0 to P1, then one just has to 
    // check that 0 <= s <= 1 to verify that there is an intersection 
    // between the segment and the plane.
    if (s >= 0.f && s <= 1.f)
    {
        *ptOut = u * s + P0;
        return true;
    }
    else
        return false;
}

inline bool __vectorcall King::Plane::Intersects(const Ray& rayIn, Point* ptOut) const
{
    // reference http://geomalgorithms.com/a05-_intersect-1.html
    auto P0 = rayIn.GetOrigin();
    auto V0 = GetOrigin();
    auto n = GetNormal();
    auto u = rayIn.GetDirection();
    auto w = P0 - V0;
    // parallel test
    if (Dot(n, u) == 0.f)
    {
        if (Dot(n, w) == 0)
        {
            // P0 is on the plane and the line is parallel to the plane
            // so all points intersects, return the origin of the ray
            *ptOut = rayIn.GetOrigin();
            return true;
        }
        return false;
    }
    auto s = Dot(n, -w) / Dot(n, u);
    // For a positive ray, there is an intersection with the plane when s >= 0.
    if (s >= 0.f)
    {
        *ptOut = u * s + P0;
        return true;
    }
    else
        return false;
}

inline bool __vectorcall King::Plane::Intersects(const Sphere & sphereIn) const
{
    float4 center(sphereIn.GetCenter(), 1.0f);
    float4 radius(sphereIn.GetRadius());
    float4 dist(DirectX::XMVector4Dot(center, v));

    float4 greater(DirectX::XMVectorGreater(dist, radius) );
    float4 less(DirectX::XMVectorLess(dist, -radius) );
    // in front of plane
    if (DirectX::XMVector4EqualInt(greater, DirectX::XMVectorTrueInt()))
        return false;
    // in back of plane
    if (DirectX::XMVector4EqualInt(less, DirectX::XMVectorTrueInt()))
        return false;
    // intersecting plane
    return true;
}
bool King::Plane::Intersects(const Triangle& triIn, float3* intersectionOut) const {
    // Get triangle vertices
    XMVECTOR v0 = triIn.GetVertex(0);
    XMVECTOR v1 = triIn.GetVertex(1);
    XMVECTOR v2 = triIn.GetVertex(2);

    // Calculate distances from the triangle vertices to the plane
    XMVECTOR Dist0 = DirectX::XMVector4Dot(v0, v);
    XMVECTOR Dist1 = DirectX::XMVector4Dot(v1, v);
    XMVECTOR Dist2 = DirectX::XMVector4Dot(v2, v);

    // Determine the minimum and maximum distances
    XMVECTOR MinDist = DirectX::XMVectorMin(Dist0, Dist1);
    MinDist = DirectX::XMVectorMin(MinDist, Dist2);

    XMVECTOR MaxDist = DirectX::XMVectorMax(Dist0, Dist1);
    MaxDist = DirectX::XMVectorMax(MaxDist, Dist2);

    // Zero vector
    XMVECTOR Zero = DirectX::XMVectorZero();

    // Check for nearly parallel planes
    float3 planeNormal = GetNormal();
    float3 triangleNormal = triIn.GetNormalCCW(); // Assuming counter-clockwise order
    // Calculate the dot product
    float dotProduct = Dot(planeNormal, triangleNormal);
    // If the dot product is close to zero, the plane and triangle are nearly parallel
    if (fabs(dotProduct) < 1e-6f) {
        return false;
    }

    // Check if triangle vertices are outside or fully inside the plane
    XMVECTOR Outside = DirectX::XMVectorGreater(MinDist, Zero);
    XMVECTOR Inside = DirectX::XMVectorLess(MaxDist, Zero);

    // If fully outside the plane
    if (DirectX::XMVector4EqualInt(Outside, DirectX::XMVectorTrueInt())) {
        return false; // No intersection
    }

    // If fully inside the plane
    if (DirectX::XMVector4EqualInt(Inside, DirectX::XMVectorTrueInt())) {
        return false; // No intersection
    }

    // If intersecting the plane, calculate the intersection point
    if (intersectionOut) {
        // Calculate the intersection point using the triangle's centroid as a reference
        float3 intersectionPoint = (float3(v0) + float3(v1) + float3(v2)) / 3.0f; // Average point

        // Check where this average point lies in relation to the plane
        float distanceToPlane = Dot(intersectionPoint, planeNormal) + GetW();

        if (intersectionOut && fabs(distanceToPlane) < 1e-6) { // On the plane
            *intersectionOut = intersectionPoint; // Output the intersection point
            return true;
        }
    }

    // If the triangle intersects the plane but we couldn't find a specific intersection point
    return true;
}



inline bool __vectorcall King::Plane::Intersects(const Box& boxIn) const
{
    // FastIntersectAxisAlignedBoxPlane from DirectXCollision.inl

    // Compute the distance to the center of the box.
    XMVECTOR Dist = DirectX::XMVector4Dot(boxIn.GetCenter(), v);

    // Project the axes of the box onto the normal of the plane.  Half the
    // length of the projection (sometime called the "radius") is equal to
    // h(u) * abs(n dot b(u))) + h(v) * abs(n dot b(v)) + h(w) * abs(n dot b(w))
    // where h(i) are extents of the box, n is the plane normal, and b(i) are the
    // axes of the box. In this case b(i) = [(1,0,0), (0,1,0), (0,0,1)].
    XMVECTOR Radius = DirectX::XMVector3Dot(boxIn.GetExtents(), DirectX::XMVectorAbs(v));

    // Outside the plane?
    XMVECTOR Outside = DirectX::XMVectorGreater(Dist, Radius);

    // Fully inside the plane?
    XMVECTOR Inside = DirectX::XMVectorLess(Dist, DirectX::XMVectorNegate(Radius));

    // in Outside of plane
    if (DirectX::XMVector4EqualInt(Outside, DirectX::XMVectorTrueInt()))
        return false;
    // in inside of plane
    if (DirectX::XMVector4EqualInt(Inside, DirectX::XMVectorTrueInt()))
        return false;
    // intersecting plane
    return true;
}
/******************************************************************************
*   Class Frustum
******************************************************************************/
// Streams
// json
// operators
// methods
// Positive normals indicate which side of the face the box is on
bool __vectorcall King::Frustum::Intersect(const Sphere sphereIn) const
{
    float3 c(sphereIn.GetCenter());
    float radius = sphereIn.GetRadius();

    for (int i = kNearPlane; i < kINVALID; ++i)
    {
        Plane p = GetFrustumPlane((PlaneID)i);

        // 0 is on plane (-0.0005f to 0.0005f)
        if (p.DistanceFromPoint(c) + radius < -0.0005f)
            return false;
    }
    return true;
}
bool King::Frustum::Intersect(Box const& boxIn, const Quaternion* quaternionIn) const
{
    for (int i = kNearPlane; i < kINVALID; ++i)
    {
        Plane p = GetFrustumPlane((PlaneID)i);
        float3 mask = DirectX::XMVectorGreater(p.GetNormal(), DirectX::XMVectorZero());

        auto furthestCorner =  DirectX::XMVectorSelect(boxIn.pt_min, boxIn.pt_max, mask);

        // behind plane (frustum normals point in, then this is outside the frustum)
        if (p.DistanceFromPoint(furthestCorner) < 0.f)
            return false;
    }
    return true;
}
King::Frustum& King::Frustum::operator*=(const DirectX::XMMATRIX& m)
{
    //DirectX::XMMATRIX xForm = DirectX::XMMatrixTranspose(DirectX::XMMatrixInverse(nullptr, m));

    for (int i = 0; i < 8; ++i)
        _FrustumCorners[i] = DirectX::XMVector3Transform(_FrustumCorners[i], m);

    for (int i = 0; i < 6; ++i)
        _FrustumPlanes[i] = DirectX::XMVector4Transform(_FrustumPlanes[i], m);

    return *this;
}

DirectX::XMMATRIX King::Pyramid::MomentsOfInertia(const float& densityIn) 
{
    // Calculate the volume of the pyramid
    float volume = (1.0f / 3.0f) * baseLength * baseLength * height;
    float mass = densityIn * volume;

    // Calculate the moments of inertia about the centroid
    float Ixx = (mass * (baseLength * baseLength)) / 12.0f;  // About X-axis
    float Iyy = (mass * (baseLength * baseLength)) / 12.0f;  // About Y-axis
    float Izz = (mass * (height * height)) / 12.0f;          // About Z-axis

    // Since the pyramid is symmetric around the Y-axis, we will have:
    // Ixy = Ixz = Iyx = Iyz = Izx = Izy = 0

    // Inertia tensor (with respect to the centroid)
    DirectX::XMMATRIX I(
        Ixx, 0, 0, 0,
        0, Iyy, 0, 0,
        0, 0, Izz, 0,
        0, 0, 0, 1
    );

    // Adjust the moments of inertia to be about the origin
    // Using parallel axis theorem, the adjustments can be made based on the location of the centroid.
    // The centroid of the pyramid is at (0, height/4, 0) since the base is in the XZ plane.

    float d = height / 4.0f; // distance from the centroid to the base
    DirectX::XMVectorSetX(I.r[1], DirectX::XMVectorGetY(I.r[1]) + mass * d * d); // Adjust Iyy for Y-axis
    // Adjust Izz and Ixx if needed based on other axes

    return I;
}


bool King::Pyramid::Contains(const Point& ptIn) const
{
    // Get the local coordinates of the point relative to the pyramid's center
    float3 localPt = ptIn - GetCenter();

    // Check if the point is within the height of the pyramid
    if (localPt.GetY() < 0 || localPt.GetY() > height)
        return false;

    // Scale down the base size as the point moves up the height of the pyramid
    float baseScale = 1.0f - (localPt.GetY() / height);
    float halfBase = (baseLength * baseScale) * 0.5f;

    // Check if the point is within the base at the given height level
    return (localPt.GetX() >= -halfBase && localPt.GetX() <= halfBase && localPt.GetZ() >= -halfBase && localPt.GetZ() <= halfBase);
}

bool King::Pyramid::Intersects(const Ray& ray, float3* ptOut) const
{
    // Check intersection with each triangular face
    std::vector<Triangle> tri = GetFaces();
    for (const auto& face : tri) {
        if (face.Intersects(ray, ptOut)) {
            return true;
        }
    }
    return false;
}

bool King::Pyramid::Intersects(const Line& lineIn, float3* intersectionOut) const 
{
    // Check intersection with each triangular face
    std::vector<Triangle> faces = GetFaces();
    for (const auto& face : faces) {
        if (face.Intersects(lineIn, intersectionOut)) {
            return true;
        }
    }
    return false;
}

bool King::Pyramid::Intersects(const Triangle& rhs, float3* intersectionOut) const {
    // Check intersection with each triangular face
    std::vector<Triangle> faces = GetFaces(); // Define this method to get all triangular faces
    for (const auto& face : faces) {
        if (face.Intersects(rhs, intersectionOut)) {
            return true;
        }
    }
    return false;
}

bool King::Pyramid::Intersects(const Quad& rhs, float3* intersectionOut) const {
    // Check intersection with each triangular face
    std::vector<Triangle> faces = GetFaces(); // Define this method to get all triangular faces
    for (const auto& face : faces) {
        if (face.Intersects(rhs, intersectionOut)) {
            return true;
        }
    }
    return false;
}

bool King::Pyramid::Intersects(const Plane& rhs, float3* intersectionOut) const {
    // Check intersection with each triangular face
    std::vector<Triangle> faces = GetFaces(); // Define this method to get all triangular faces
    for (const auto& face : faces) {
        if (face.Intersects(rhs, intersectionOut)) {
            return true;
        }
    }
    return false;
}

bool King::Pyramid::Intersects(const Sphere& rhs, float3* intersectionOut) const {
    // Check intersection with each triangular face
    std::vector<Triangle> faces = GetFaces(); // Define this method to get all triangular faces
    for (const auto& face : faces) {
        if (face.Intersects(rhs, intersectionOut)) {
            return true;
        }
    }
    return false;
}

bool King::Pyramid::Intersects(const Box& boxIn, const Quaternion& orientationIn, float3* intersectionOut) const 
{
    DirectX::XMMATRIX boxToWorld = DirectX::XMMatrixRotationQuaternion(orientationIn);
    DirectX::XMMATRIX worldToBox = DirectX::XMMatrixInverse(nullptr, boxToWorld);

    // Transform pyramid center and base
    float3 pyramidCenter = GetCenter();
    pyramidCenter = DirectX::XMVector3Transform(pyramidCenter, worldToBox);

    std::vector<float3> pyramidBase = GetBaseVertices();
    for (auto& vertex : pyramidBase) {
        vertex = DirectX::XMVector3Transform(vertex, worldToBox);
    }

    // Transform apex of the pyramid
    float3 pyramidApex = float3(0.f, height, 0.f) + GetCenter();
    pyramidApex = DirectX::XMVector3Transform(pyramidApex, worldToBox);


    std::vector<Triangle> pyramidFaces = GetFaces();
    for (const auto& face : pyramidFaces) {
        if (boxIn.Intersects(face, intersectionOut)) {
            if (intersectionOut != nullptr) {
                *intersectionOut = DirectX::XMVector3Transform(*intersectionOut, boxToWorld);
            }
            return true;
        }
    }

    return false;
}


bool King::Pyramid::Intersects(const Pyramid& rhs, float3* intersectionOut) const 
{
    // Step 1: Get the base and face triangles of both pyramids
    std::vector<Triangle> thisFaces = GetFaces();
    std::vector<Triangle> rhsFaces = rhs.GetFaces();

    // Step 2: Check intersection between all face pairs of both pyramids
    for (const auto& thisFace : thisFaces) {
        for (const auto& rhsFace : rhsFaces) {
            if (thisFace.Intersects(rhsFace, intersectionOut)) {
                return true;
            }
        }
    }

    return false;
}

bool King::Pyramid::Intersects(const Capsule& capsuleIn, float3* intersectionOut) const {
    // Get the line segment representing the capsule
    Line capsuleLine = capsuleIn.GetSegment();
    float radius = capsuleIn.GetRadius();

    // Get the triangle faces of the pyramid
    std::vector<Triangle> faces = GetFaces();

    // Iterate through each triangle face
    for (const Triangle& face : faces) {
        float3 intersectionPoint;

        // Check for intersection with the line segment of the capsule
        if (face.Intersects(capsuleLine, &intersectionPoint)) {
            // Check if the intersection point is within the radius of the capsule
            float3 nearestPoint = capsuleLine.FindNearestPointOnLineSegment(intersectionPoint);
            if (Distance(intersectionPoint, nearestPoint) <= radius) {
                if (intersectionOut) {
                    *intersectionOut = intersectionPoint; // Output the intersection point
                }
                return true; // An intersection was found
            }
        }
    }

    return false; // No intersection found with any face
}

King::Quad King::Pyramid::GetBaseQuad() const 
{
    // Retrieve the base vertices using the GetBase() method
    std::vector<float3> baseVertices = GetBaseVertices();

    // Assuming baseVertices are in the order: Bottom Left, Bottom Right, Top Right, Top Left
    return Quad(baseVertices[0], baseVertices[1], baseVertices[2], baseVertices[3]);
}

std::vector<Triangle> King::Pyramid::GetFaces() const {
    std::vector<Triangle> faces;

    // Get base vertices
    std::vector<float3> baseVertices = GetBaseVertices();

    // Calculate the apex position
    float3 apex = float3(0.f, height, 0.f) + GetCenter();

    // Assuming the base vertices are ordered in a counter-clockwise manner
    // Create triangles for each side face
    faces.emplace_back(apex, baseVertices[0], baseVertices[1]); // Face 1: Apex, Bottom Left, Bottom Right
    faces.emplace_back(apex, baseVertices[1], baseVertices[2]); // Face 2: Apex, Bottom Right, Top Right
    faces.emplace_back(apex, baseVertices[2], baseVertices[3]); // Face 3: Apex, Top Right, Top Left
    faces.emplace_back(apex, baseVertices[3], baseVertices[0]); // Face 4: Apex, Top Left, Bottom Left
    // and the base
    faces.emplace_back(baseVertices[0], baseVertices[1], baseVertices[2]);
    faces.emplace_back(baseVertices[0], baseVertices[2], baseVertices[3]);

    return faces;
}

std::vector<float3> King::Pyramid::GetBaseVertices() const
{
    std::vector<float3> baseVertices;
    float halfBase = baseLength * 0.5f;

    const float3 c(GetCenter());
    const float x = c.GetX();
    const float y = c.GetY();
    const float z = c.GetZ();

    // Calculate the base vertices assuming the base is aligned with the XZ plane
    baseVertices.push_back(float3(x - halfBase, y, z - halfBase)); // Bottom-left
    baseVertices.push_back(float3(x + halfBase, y, z - halfBase)); // Bottom-right
    baseVertices.push_back(float3(x + halfBase, y, z + halfBase)); // Top-right
    baseVertices.push_back(float3(x - halfBase, y, z + halfBase)); // Top-left

    return baseVertices; // Return the vector of base vertices
}

float3 King::Pyramid::FindNearestPoint(const float3& point) const 
{
    float minDistanceSquared = FLT_MAX;
    float3 closestPoint(FLT_MAX);

    std::vector<Triangle> faces = GetFaces();

    // Check each face for the nearest point
    for (const auto& face : faces) 
    {
        float3 nearestPointOnFace = face.FindNearestPoint(point);

        // Calculate distance to this point
        float distanceSquared = DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(point - nearestPointOnFace));

        // Update closest point if this point is closer
        if (distanceSquared < minDistanceSquared) {
            minDistanceSquared = distanceSquared;
            closestPoint = nearestPointOnFace;
        }
    }

    return closestPoint;
}


// Method to calculate the distance from a point to the pyramid
float King::Pyramid::DistanceFromPoint(const float3& point) const 
{
    // Calculate the nearest point on the pyramid's base
    float3 nearestPointOnBase = FindNearestPoint(point);

    // Calculate the distance from the nearest point on the base to the given point
    return (point - nearestPointOnBase).GetMagnitudeEst();
}

// double dispatch

//inline bool King::Pyramid::Collision(Point const& pointIn) const { return Contains(pointIn); }
//
//inline bool King::Pyramid::Collision(Ray const& rayIn) const { float3 out; return Intersects(rayIn, &out); }
//
//inline bool King::Pyramid::Collision(Line const& lineIn) const { float3 out; return Intersects(lineIn, &out); }
//
//inline bool King::Pyramid::Collision(Plane const& planeIn) const { float3 out; return Intersects(planeIn, &out); }
//
//inline bool King::Pyramid::Collision(Sphere const& sphereIn) const { float3 out; return Intersects(sphereIn, &out); }
//
//inline bool King::Pyramid::Collision(Pyramid const& pyramidIn) const { float3 out; return Intersects(pyramidIn, &out); }
//
//inline bool King::Pyramid::Collision(Capsule const& capsuleIn) const { float3 out; return Intersects(capsuleIn, &out); }
//
//inline bool King::Pyramid::Collision(Box const& boxIn) const { float3 out; return Intersects(boxIn, Quaternion(), &out); }

inline bool __vectorcall King::Line::Intersects(const Triangle& triangle, float3* intersectionOut) const
{
    // Get the vertices of the triangle
    const float3& v0 = triangle.GetVertex(0);

    // Direction of the line (pt[1] - pt[0])
    float3 dir = pt[1] - pt[0];

    // Edge vectors of the triangle
    float3 edge1 = triangle.GetEdge1();
    float3 edge2 = triangle.GetEdge2();

    // Calculate determinant (pVec = direction cross edge2)
    float3 pVec = Cross(dir, edge2);
    float det = Dot(edge1, pVec);

    // If the determinant is close to 0, the line is parallel to the triangle plane
    if (fabs(det) < 1e-8f) {
        return false; // Line does not intersect the triangle
    }

    // Inverse determinant
    float invDet = 1.0f / det;

    // Calculate distance from v0 to the line's start point (tVec = pt[0] - v0)
    float3 tVec = pt[0] - v0;

    // Calculate u parameter and test bounds
    float u = Dot(tVec, pVec) * invDet;
    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    // Calculate qVec = tVec cross edge1
    float3 qVec = Cross(tVec, edge1);

    // Calculate v parameter and test bounds
    float v = Dot(dir, qVec) * invDet;
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }

    // Calculate t (intersection point)
    float t = Dot(edge2, qVec) * invDet;

    // If t is within the line segment, we have an intersection
    if (t >= 0.0f && t <= 1.0f) {
        if (intersectionOut != nullptr) {
            *intersectionOut = pt[0] + t * dir; // Calculate the intersection point
        }
        return true; // Intersection found
    }

    // No intersection
    return false;
}

inline bool __vectorcall Line::Intersects(const Quad& quadIn, float3* intersectionOut) const
{
    // Extract the two triangles that form the quad
    Triangle tri1 = quadIn.GetTriangle1();
    Triangle tri2 = quadIn.GetTriangle2();

    // Check intersection with the first triangle
    if (this->Intersects(tri1, intersectionOut)) {
        return true; // If an intersection occurs with the first triangle, return true
    }

    // Check intersection with the second triangle
    if (this->Intersects(tri2, intersectionOut)) {
        return true; // If an intersection occurs with the second triangle, return true
    }

    // No intersection with either triangle
    return false;
}


/******************************************************************************
*   Class Contact
******************************************************************************/
// Streams
// json
// operators
// methods
//bool King::Collided::GetHasPenetration() const
//{
//    const auto& mag = _Obj2_to_Obj1.Get_magnitude();
//    const float epsilon = 5.e-5f;
//
//    return (mag > epsilon);
//}

//void King::Contact::SetContactLocalObjectSpace(const Position& contactObj1LocalIn, const Position& contactObj2LocalIn, const Distance& worldSpaceDistanceObj1ToObj2)
//{
//    const DirectX::XMVECTOR epsilon = DirectX::XMVectorReplicate(5.e-5f);
//    const DirectX::XMVECTOR & v1 = static_cast<DirectX::XMVECTOR>(_localContactObj1);
//    const DirectX::XMVECTOR & v2 = static_cast<DirectX::XMVECTOR>(_localContactObj2);
//
//    if (DirectX::XMVector3NearEqual(static_cast<DirectX::XMVECTOR>(contactObj1LocalIn), v1, epsilon))
//    {
//        if (DirectX::XMVector3NearEqual(static_cast<DirectX::XMVECTOR>(contactObj2LocalIn), v2, epsilon))
//        {
//            _staticContact = true;
//            return;
//        }
//    }
//    _staticContact = false;
//
//    _inContact = true;
//    _localContactObj1 = contactObj1LocalIn;
//    _localContactObj2 = contactObj2LocalIn;
//    _directionContactObj1_to_ContactObj2 = worldSpaceDistanceObj1ToObj2; // Distance(float3(contactObj2In - contactObj1In));
//}

// Separating axis theorem of orientated boxes in 3D Space
//
// Based on the paper Separating Axis Theorem for Oriented Bounding Boxes, Johnny Huynh, 2009, www.jkh.me
// https://www.jkh.me/files/tutorials/Separating%20Axis%20Theorem%20for%20Oriented%20Bounding%20Boxes.pdf
// Which expands the equations into the 15 cases set forth in the original separating axis theorem paper 
// by S. Gottschalk, M. C. Lin, D. Manocha, University of North Carolina, 1996.
// http://gamma.cs.unc.edu/SSV/obb.pdf.  
// (c) Christopher H. King 2020, All rights reserved

std::vector<Collided> King::SAT_OBBonOBB(const Box& A, const King::Quaternion& qA, const Box& B, const King::Quaternion& qB)
{
    // Output of contacts, each with a vector of contact points in world coordinates
    vector<Collided> contacts;
    vector<Collided> rtn;

    // A and B are oriented bounding boxes(OBB) with 8 corners each
    vector<float3> cA = A.GetCorners8(&qA);
    vector<float3> cB = B.GetCorners8(&qB);
    // Local axis A in world space
    float3 PA = A.GetCenter();
    float3 Ax = cA[Box::CornerDescription::rbf] - cA[Box::CornerDescription::lbf];
    float3 Ay = cA[Box::CornerDescription::ltf] - cA[Box::CornerDescription::lbf];
    float3 Az = cA[Box::CornerDescription::lbf] - cA[Box::CornerDescription::lbb];
    Ax.MakeNormalize();
    Ay.MakeNormalize();
    Az.MakeNormalize();
    float3 whdA(A.GetExtents());

    // Local axis B in world space
    float3 PB = B.GetCenter();
    float3 Bx = cB[Box::CornerDescription::rbf] - cB[Box::CornerDescription::lbf];
    float3 By = cB[Box::CornerDescription::ltf] - cB[Box::CornerDescription::lbf];
    float3 Bz = cB[Box::CornerDescription::lbf] - cB[Box::CornerDescription::lbb];
    Bx.MakeNormalize();
    By.MakeNormalize();
    Bz.MakeNormalize();
    float3 whdB(B.GetExtents());

    // Translation vector from A to B
    auto T = PB - PA;
    float tDotAx, tDotAy, tDotAz;
    float3 L; // axis
    /*
    If at least one of these 15 inequalities is true, then there exists a separating axis. If a
    separating axis exists, then the two OBBs (A and B) do not intersect, and there is no need to
    further check the other cases.
*/
    float WA = whdA.GetX();
    float WB = whdB.GetX();
    float HB = whdB.GetY();
    float DB = whdB.GetZ();

    auto Rxx = Dot(Ax, Bx);
    auto Rxy = Dot(Ax, By);
    auto Rxz = Dot(Ax, Bz);

    if (tDotAx = Dot(T, Ax) > WA + abs(WB * Rxx) + abs(HB * Rxy) + abs(DB * Rxz))
        return rtn;
    else
    {
        L = Ax;
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    float HA = whdA.GetY();
    auto Ryx = Dot(Ay, Bx);
    auto Ryy = Dot(Ay, By);
    auto Ryz = Dot(Ay, Bz);

    if (tDotAy = Dot(T, Ay) > HA + abs(WB * Ryx) + abs(HB * Ryy) + abs(DB * Ryz))
        return rtn;
    else
    {
        L = Ay;
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    float DA = whdA.GetZ();
    auto Rzx = Dot(Az, Bx);
    auto Rzy = Dot(Az, By);
    auto Rzz = Dot(Az, Bz);

    if (tDotAz = Dot(T, Az) > DA + abs(WB * Rzx) + abs(HB * Rzy) + abs(DB * Rzz))
        return rtn;
    else
    {
        L = Az;
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (Dot(T, Bx) > WB + abs(WA * Rxx) + abs(HA * Ryx) + abs(DA * Rzx))
        return rtn;
    else
    {
        L = Bx;
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (Dot(T, By) > HB + abs(WA * Rxy) + abs(HA * Ryy) + abs(DA * Rzy))
        return rtn;
    else
    {
        L = By;
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (Dot(T, Bz) > DB + abs(WA * Rxz) + abs(HA * Ryz) + abs(DA * Rzz))
        return rtn;
    else
    {
        L = Bz;
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAz * Ryx - tDotAy * Rzx) > abs(HA * Rzx) + abs(DA * Ryx) + abs(HB * Rxz) + abs(DB * Rxy))
        return rtn;
    else
    {
        L = Cross(Ax, Bx);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAz * Ryy - tDotAy * Rzy) > abs(HA * Rzy) + abs(DA * Ryy) + abs(WB * Rxz) + abs(DB * Rxx))
        return rtn;
    else
    {
        L = Cross(Ax, By);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAz * Ryz - tDotAy * Rzz) > abs(HA * Rzz) + abs(DA * Ryz) + abs(WB * Rxy) + abs(HB * Rxx))
        return rtn;
    else
    {
        L = Cross(Ax, Bz);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAx * Rzx - tDotAz * Rxx) > abs(WA * Rzx) + abs(DA * Rxx) + abs(HB * Ryz) + abs(DB * Ryy))
        return rtn;
    else
    {
        L = Cross(Ay, Bx);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAx * Rzy - tDotAz * Rxy) > abs(WA * Rzy) + abs(DA * Rxy) + abs(WB * Ryz) + abs(DB * Ryx))
        return rtn;
    else
    {
        L = Cross(Ay, By);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAx * Rzz - tDotAz * Rxz) > abs(WA * Rzz) + abs(DA * Rxz) + abs(WB * Ryy) + abs(HB * Ryx))
        return rtn;
    else
    {
        L = Cross(Ay, Bz);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAy * Rxx - tDotAx * Ryx) > abs(WA * Ryx) + abs(HA * Rxx) + abs(HB * Rzz) + abs(DB * Rzy))
        return rtn;
    else
    {
        L = Cross(Az, Bx);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAy * Rxy - tDotAx * Ryy) > abs(WA * Ryy) + abs(HA * Rxy) + abs(WB * Rzz) + abs(DB * Rzx))
        return rtn;
    else
    {
        L = Cross(Az, By);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }
    if (abs(tDotAy * Rxz - tDotAx * Ryz) > abs(WA * Ryz) + abs(HA * Rxz) + abs(WB * Rzy) + abs(HB * Rzx))
        return rtn;
    else
    {
        L = Cross(Az, Bz);
        auto c = SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(cA, cB, L);
        contacts.push_back(c);
    }

    // if we made it here then there were no separating axis
    for (auto& ea : contacts)
    {
        if (ea.OBBonOBBIntersection(A, qA, B, qB, cA, cB))
            rtn.push_back(ea);
    }


    return rtn;
}
Collided King::SAT_ContactDepthAndDirectionFromOBBonOBBIntersection(const vector<float3>& cornersA, const vector<float3>& cornersB, const float3& nonSeparatingAxis)
{
    Collided contact;
    // *** TO DO *** commented out on 7/15/2023; code is fine, just need to consider contact.SetPenetration which was moved into each contact point struct
    //if (float3::Magnitude(nonSeparatingAxis) < 0.02f)
    //{
    //    // cross products of parallel axis is zero (and near zero) process as contact
    //    contact.SetPenetration(Distance(10000.f, nonSeparatingAxis));
    //    return contact;
    //}

    //// project each corner onto the axis for each box
    //float minA, maxA, dist;
    //minA = maxA = Dot(cornersA.front(), nonSeparatingAxis);
    //for (const auto& ea : cornersA)
    //{
    //    dist = Dot(ea, nonSeparatingAxis);
    //    if (dist < minA) minA = dist;
    //    else maxA = max(maxA, dist);
    //}
    //float distA = maxA - minA;

    //// project each corner onto the axis for each box
    //float minB, maxB;
    //minB = maxB = Dot(cornersB.front(), nonSeparatingAxis);
    //for (const auto& ea : cornersB)
    //{
    //    dist = Dot(ea, nonSeparatingAxis);
    //    if (dist < minB) minB = dist;
    //    else maxB = max(maxB, dist);
    //}
    //float distB = maxB - minB;

    //if (minB > maxA || minA > maxB)
    //{
    //    assert(0);
    //    // separated, this was a false contact call
    //    return contact;
    //}
    //else
    //{
    //    // contact
    //    float minC = min(minA, minB);
    //    float maxC = max(maxA, maxB);
    //    float distC = maxC - minC;
    //    auto axis = nonSeparatingAxis;

    //    float penetration = distA + distB - distC;

    //    // BoxA pushes BoxB away
    //    // world space distance from objA to objB
    //    // minA < minB, if not reverse the axis
    //    if (minA > minB)
    //    {
    //        axis *= -1.f;
    //    }

    //    contact.SetPenetration(Distance(penetration, axis));
    //}
    return contact; // compiler to optimize as && from temporary
}

bool King::Collided::SphereOnSphereIntersection(const Sphere& A, const Sphere& B)
{
    // check for collision
    contactPoints.clear();

    const float radiusSum = A.GetRadius() + B.GetRadius();
    const auto AtoB = B.GetCenter() - A.GetCenter();
    const float distance = AtoB.GetMagnitudeEst();   

    if (distance < radiusSum) {
        normal = float3::Normal(AtoB);
        penetrationDepth = radiusSum - distance;

        // Calculate contact points on the surface of each collider
        DirectX::XMVECTOR contactPoint1 = DirectX::XMVectorMultiplyAdd(normal, DirectX::XMVectorReplicate(A.GetRadius()), A.GetCenter());
        DirectX::XMVECTOR contactPoint2 = DirectX::XMVectorMultiplyAdd(DirectX::XMVectorNegate(normal), DirectX::XMVectorReplicate(B.GetRadius()), B.GetCenter());

        contactPoints.push_back(contactPoint1);
        contactPoints.push_back(contactPoint2);
        return true;
    }

    return false;
}

bool King::Collided::OBBonOBBIntersection(const Box& A, const Quaternion qA, const Box& B, const Quaternion qB, const vector<float3>& cornersA, const vector<float3>& cornersB)
{
    //// *** TO DO *** pass in penetration normal and magnitude as a Distince ?? ***
    // *** TO DO *** refactor below for _contacts was just a point and now is struct Contact that include all information (ex: penetration normal)
    //// penetration must already be defined
    ////auto& p = GetPenetration();
    ////auto& d = p.Get_magnitude();

    //_contacts.clear();
    ////SetObjects(A.GetCenter(), B.GetCenter());

    //auto contactVertsA = GetContactPoints(cornersA, p);
    //auto contactVertsB = GetContactPoints(cornersB, -p);

    ////cout << "    Contact VertsA: " << contactVertsA.size() << "\n";
    ////for (auto& ea : contactVertsA)
    ////    cout << "        " << ea << "\n";
    ////cout << "    Contact VertsB: " << contactVertsB.size() << "\n";
    ////for (auto& ea : contactVertsB)
    ////    cout << "        " << ea << "\n";

    //if (!contactVertsA.size() || !contactVertsB.size())
    //    return false; // this should not really happen unless p or corners have an issue

    //// if we have more than four both boxes, clip them to face contacts only
    //if (contactVertsA.size() > 4 && contactVertsB.size() > 4)
    //{
    //    _contacts = ClosestPointsToFaces(&contactVertsA, &contactVertsB);
    //}

    //if (contactVertsA.size() == 4)
    //{
    //    _contacts = contactVertsA;
    //}

    //if (contactVertsB.size() == 4)
    //{
    //    for (auto& ea : contactVertsB)
    //        _contacts.push_back(ea);
    //}

    //if (contactVertsB.size() < contactVertsA.size())
    //{
    //    _contacts = contactVertsB;
    //    _Obj2_to_Obj1 = -_Obj2_to_Obj1;
    //}

    //if (contactVertsA.size() == 2 && contactVertsB.size() == 2)
    //{
    //    _contacts.push_back(ClosestPointEdgeEdge(contactVertsA, contactVertsB));
    //}

    //if (contactVertsA.size() == 2 && contactVertsB.size() >= 3)
    //{
    //    XMMATRIX rB = qB;
    //    _contacts.push_back(B.FindNearestPointOnBox(contactVertsA[0], rB));
    //    _contacts.push_back(B.FindNearestPointOnBox(contactVertsA[1], rB));
    //}

    //if (contactVertsA.size() >= 3 && contactVertsB.size() == 2)
    //{
    //    XMMATRIX rA = qA;
    //    _contacts.push_back(A.FindNearestPointOnBox(contactVertsB[0], rA));
    //    _contacts.push_back(A.FindNearestPointOnBox(contactVertsB[1], rA));
    //}

    //// Check for duplicates
    //for (unsigned int i=0; i < _contacts.size();++i)
    //{
    //    for (unsigned int j = i+1; j < _contacts.size(); ++j)
    //    {
    //        if (_contacts[i] == _contacts[j])
    //        {
    //            // duplicate so remove
    //            for (unsigned int k = j; k < _contacts.size() - 1; ++k)
    //                _contacts[k] = _contacts[k + 1];
    //            _contacts.pop_back();
    //        }
    //    }
    //}

    ////cout << "    Clipped Verts: " << _contactVerts.size() << "\n";
    ////for (auto& ea : _contactVerts)
    ////    cout << "        " << ea << "\n";

    if (contactPoints.size())
        return true;
    else
        return false;
}

inline vector<float3> King::Collided::GetContactPoints(const vector<float3>& corners, const Distance& dist)
{
    // find the corners that are in contact
    const auto& penetration = dist.Get_magnitude();
    const auto& N = dist.Get_unit_direction();
    auto planePoint = corners[0];
    float maxdist = Dot(planePoint, N);

    for (const auto& ea : corners)
    {
        float d = Dot(ea, N);
        if (d > maxdist)
        {
            maxdist = d;
            planePoint = ea;
        }
    }

    // Plane Equation (A dot N) - d = 0;
    float d = Dot(planePoint, N);
    d -= penetration + 0.001f;
    vector<float3> contactVerts;

    for (const auto& ea : corners)
    {
        float side = Dot(ea, N) - d;

        if (side > 0.f)
        {
            contactVerts.push_back(ea);
        }
    }

    return contactVerts;
}

inline vector<float3> King::Collided::ClosestPointsToFaces(vector<float3>* contactVertsA, vector<float3>* contactVertsB)
{
    // *** TO DO *** we might have to sort them to get CW ordering correct
    //SortVertices(verts0, vertIndexs0);
    //SortVertices(verts1, vertIndexs1);

    Plane pA(contactVertsA->at(0), contactVertsA->at(1), contactVertsA->at(2));

    vector<float3> vertsTemp1;
    for (const auto& ea : *contactVertsB)
    {
        vertsTemp1.push_back(ea + (pA.GetNormal() * Dot(pA.GetNormal(), contactVertsA->at(0) - ea)));
    }

    vector<float3> temp;

    for (int c = 0; c < 2; c++)
    {
        vector<float3>* vertA = &vertsTemp1;
        vector<float3>* vertB = contactVertsA;

        // 2nd time swap
        if (c == 1)
        {
            vertA = contactVertsA;
            vertB = &vertsTemp1;
        }

        // Work out the normal for the face
        Plane face(vertA->at(0), vertA->at(1), vertA->at(2));
        auto n = face.GetNormal();

        for (int i = 0; i < 4; i++)
        {
            float3 s0 = vertA->at(i);
            float3 s1 = vertA->at((i + 1) % 4);
            float3 sx = s0 + n * 10.0f;

            // Work out the normal for the face
            float3 sv0 = s1 - s0;
            float3 sv1 = sx - s0;
            float3 sn = Cross(sv1, sv0);
            sn = Normalize(sn);

            float d = Dot(s0, sn);

            for (int j = 0; j < 4; j++)
            {
                float3 p0 = vertB->at(j);
                float3 p1 = vertB->at((j + 1) % 4); // Loops back to the 0th for the last one

                float d0 = Dot(p0, sn) - d;
                float d1 = Dot(p1, sn) - d;

                // Check there on opposite sides of the plane
                if ((d0 * d1) < 0.0f)
                {
                    float3 pX = p0 + (Dot(sn, (s0 - p0)) / Dot(sn, (p1 - p0))) * (p1 - p0);

                    if (VertInsideFace(*vertA, pX, 0.01f))
                    {
                        temp.push_back(pX);
                    }

                }

                if (VertInsideFace(*vertA, p0))
                {
                    temp.push_back(p0);
                }
            }
        }
    }

    // Remove verts which are very close to each other

    auto n(temp.size());
    for (size_t i = 0; i < n; i++)
    {
        for (size_t j = i; j < n; j++)
        {
            if (i != j)
            {
                auto t = temp[i] - temp[j];
                float dist = Dot(t, t);

                if (dist < 0.05f)
                {

                    for (size_t k = j; k < n-1; k++)
                    {
                        temp[k] = temp[k + 1];
                    }
                    temp.pop_back();
                    --n;
                }
            }
        }
    }

    return temp;
}
/******************************************************************************
*    ClosestPointEdgeEdge
*        Input four points of contacts (an edge of object A and an edge of obect B) 
*        and returns the average point between the contacts to collapse the edges
*        of contact down to one point.
******************************************************************************/
inline float3 King::Collided::ClosestPointEdgeEdge(const vector<float3>& contactVertsA, const vector<float3>& contactVertsB)
{
    assert(contactVertsA.size() > 1 && contactVertsB.size() > 1);

    /*        lA
        A[0]------A[1]
             \  /lBA   
              /\ 
            /    \lAB 
        B[0]------B[1]
              lB
    */
    Line lA(contactVertsA[0], contactVertsA[1]);
    Line lB(contactVertsB[0], contactVertsB[1]);
    Line lAB(contactVertsA[0], contactVertsB[1]);
    Line lBA(contactVertsB[0], contactVertsA[1]);

    auto pt1 = lA.FindNearestPointOnLineSegment(lBA.GetCenter());
    auto pt2 = lB.FindNearestPointOnLineSegment(lAB.GetCenter());

    return (pt1 + pt2) * 0.5f;
}

inline bool King::Collided::VertInsideFace(const vector<float3>& contactPoints, const float3& p0, const float& planeErr)
{
    // Work out the normal for the face
    float3 v0 = contactPoints[1] - contactPoints[0];
    float3 v1 = contactPoints[2] - contactPoints[0];
    float3 n = Cross(v1, v0);
    n = Normalize(n);

    for (size_t i = 0; i < contactPoints.size(); i++)
    {
        float3 s0 = contactPoints[i];
        float3 s1 = contactPoints[(i + 1) % 4];
        float3 sx = s0 + n * 10.0f;

        // Work out the normal for the face
        float3 sv0 = s1 - s0;
        float3 sv1 = sx - s0;
        float3 sn = Cross(sv1, sv0);
        sn = Normalize(sn);

        float d = Dot(s0, sn);
        float d0 = Dot(p0, sn) - d;

        // Outside the plane
        if (d0 > planeErr)
        {
            return false;
        }
    }

    return true;
}
// Functionality
/* To test collision against a capsule, you must find the closest point to it's line segment to whatever it is you are testing, then construct a spehre at that point. Then it becomes a whatever / sphere test.*/

inline bool King::Capsule::Intersects(const Point& ptIn) const { auto c = segment.FindNearestPointOnLineSegment(ptIn); return Sphere(c, radius).Contains(ptIn); }

bool King::Capsule::Intersects(const Ray& rayIn, Ray *intersectAndNormal1Out, Ray* intersectAndNormal2Out) const
{
    // Credits to https://gist.github.com/jdryg/ecde24d34aa0ce2d4d87 for method

    // Substituting equ. (1) - (6) to equ. (I) and solving for t' gives:
    //
    // t' = (t * dot(AB, d) + dot(AB, AO)) / dot(AB, AB); (7) or
    // t' = t * m + n where 
    // m = dot(AB, d) / dot(AB, AB) and 
    // n = dot(AB, AO) / dot(AB, AB)
    //
    const Line& capSeg = GetSegment();
    const float& capRadius = GetRadius();

    float3 AB = capSeg.GetVertex(1) - capSeg.GetVertex(0);
    float3 AO = rayIn.GetOrigin() - capSeg.GetVertex(0);

    float AB_dot_d = AB.DotProduct(rayIn.GetDirection());
    float AB_dot_AO = AB.DotProduct(AO);
    float AB_dot_AB = AB.DotProduct(AB);

    float m = AB_dot_d / AB_dot_AB;
    float n = AB_dot_AO / AB_dot_AB;

    // Substituting (7) into (II) and solving for t gives:
    //
    // dot(Q, Q)*t^2 + 2*dot(Q, R)*t + (dot(R, R) - r^2) = 0
    // where
    // Q = d - AB * m
    // R = AO - AB * n
    float3 Q = rayIn.GetDirection() - (AB * m);
    float3 R = AO - (AB * n);

    float a = Q.DotProduct(Q);
    float b = 2.0f * Q.DotProduct(R);
    float c = R.DotProduct(R) - (capRadius * capRadius);

    if (a == 0.0f)
    {
        // Special case: AB and ray direction are parallel. If there is an intersection it will be on the end spheres...
        // because the ray is down the local axis of the capsule
        // Q = d - AB * m =>
        // Q = d - AB * (|AB|*|d|*cos(AB,d) / |AB|^2) => |d| == 1.0
        // Q = d - AB * (|AB|*cos(AB,d)/|AB|^2) =>
        // Q = d - AB * cos(AB, d) / |AB| =>
        // Q = d - unit(AB) * cos(AB, d)
        //
        // |Q| == 0 means Q = (0, 0, 0) or d = unit(AB) * cos(AB,d)
        // both d and unit(AB) are unit vectors, so cos(AB, d) = 1 => AB and d are parallel.
        // 
        Sphere sphereA, sphereB;
        sphereA.SetCenter(capSeg.GetVertex(0));
        sphereA.SetRadius(radius);
        sphereB.SetCenter(capSeg.GetVertex(1));
        sphereB.SetRadius(radius);

        float3 pt;
        if (!sphereA.Intersects(rayIn, &pt) || !sphereB.Intersects(rayIn, &pt))
        {
            // No intersection with one of the spheres means no intersection at all...
            return false;
        }

        float atmin=0.f, atmax = 0.f, btmin = 0.f, btmax = 0.f;
        {
            float3 CO_A = rayIn.GetOrigin() - sphereA.GetCenter();
            float a_A = Dot(rayIn.GetDirection(), rayIn.GetDirection());
            float b_A = 2.0f * Dot(CO_A, rayIn.GetDirection());
            float c_A = Dot(CO_A, CO_A) - (sphereA.GetRadius() * sphereA.GetRadius());
            float discriminant_A = b_A * b_A - 4.0f * a_A * c_A;
            if (discriminant_A >= 0.0f)
            {
                atmin = (-b - sqrtf(discriminant_A)) / (2.0f * a);
                atmax = (-b + sqrtf(discriminant_A)) / (2.0f * a);
            }
            if (atmin > atmax)
                std::swap(atmin, atmax);
        }
        {
            float3 CO_B = rayIn.GetOrigin() - sphereB.GetCenter();
            float a_B = Dot(rayIn.GetDirection(), rayIn.GetDirection());
            float b_B = 2.0f * Dot(CO_B, rayIn.GetDirection());
            float c_B = Dot(CO_B, CO_B) - (sphereB.GetRadius() * sphereB.GetRadius());
            float discriminant_B = b_B * b_B - 4.0f * a_B * c_B;
            if (discriminant_B >= 0.0f)
            {
                btmin = (-b - sqrtf(discriminant_B)) / (2.0f * a);
                btmax = (-b + sqrtf(discriminant_B)) / (2.0f * a);
            }
            if (btmin > btmax)
                std::swap(btmin, btmax);
        }
        if (atmin < btmin)
        {
            intersectAndNormal1Out->SetOrigin(rayIn.GetOrigin() + (rayIn.GetDirection() * atmin));
            intersectAndNormal1Out->SetDirection(intersectAndNormal1Out->GetOrigin() - capSeg.GetVertex(0));
        }
        else
        {
            intersectAndNormal1Out->SetOrigin(rayIn.GetOrigin() + (rayIn.GetDirection() * btmin));
            intersectAndNormal1Out->SetDirection(intersectAndNormal1Out->GetOrigin() - capSeg.GetVertex(1));
        }

        if (atmax > btmax)
        {
            intersectAndNormal2Out->SetOrigin(rayIn.GetOrigin() + (rayIn.GetDirection() * atmax));
            intersectAndNormal2Out->SetDirection(intersectAndNormal2Out->GetOrigin() - capSeg.GetVertex(0));
        }
        else
        {
            intersectAndNormal2Out->SetOrigin(rayIn.GetOrigin() + (rayIn.GetDirection() * btmax));
            intersectAndNormal2Out->SetDirection(intersectAndNormal2Out->GetOrigin() - capSeg.GetVertex(1));
        }

        return true;
    }

    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f)
    {
        // The ray doesn't hit the infinite cylinder defined by (A, B).
        // No intersection.
        return false;
    }

    float tmin = (-b - sqrtf(discriminant)) / (2.0f * a);
    float tmax = (-b + sqrtf(discriminant)) / (2.0f * a);
    if (tmin > tmax)
    {
        std::swap(tmin, tmax);
    }

    // Now check to see if K1 and K2 are inside the line segment defined by A,B
    float t_k1 = tmin * m + n;
    if (t_k1 < 0.0f)
    {
        // On sphere (A, r)...
        Sphere s(capSeg.GetVertex(0), capRadius);

        float3 pt;
        if (s.Intersects(rayIn, &pt))
        {
            intersectAndNormal1Out->SetOrigin(pt);
            intersectAndNormal1Out->SetDirection(pt - s.GetCenter());
        }
        else
            return false;
    }
    else if (t_k1 > 1.0f)
    {
        // On sphere (B, r)...
        Sphere s(capSeg.GetVertex(1), capRadius);

        float3 pt;
        if (s.Intersects(rayIn, &pt))
        {
            intersectAndNormal1Out->SetOrigin(pt);
            intersectAndNormal1Out->SetDirection(pt - s.GetCenter());
        }
        else
            return false;
    }
    else
    {
        // On the cylinder...
        intersectAndNormal1Out->SetOrigin( rayIn.GetOrigin() + (rayIn.GetDirection() * tmin));
        float3 k1 = capSeg.GetVertex(0) + AB * t_k1;
        intersectAndNormal1Out->SetDirection(intersectAndNormal1Out->GetOrigin() - k1);
    }

    float t_k2 = tmax * m + n;
    if (t_k2 < 0.0f)
    {
        // On sphere (A, r)...
        Sphere s(capSeg.GetVertex(0), capRadius);

        float3 pt;
        if (s.Intersects(rayIn, &pt))
        {
            intersectAndNormal2Out->SetOrigin(pt);
            intersectAndNormal2Out->SetDirection(pt - s.GetCenter());
        }
        else
            return false;
    }
    else if (t_k2 > 1.0f)
    {
        // On sphere (B, r)...
        Sphere s(capSeg.GetVertex(1), capRadius);

        float3 pt;
        if (s.Intersects(rayIn, &pt))
        {
            intersectAndNormal2Out->SetOrigin(pt);
            intersectAndNormal2Out->SetDirection(pt - s.GetCenter());
        }
        else
            return false;
    }
    else
    {
        // On the cylinder...
        intersectAndNormal2Out->SetOrigin(rayIn.GetOrigin() + (rayIn.GetDirection() * tmax));
        float3 k2 = capSeg.GetVertex(0) + AB * t_k2;
        intersectAndNormal2Out->SetDirection(intersectAndNormal2Out->GetOrigin() - k2);
    }

    return true;
}

bool King::Capsule::Intersects(const Line& lineIn) const
{
    // find the nearest point on the capsule line segment and the line
    // it then become a sphere point test
    const float epsilon_squared = FLT_EPSILON * FLT_EPSILON;
    float3 rayA = segment.pt[1] - segment.pt[0];
    float3 rayB = lineIn.pt[1] - lineIn.pt[0];

    float3 pointOnSegA, pointOnSegB;

    // line segment is degenerate (distance between end points is near zero)
    // this is the same as Dot(rayA,rayA) and Dot(rayB,rayB)
    float L11 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayA, float3(2.f, 2.f, 2.f))));
    float L22 = DirectX::XMVectorGetX(DirectX::XMVectorSum(DirectX::XMVectorPow(rayB, float3(2.f, 2.f, 2.f))));

    if (L11 < epsilon_squared)
    {
        pointOnSegB = lineIn.FindNearestPointOnLineSegment(segment.pt[0]);
    }
    else if (L22 < epsilon_squared)
    {
        pointOnSegA = segment.FindNearestPointOnLineSegment(lineIn.pt[0]);
    }
    else
    {
        float3 AB = lineIn.pt[0] - segment.pt[0];
        // - dot product of A and B
        float L12 = -float3::SumComponents(rayA * rayB);
        float DetL = L11 * L22 - L12 * L12;

        // Lines/Segments A and B are parallel
        if (fabsf(DetL) < FLT_EPSILON)
        {
            return false;
        }
        // The general case
        else
        {
            float ra = float3::SumComponents(rayA * AB);
            float rb = -float3::SumComponents(rayB * AB);

            float t = (L11 * rb - ra * L12) / DetL;
            float s = (ra - L12 * t) / L11;

            pointOnSegA = rayA * s + segment.pt[0];
            pointOnSegB = rayB * t + lineIn.pt[0];
        }

    }
    Sphere s(pointOnSegA,GetRadius());
    return s.Contains(pointOnSegB);
}

inline bool King::Capsule::Intersects(const Sphere& sphereIn) const 
{ 
    auto c = segment.FindNearestPointOnLineSegment(sphereIn.GetCenter()); 
    return Sphere(c, radius).Intersects(sphereIn); 
}

bool King::Capsule::Intersects(const Box& boxIn) const
{
    const float3& c = boxIn.GetCenter();
    auto s_center = segment.FindNearestPointOnLineSegment(c);
    auto b_nearest = boxIn.FindNearestPointOnBox(s_center);

    Sphere s(s_center, GetRadius());
    return s.Contains(b_nearest);
}

// double dispatch

inline bool King::Capsule::Collision(Point const& pointIn) const { return Intersects(pointIn); }

inline bool King::Capsule::Collision(Ray const& rayIn) const { Ray ptN1, ptN2; return Intersects(rayIn, &ptN1, &ptN2); }

inline bool King::Capsule::Collision(Line const& lineIn) const { return Intersects(lineIn); }

inline bool King::Capsule::Collision(const Sphere& sphereIn) const { return Intersects(sphereIn); }

inline bool King::Capsule::Collision(Box const& boxIn) const { return Intersects(boxIn); }

bool King::Capsule::Collision(Pyramid const& pyramidIn) const { return false; }// { return pyramidIn.Collision(*this); }

bool King::Material::Read_v1(ifstream& dataFileIn)
{
    if (!dataFileIn.is_open()) return false;
    // lambda
    auto ReadString = [&dataFileIn](string& strOut)
    {
        size_t len;
        dataFileIn.read(reinterpret_cast<char*>(&len), sizeof(len));
        char* temp = new char[len + 1];
        dataFileIn.read(temp, len);
        temp[len] = '\0';
        strOut = temp;
        delete[] temp;
    };
    // read contents
    dataFileIn.read(reinterpret_cast<char*>(&_shaderMethod), sizeof(_shaderMethod));
    ReadString(_name);
    dataFileIn.read(reinterpret_cast<char*>(&_properties), sizeof(_properties));
    dataFileIn.read(reinterpret_cast<char*>(&_fileNames.ver), sizeof(_fileNames.ver));
    ReadString(_fileNames.light);
    ReadString(_fileNames.diffuse);
    ReadString(_fileNames.specular);
    ReadString(_fileNames.specular_strength);
    ReadString(_fileNames.normal);
    ReadString(_fileNames.emmissive);
    ReadString(_fileNames.transparency);
    ReadString(_fileNames.displacement);
    ReadString(_fileNames.stencil);
    ReadString(_fileNames.reflection);

    if (dataFileIn.fail()) return false;
    return true;
}

bool King::Material::Write_v1(ofstream& outfileIn)
{
    if (!outfileIn.is_open()) return false;
    // lambda
    auto WriteString = [&outfileIn](const string& str)
    {
        size_t len = str.size();
        outfileIn.write(reinterpret_cast<const char*>(&len), sizeof(len));
        outfileIn.write(str.c_str(), len);
    };
    // write contents
    outfileIn.write(reinterpret_cast<char*>(&_shaderMethod), sizeof(_shaderMethod));
    WriteString(_name);
    outfileIn.write(reinterpret_cast<char*>(&_properties), sizeof(_properties));
    outfileIn.write(reinterpret_cast<char*>(&_fileNames.ver), sizeof(_fileNames.ver));
    WriteString(_fileNames.light);
    WriteString(_fileNames.diffuse);
    WriteString(_fileNames.specular);
    WriteString(_fileNames.specular_strength);
    WriteString(_fileNames.normal);
    WriteString(_fileNames.emmissive);
    WriteString(_fileNames.transparency);
    WriteString(_fileNames.displacement);
    WriteString(_fileNames.stencil);
    WriteString(_fileNames.reflection);

    if (outfileIn.fail()) return false;
    return true;
}
/******************************************************************************
*    Method:    Read_v1
******************************************************************************/
bool King::BoneHierarchy::Read_v1(ifstream& dataFileIn)
{
    bool good = true;

    if (!dataFileIn.is_open()) return false;
    // lambda
    auto ReadString = [&dataFileIn](string& strOut)
    {
        size_t len;
        dataFileIn.read(reinterpret_cast<char*>(&len), sizeof(len));
        char* temp = new char[len + 1];
        dataFileIn.read(temp, len);
        temp[len] = '\0';
        strOut = temp;
        delete[] temp;
    };
    // read contents
    uint32_t num_bones;
    dataFileIn.read(reinterpret_cast<char*>(&num_bones), sizeof(num_bones));
    while (dataFileIn && num_bones)
    {
        Bone bone;

        ReadString(bone._name);
        dataFileIn.read(reinterpret_cast<char*>(&bone._transform), sizeof(bone._transform));
        
        _skeleton._bones.push_back({ bone._name, bone._transform });
        --num_bones;
    }

    uint32_t num_toParentTransform;
    dataFileIn.read(reinterpret_cast<char*>(&num_toParentTransform), sizeof(num_toParentTransform));
    while (dataFileIn && num_toParentTransform)
    {
        DirectX::XMFLOAT4X4 toParentTransform;

        dataFileIn.read(reinterpret_cast<char*>(&toParentTransform), sizeof(toParentTransform));

        _skeleton._toParentTransform.push_back(toParentTransform);
        --num_toParentTransform;
    }

    uint32_t num_boneInfluenceCount;
    dataFileIn.read(reinterpret_cast<char*>(&num_boneInfluenceCount), sizeof(num_boneInfluenceCount));
    while (dataFileIn && num_boneInfluenceCount)
    {
        uint8_t boneInfluenceCount;

        dataFileIn.read(reinterpret_cast<char*>(&boneInfluenceCount), sizeof(boneInfluenceCount));

        _boneInfluenceCount.push_back(boneInfluenceCount);
        --num_boneInfluenceCount;
    }

    uint32_t num_boneIndex;
    dataFileIn.read(reinterpret_cast<char*>(&num_boneIndex), sizeof(num_boneIndex));
    while (dataFileIn && num_boneIndex)
    {
        uint8_t boneIndex;

        dataFileIn.read(reinterpret_cast<char*>(&boneIndex), sizeof(boneIndex));

        _boneIndex.push_back(boneIndex);
        --num_boneIndex;
    }

    uint32_t num_boneWeight;
    dataFileIn.read(reinterpret_cast<char*>(&num_boneWeight), sizeof(num_boneWeight));
    while (dataFileIn && num_boneWeight)
    {
        uint8_t boneWeight;

        dataFileIn.read(reinterpret_cast<char*>(&boneWeight), sizeof(boneWeight));

        _boneWeight.push_back(boneWeight);
        --num_boneWeight;
    }

    if (dataFileIn.fail() || !good) return false;
    return true;
}
/******************************************************************************
*    Method:    Write_v1
******************************************************************************/
bool King::BoneHierarchy::Write_v1(ofstream& outfileIn)
{
    if (!outfileIn.is_open()) return false;
    // lambda
    auto WriteString = [&outfileIn](const string& str)
    {
        size_t len = str.size();
        outfileIn.write(reinterpret_cast<const char*>(&len), sizeof(len));
        outfileIn.write(str.c_str(), len);
    };
    // write contents
    // Skeleton
    size_t num_bones = _skeleton._bones.size();
    outfileIn.write(reinterpret_cast<const char*>(&num_bones), sizeof(num_bones));
    for (const auto &bone : _skeleton._bones)
    {
        // vector
        WriteString(bone._name);
        outfileIn.write(reinterpret_cast<const char*>(&bone._transform), sizeof(bone._transform));
    }
    size_t num_toParentTransform = _skeleton._toParentTransform.size();
    outfileIn.write(reinterpret_cast<const char*>(&num_toParentTransform), sizeof(num_toParentTransform));
    for (const auto& toParentTransform : _skeleton._toParentTransform)
    {
        // vector
        outfileIn.write(reinterpret_cast<const char*>(&toParentTransform), sizeof(toParentTransform));
    }
    size_t num_boneInfluenceCount = _boneInfluenceCount.size();
    outfileIn.write(reinterpret_cast<const char*>(&num_boneInfluenceCount), sizeof(num_boneInfluenceCount));
    for (const auto& boneInfluence : _boneInfluenceCount)
    {
        // vector
        outfileIn.write(reinterpret_cast<const char*>(&boneInfluence), sizeof(boneInfluence));
    }
    size_t num_boneIndex = _boneIndex.size();
    outfileIn.write(reinterpret_cast<const char*>(&num_boneIndex), sizeof(num_boneIndex));
    for (const auto& boneIndex : _boneIndex)
    {
        // vector
        outfileIn.write(reinterpret_cast<const char*>(&boneIndex), sizeof(boneIndex));
    }
    size_t num_boneWeight = _boneWeight.size();
    outfileIn.write(reinterpret_cast<const char*>(&num_boneWeight), sizeof(num_boneWeight));
    for (const auto& boneWeight : _boneWeight)
    {
        // vector
        outfileIn.write(reinterpret_cast<const char*>(&boneWeight), sizeof(boneWeight));
    }

    if (outfileIn.fail()) return false;
    return true;
}
/******************************************************************************
*    VertexAttrib:    GetByteSize
******************************************************************************/
uint32_t King::VertexAttrib::GetByteSize() const
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
/******************************************************************************
*    Path:    Get_Polygon2DF
*       convert a 3D Path that is planar to the XY plane
*       Note, if all the points are not in a plane then they will be rotates
*       and z and w will be zero'd out regardless, this projects the Path
*       onto the xy plane using the first, second, and last points to form
*       the plane normal to align with the XY plane first.
******************************************************************************/
Polygon2DF King::Path::Get_Polygon2DF() const
{
    assert(size() > 2);
    vector<float2> out(size());

    const auto& b = begin();
    const auto& n = std::next(b,1);
    const auto& e = std::prev(end(),1);
    // RHS CCW
    auto normal = Cross(*n - *b, *e - *b);
    normal.MakeNormalize();
    // from / to rotation
    const float3 to = float3(0.f, 0.f, 1.f);
    Quaternion toPlanarSpace(normal, to);

    const DirectX::XMMATRIX m = toPlanarSpace;
    const auto p2 = *this * m;

    for (const auto& ea : p2)
    {
        out.push_back(float2(ea));
    }

    return Polygon2DF();
}
/******************************************************************************
*    Path:    Offset
*       Transforms the path outward by a set amount
******************************************************************************/
void King::Path::Offset(const float dist, Plane offsetPlaneIn)
{
    auto temp = *this; // copy
    for (size_t i = 1; i < size(); ++i)
    {
        Line l(at(i - 1), at(i));
        l.Offset(dist, offsetPlaneIn);
        temp[i - 1] = l.GetVertex(0);
        temp[i] = l.GetVertex(1);
    }
    swap(temp);
}
