/***************************************************************************/
/*                                                                         */
/*  File: CubeCollision.cpp                                                */
/*  Date: 10-11-07                                                         */
/*                                                                         */
/***************************************************************************/
/*
	See .h file
*/
/***************************************************************************/

#include <math.h>		// cosf(..) sinf(..), sqrt(..)
#include <stdio.h>      // sprintf(..)

#include <stdio.h>					// sprintf(..)

#include "CK_CubeCollision.h"

using namespace std;
using namespace King;

// Globals
const static float      g_sleepEpsilon = 0.05f; // 0 to disable, 0.05f for test value
const static bool       g_startSleeping = false;
const static float		g_friction = 0.6f;
const static bool		g_positionCorrection = true;  // Sinking fix!
const static int		g_numIterations = 7;
int                     g_numCols = 0;

extern stCollisionPoints g_CollisionsArray[100];
// Single list of all the collisions are stored in this variable

namespace King {
    /***************************************************************************/

    void King::stCollisions::Add(Cube* box0, Cube* box1, float3& point, float3& normal, float pen)
    {
        // First we determine if we have any collisions between these two
        // rigid bodies and store them in that array...so we can group
        // rigid body collisions....very useful in the long run
        stCollisionPoints* cp = NULL;

        for (int i = 0; i < g_numCols; i++)
        {
            if (g_CollisionsArray[i].box0 == box0 &&
                g_CollisionsArray[i].box1 == box1)
            {
                cp = &g_CollisionsArray[i];
                break;
            }
        }

        // We've not found one, hence add it to our list, with the data
        // and return
        if (cp == NULL)
        {
            g_CollisionsArray[g_numCols].box0 = box0;
            g_CollisionsArray[g_numCols].box1 = box1;
            g_CollisionsArray[g_numCols].numPoints = 1;
            g_CollisionsArray[g_numCols].points[0].normal = normal;
            g_CollisionsArray[g_numCols].points[0].point = point;
            g_CollisionsArray[g_numCols].points[0].pos0 = box0->m_c;
            g_CollisionsArray[g_numCols].points[0].pos1 = box1->m_c;
            g_CollisionsArray[g_numCols].points[0].pen = pen;

            g_numCols++;
            return;
        }


        // Multiple collision points between a single rigid body, so add
        // it to our array
        cp->points[cp->numPoints].normal = normal;
        cp->points[cp->numPoints].point = point;
        cp->points[cp->numPoints].pos0 = box0->m_c;
        cp->points[cp->numPoints].pos1 = box1->m_c;
        cp->points[cp->numPoints].pen = pen;

        cp->numPoints++;


        if (cp->numPoints > 99)
        {
            // Assert here, as I've only made the array 100 in size
            // so will lead to memory corruption
            assert(0);
        }

        if (g_numCols > 99)
        {
            // Our array is only 100, so if we get to this size, it will
            // lead to memory corruption, debug halt here in the debugger!
            assert(0);
        }

    }




    inline void CalculateInterval(const Cube& box, const float3& axis, float& min, float& max)
    {
        float x = box.m_e.f[0];
        float y = box.m_e.f[1];
        float z = box.m_e.f[2];
        float3 Vertex[8] =
        {
            float3(x,    y,    z),
            float3(-x,    y,	  z),
            float3(x,   -y,	  z),
            float3(-x,   -y,	  z),

            float3(x,    y,	 -z),
            float3(-x,    y,	 -z),
            float3(x,   -y,	 -z),
            float3(-x,   -y,	 -z)
        };

        for (int i = 0; i < 8; i++)
        {
            Vertex[i] *= box.m_matWorld;
            //D3DXVec3TransformCoord(&Vertex[i], &Vertex[i], &box.m_matWorld);
        }

        min = max = Dot(Vertex[0], axis);//D3DXVec3Dot(&Vertex[0], &axis);
        for (int i = 0; i < 8; i++)
        {
            float d = Dot(Vertex[i], axis);// D3DXVec3Dot(&Vertex[i], &axis);

            if (d < min) min = d;
            if (d > max) max = d;
        }
    }

    /***************************************************************************/

    inline bool SpanIntersect(const Cube& box0,
        const Cube& box1,
        const float3& axisc,
        float* minPenetration = NULL,
        float3* axisPenetration = NULL,
        float* pen = NULL)
    {

        float3 axis = axisc;

        float lq = axis.GetMagnitude();
        if (lq <= 0.02f)
        {
            if (pen) *pen = 100000.0f;
            return true;
        }

        axis = Normalize(axis);

        float mina, maxa;
        float minb, maxb;
        CalculateInterval(box0, axis, mina, maxa);
        CalculateInterval(box1, axis, minb, maxb);

        float lena = maxa - mina;
        float lenb = maxb - minb;

        float minv = min(mina, minb);
        float maxv = max(maxa, maxb);
        float lenv = maxv - minv;

        if (lenv > (lena + lenb))
        {
            // Collision
            return false;
        }

        /*
        // You could do this test, but I prefair the above
        // method
        if (mina>maxb || minb>maxa)
        {
            return false;
        }
        */

        float penetration = (lena + lenb) - lenv;

        if (pen)
        {
            *pen = penetration;
        }

        if (minPenetration && axisPenetration)
        {
            if (penetration < *minPenetration)
            {
                *minPenetration = penetration;
                *axisPenetration = axis;

                // BoxA pushes BoxB away in the correct Direction
                if (minb < mina)
                {
                    *axisPenetration *= -1;
                }
            }
        }


        // Colllision
        return true;
    }

    /***************************************************************************/

    inline int GetNumHitPoints(const Cube& box0,
        const float3& hitNormal,
        const float penetration,
        float3 verts[8],
        int* vertIndexs)
    {

        //{
        //	DrawLine3D(g_pD3DDevice, box0.m_c, (box0.m_c + hitNormal*3.0f), 0xffff00ff);
        //}


        float x = box0.m_e.GetX();
        float y = box0.m_e.GetY();
        float z = box0.m_e.GetZ();
        float3 Vertex[8] =
        {
            float3(-x,   -y,   -z),
            float3(x,   -y,	  -z),
            float3(x,   -y,	   z),
            float3(-x,   -y,	   z),

            float3(-x,    y,	  -z),
            float3(x,    y,	  -z),
            float3(x,    y,	   z),
            float3(-x,    y,	   z)
        };

        for (int i = 0; i < 8; i++)
        {
            //D3DXVec3TransformCoord(&Vertex[i], &Vertex[i], &box0.m_matWorld);
            Vertex[i] *= box0.m_matWorld;
        }

        float3 planePoint = Vertex[0];
        //float maxdist = D3DXVec3Dot(&Vertex[0], &hitNormal);
        float maxdist = Dot(Vertex[0], hitNormal);

        for (int i = 0; i < 8; i++)
        {
            float d = Dot(Vertex[i], hitNormal);
            if (d > maxdist)
            {
                maxdist = d;
                planePoint = Vertex[i];
            }
        }

        // Plane Equation (A dot N) - d = 0;

        float d = Dot(planePoint, hitNormal);
        d -= penetration + 0.01f;


        int numVerts = 0;
        for (int i = 0; i < 8; i++)
        {
            float side = Dot(Vertex[i], hitNormal) - d;

            if (side > 0)
            {
                verts[numVerts] = Vertex[i];
                vertIndexs[numVerts] = i;
                numVerts++;
            }
        }

        return numVerts;
    }

    /***************************************************************************/

    inline void SortVertices(float3* verts0,
        int* vertIndexs0)
    {
        int faces[6][4] =
        {
            {4,0,3,7},
            {1,5,6,2},
            {0,1,2,3},
            {7,6,5,4},
            {5,1,0,4},
            {6,7,3,2}
        };

        int faceSet = -1;
        float3 temp[4]; // New correct clockwise order

        // Work out which face to use
        for (int i = 0; i < 6 && faceSet == -1; i++)
        {
            int numFound = 0;
            for (int j = 0; j < 4; j++)
            {
                if (vertIndexs0[j] == faces[i][j])
                {
                    temp[numFound] = verts0[j];
                    numFound++;

                    if (numFound == 4)
                    {
                        faceSet = i;
                        break;
                    }
                }
            }
        }

        if (faceSet < 0)
        {
            int errorHasOccured = 1;
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                verts0[i] = temp[i];
            }
        }
    }

    /***************************************************************************/

    inline bool VertInsideFace(float3* verts0, float3& p0, float planeErr = 0.0f)
    {
        // Four points of a plane face
        // test if p0 is
        //
        // Work out the normal for the face
        // EX: verts0[0] = {2.5f, -2.5f, -2.5f} & verts0[1] = {2.5f, -2.5f, 2.5f} & verts0[2] = {2.5f, 2.5f, -2.5f} & verts0[3] = {2.5f, 2.5f, 2.5f}
        //     p0 = {2.5f, -2.5f, -2.5f}, so this test will fail
        //
        // [3].     2.5 | +Y      .[2]
        //              |
        //              |
        //    2.5       |
        // +Z --------------------- -Z
        //              | 
        //              |
        //              |
        // [1].    -2.5 | -Y   [0].
        // face normal for CCW RHS ordering is down the +X axis (out of the diagram)
        float3 v0 = verts0[1] - verts0[0]; // Ex: {0.f, 0.f, 5.f}
        float3 v1 = verts0[2] - verts0[0]; // Ex: {0.f, 5.f, 0.f}
        float3 n = Cross(v1, v0); // Ex: {25.f, 0.f, 0.f}
        n = Normalize(n); // Ex: {1.f, 0.f, 0.f}

        // now each vertex we use the face normal, n, to project a new point, sx, out in front of the our face.
        // then using CCW RHS we check if it is outside our plane projection
        for (int i = 0; i < 4; i++)
        {
            float3 s0 = verts0[i]; // Ex: {2.5f, -2.5f, 2.5f}; Ex2: {2.5f, 2.5f, -2.5f}
            float3 s1 = verts0[(i + 1) % 4]; // Ex: {2.5f, -2.5f, 2.5f}; Ex2: {2.5f, 2.5f, 2.5f}
            float3 sx = s0 + n * 10.0f;  // Ex: {12.5f, -2.5f, -2.5f}; // Ex2: {12.5f, +2.5f, -2.5f}

            // Work out the normal for the face
            float3 sv0 = s1 - s0; // Ex: {0.f, 0.f, 5.f}
            float3 sv1 = sx - s0; // Ex: {10.f, 0.f, 0.f}
            float3 sn = Cross(sv1, sv0); // Ex: {0.f, -50.f, 0.f}
            sn = Normalize(sn); // Ex: {0.f, -1.f, 0.f}

            float d = Dot(s0, sn); // Ex: 2.5
            float d0 = Dot(p0, sn) - d; // Ex: 0
            // Ex: loop 1,2 pass and 3 it fails
            // d0 = 0.0
            // d0 = -3.53553391
            // d0 = 5.0
            // Outside the plane
            if (d0 > planeErr) // Ex: 0 > 0 false
            {
                return false;
            }
        }

        return true;
    }


    inline void ClipFaceFaceVerts(float3* verts0,
        int* vertIndexs0,
        float3* verts1,
        int* vertIndexs1,
        float3* vertsX,
        int* numVertsX)
    {
        SortVertices(verts0, vertIndexs0);
        SortVertices(verts1, vertIndexs1);

        // Work out the normal for the face
        float3 v0 = verts0[1] - verts0[0];
        float3 v1 = verts0[2] - verts0[0];
        float3 n = Cross(v1, v0);
        n = Normalize(n);

        // Project all the vertices onto a shared plane, plane0
        float3 vertsTemp1[4];
        for (int i = 0; i < 4; i++)
        {
            vertsTemp1[i] = verts1[i] + (n * Dot(n, verts0[0] - verts1[i]));
        }


        static float3 temp[50];
        int numVerts = 0;

        for (int c = 0; c < 2; c++)
        {
            float3* vertA = vertsTemp1;
            float3* vertB = verts0;
            if (c == 1)
            {
                vertA = verts0;
                vertB = vertsTemp1;
            }

            // Work out the normal for the face
            float3 v0 = vertA[1] - vertA[0];
            float3 v1 = vertA[2] - vertA[0];
            float3 n = Cross(v1, v0);
            n = Normalize(n);

            for (int i = 0; i < 4; i++)
            {
                float3 s0 = vertA[i];
                float3 s1 = vertA[(i + 1) % 4];
                float3 sx = s0 + n * 10.0f;

                // Work out the normal for the face
                float3 sv0 = s1 - s0;
                float3 sv1 = sx - s0;
                float3 sn = Cross(sv1, sv0);
                sn = Normalize(sn);

                float d = Dot(s0, sn);


                for (int j = 0; j < 4; j++)
                {
                    float3 p0 = vertB[j];
                    float3 p1 = vertB[(j + 1) % 4]; // Loops back to the 0th for the last one

                    float d0 = Dot(p0, sn) - d;
                    float d1 = Dot(p1, sn) - d;

                    // Check there on opposite sides of the plane
                    if ((d0 * d1) < 0.0f)
                    {
                        float3 pX = p0 + (Dot(sn, (s0 - p0)) / Dot(sn, (p1 - p0))) * (p1 - p0);

                        if (VertInsideFace(vertA, pX, 0.1f))
                        {
                            temp[numVerts] = pX;
                            numVerts++;
                        }

                    }


                    if (VertInsideFace(vertA, p0))
                    {
                        temp[numVerts] = p0;
                        numVerts++;
                    }


                    if (numVerts > 40)
                    {
                        // We have a fixed array we pass in, which has a max size of 50
                        // so if we go past this we'll have memory corruption

                        // temp above is size 50 as well
                        //DBG_HALT;
                        assert(0);
                    }
                }
            }



        }

        // Remove verts which are very close to each other
        for (int i = 0; i < numVerts; i++)
        {
            for (int j = i; j < numVerts; j++)
            {
                if (i != j)
                {
                    //float dist = D3DXVec3LengthSq(&(temp[i] - temp[j]));
                    float dist = float3::Magnitude(temp[i] - temp[j]);

                    if (dist < 6.5f)
                    {

                        for (int k = j; k < numVerts; k++)
                        {
                            temp[k] = temp[k + 1];
                        }
                        numVerts--;
                    }
                }
            }
        }

        *numVertsX = numVerts;
        for (int i = 0; i < numVerts; i++)
        {
            vertsX[i] = temp[i];
        }
    }

    /***************************************************************************/


    inline void ClipFaceFaceVertsOld(float3* verts0,
        int* vertIndexs0,
        float3* verts1,
        int* vertIndexs1)
    {
        SortVertices(verts0, vertIndexs0);
        SortVertices(verts1, vertIndexs1);

        // Work out the normal for the face
        float3 v0 = verts1[1] - verts1[0];
        float3 v1 = verts1[2] - verts1[0];
        float3 n = Cross(v1, v0);
        n = Normalize(n);

        //DrawLine(g_pD3DDevice, v0, v0 + n*4.0f);

        float3 temp[4];

        for (int i = 0; i < 4; i++)
        {
            float3 s0 = verts1[i];
            float3 s1 = verts1[(i + 1) % 4];
            float3 sx = s0 + n * 10.0f;

            // Work out the normal for the face
            float3 sv0 = s1 - s0;
            float3 sv1 = sx - s0;
            float3 sn = Cross(sv1, sv0);
            sn = Normalize(sn);


            //DrawLine3D(g_pD3DDevice, (s0 + (s1-s0)*0.5f), (s0 + (s1-s0)*0.5f) + sn*1.0f);

            float d = Dot(s0, sn);

            for (int r = 0; r < 4; r++)
            {
                temp[r] = verts0[r];
            }

            for (int j = 0; j < 4; j++)
            {
                float3 p0 = verts0[j];
                float3 p1 = verts0[(j + 1) % 4]; // Loops back to the 0th for the last one

                float d0 = Dot(p0, sn) - d;
                float d1 = Dot(p1, sn) - d;

                // Check there on opposite sides of the plane
                if ((d0 * d1) < 0.0f)
                {
                    float3 pX = p0 + (Dot(sn, (s0 - p0)) / Dot(sn, (p1 - p0))) * (p1 - p0);

                    if (d0 > 0.0f)
                    {
                        //verts0[j] = pX;
                        temp[j] = pX;
                    }
                    else
                    {
                        //verts0[(j+1)%4] = pX;
                        temp[(j + 1) % 4] = pX;
                    }
                }
            }

            for (int r = 0; r < 4; r++)
            {
                verts0[r] = temp[r];
            }
        }

        //static debugsphere sphere(g_pD3DDevice);
        //for (int i=0; i<4; i++)
        //{
        //	sphere.Draw(g_pD3DDevice, verts0[i].x, verts0[i].y, verts0[i].z, 0.4f);
        //}

    }

    /***************************************************************************/

    inline void ClosestPtPointOBB(const float3& point,
        const Cube& box0,
        float3* closestP)
    {
        float3 d = point - box0.m_c;

        float3 q = box0.m_c;

        // *** TO DO *** likely does not work, look at another source or modify the below dist.xyz compare vs component wise extent (for a non-uniformed box)

        for (int i = 0; i < 3; i++)
        {
            auto dist = float3(Dot(d, box0.m_u[i]));

            if (dist > box0.m_e) dist = box0.m_e;
            if (dist < -box0.m_e) dist = -box0.m_e;

            q += dist * box0.m_u[i];
        }

        *closestP = q;
    }

    /***************************************************************************/

    void ClipLinePlane(const float3* verts0,	// 2 verts
        const int* vertIndexs0,
        const Cube& box0,
        const float3* verts1, // 4 verts
        int* vertIndexs1,
        const Cube& box1,
        float3* vertsX,
        int* numVertX)
    {

        // Work out the normal for the face
        //float3 v0 = verts1[1] - verts1[0];
        //float3 v1 = verts1[2] - verts1[0];
        //float3 N  = Cross(v1, v0);
        //N = Normalize(N);


        //float3 P0 = verts0[0];
        //float3 P1 = verts0[1];
        //float3 P2 = verts1[0];

        //float3 Px = P0   +   ( Dot(N, (P2-P0)) /  Dot(N, (P1-P0)) )  *   (P1 - P0);


        //float d = Dot(P2, N);

        //float t0 = Dot(P0, N) - d;
        //float t1 = Dot(P1, N) - d;


        ClosestPtPointOBB(verts0[0], box1, &vertsX[0]);
        ClosestPtPointOBB(verts0[1], box1, &vertsX[1]);
        *numVertX = 2;

    }

    /***************************************************************************/

    void ClosestPointLineLine(const float3* verts0,	// 2 verts
        const float3* verts1, // 2 verts
        float3* vertsX,
        int* numVertX)
    {
        float3 p1 = verts0[0];
        float3 q1 = verts0[1];
        float3 p2 = verts1[0];
        float3 q2 = verts1[1];

        float3 d1 = q1 - p1;
        float3 d2 = q2 - p2;
        float3 r = p1 - p2;
        float a = Dot(d1, d1);
        float e = Dot(d2, d2);
        float f = Dot(d2, r);

        const float epsilon = 0.00001f;

        float s, t;
        float3 c1, c2;

        if (a <= epsilon && e <= epsilon)
        {
            s = t = 0.0f;
            c1 = p1;
            c2 = p2;

            vertsX[0] = c1;
            *numVertX = 1;
            return;
        }

        if (a <= epsilon)
        {
            s = 0.0f;
            t = f / e;
            t = Clamp(t, 0.0f, 1.0f);
        }
        else
        {
            float c = Dot(d1, r);
            if (e <= epsilon)
            {
                t = 0.0f;
                s = Clamp(-c / a, 0.0f, 1.0f);
            }
            else
            {
                float b = Dot(d1, d2);
                float denom = a * e - b * b;

                if (denom != 0.0f)
                {
                    s = Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
                }
                else
                {
                    s = 0.0f;
                }

                t = (b * s + f) / e;

                if (t < 0.0f)
                {
                    t = 0.0f;
                    s = Clamp(-c / a, 0.0f, 1.0f);
                }
                else if (t > 1.0f)
                {
                    t = 1.0f;
                    s = Clamp((b - c) / a, 0.0f, 1.0f);
                }
            }
        }

        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;

        vertsX[0] = (c1 + c2) * 0.5f;
        *numVertX = 1;
    }

    /***************************************************************************/

    void CalculateHitPoint(const Cube& box0,
        const Cube& box1,
        const float penetration,
        float3& hitNormal,
        float3* hitPoints,
        int* numHitPoints)
    {
        float3 verts0[8];
        int vertIndex0[8];
        float3 norm0 = hitNormal;
        int numVerts0 = GetNumHitPoints(box0,
            norm0,
            penetration,
            verts0,
            vertIndex0);

        float3 verts1[8];
        int vertIndex1[8];
        float3 norm1 = -hitNormal;
        int numVerts1 = GetNumHitPoints(box1,
            norm1,
            penetration,
            verts1,
            vertIndex1);

        // This should never really happen!
        if (numVerts0 == 0 || numVerts1 == 0)
        {
            return;
        }


        int numVertsX = numVerts0;
        float3* vertsX = verts0;
        float3 clipVerts[50];

        if (numVerts0 >= 4 && numVerts1 >= 4)
        {

            ClipFaceFaceVerts(verts0, vertIndex0,
                verts1, vertIndex1,
                clipVerts, &numVertsX);

            vertsX = clipVerts;
        }


        {
            // TO-DO - work out which one is the least number
            // of verts, and use that, if both have two, work out
            // the edge point exactly...if its just a single point, only
            // use that single vert

            // TO-DO** TO-DO
            //int numVertsX = numVerts0;
            //float3* vertsX = verts0;


            if (numVerts1 < numVerts0)
            {
                numVertsX = numVerts1;
                vertsX = verts1;
                hitNormal = -norm1;
            }

            if (numVerts1 == 2 && numVerts0 == 2)
            {
                static float3 V[2];
                static int numV = 0;

                ClosestPointLineLine(verts0,
                    verts1,
                    V, &numV);

                vertsX = V;
                numVertsX = numV;
            }

            if (numVerts0 == 2 && numVerts1 == 4)
            {
                ClipLinePlane(verts0, vertIndex0, box0,
                    verts1, vertIndex1, box1,
                    vertsX, &numVertsX);
            }

            if (numVerts0 == 4 && numVerts1 == 2)
            {
                ClipLinePlane(verts1, vertIndex1, box1,
                    verts0, vertIndex0, box0,
                    vertsX, &numVertsX);
            }



            /*
            static debugsphere sphere0(g_pD3DDevice, 0xff00ff00);
            for (int i=0; i<numVerts0; i++)
            {
                sphere0.Draw(g_pD3DDevice, verts0[i].x, verts0[i].y, verts0[i].z, 0.5f);
            }

            static debugsphere sphere1(g_pD3DDevice, 0xff005500);
            for (int i=0; i<numVerts1; i++)
            {
                sphere1.Draw(g_pD3DDevice, verts1[i].x, verts1[i].y, verts1[i].z, 0.5f);
            }
            */


            //char buf[128];
            //sprintf(buf, "spheres: %d\n", numVertsX);
            //
            //static debugsphere sphere(g_pD3DDevice, 0xff000000);
            //for (int i=0; i<numVertsX; i++)
            //{
            //	sphere.Draw(g_pD3DDevice, vertsX[i].x, vertsX[i].y, vertsX[i].z, 0.5f);
            //}



            /*
            *hitPoint = float3(0,0,0);
            // Work out the average hit point
            for (int i=0; i<numVertsX; i++)
            {
                *hitPoint += vertsX[i];
            }
            *hitPoint = (*hitPoint) * (1.0f/numVertsX);
            */

            * numHitPoints = numVertsX;
            for (int i = 0; i < numVertsX; i++)
            {
                hitPoints[i] = vertsX[i];
            }
        }


    }

    /***************************************************************************/

    bool CubeCubeCollisionCheck(const Cube& box0,
        const Cube& box1,
        float3* hitPoints,
        int* numHitPoints,
        float* penetration,
        float3* hitNormal)
    {

        // Simple bounding sphere check first
        float len = (box0.m_radius + box1.m_radius);
        if (float3::Magnitude((box1.m_c - box0.m_c)) > (len * len))
        {
            return false;
        }

        bool hit = true;
        float p = 10000.0f;
        float3 h;

        hit &= SpanIntersect(box0, box1, box0.m_u[0], &p, &h);
        hit &= SpanIntersect(box0, box1, box0.m_u[1], &p, &h);
        hit &= SpanIntersect(box0, box1, box0.m_u[2], &p, &h);

        hit &= SpanIntersect(box0, box1, box1.m_u[0], &p, &h);
        hit &= SpanIntersect(box0, box1, box1.m_u[1], &p, &h);
        hit &= SpanIntersect(box0, box1, box1.m_u[2], &p, &h);

        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[0], box1.m_u[0]), &p, &h);
        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[0], box1.m_u[1]), &p, &h);
        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[0], box1.m_u[2]), &p, &h);

        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[1], box1.m_u[0]), &p, &h);
        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[1], box1.m_u[1]), &p, &h);
        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[1], box1.m_u[2]), &p, &h);

        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[2], box1.m_u[0]), &p, &h);
        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[2], box1.m_u[1]), &p, &h);
        hit &= SpanIntersect(box0, box1, Cross(box0.m_u[2], box1.m_u[2]), &p, &h);

        *numHitPoints = 0;

        if (penetration && hitNormal && hit)
        {
            CalculateHitPoint(box0,
                box1,
                p,
                h,
                hitPoints,
                numHitPoints);
            *penetration = p;
            *hitNormal = -h;
        }
        return hit;
    }

    /***************************************************************************/

}



