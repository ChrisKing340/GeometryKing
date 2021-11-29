/***************************************************************************/
/*                                                                         */
/*  File: Cube.h                                                           */
/*  Date: 24-10-07                                                         */
/*                                                                         */
/***************************************************************************/
/*
*/
/***************************************************************************/
#include "..\MathSIMD\MathSIMD.h"

#ifndef CUBE_H
#define CUBE_H

extern const float      g_sleepEpsilon; // 0 to disable, 0.05f for test value
extern const bool       g_startSleeping;
extern const float		g_friction;
extern const bool		g_positionCorrection;  // Sinking fix!
extern const int		g_numIterations;
extern int              g_numCols;

namespace King {

    inline DirectX::XMMATRIX Inverse(DirectX::XMMATRIX& m)
    {
        //DirectX::XMMATRIX temp;
        return DirectX::XMMatrixInverse(nullptr, m);
        //DirectX::XMMATRIXInverse(&temp, NULL, &m);
        //return temp;
    }



    //*****************************************************************************


    class Cube
    {
    public:
        float3 m_c;			// Center Of Cube
        float3 m_u[3];		// Local x-y-z Axes
        float3 m_e;			// Positive halfwidths along each axis

        DirectX::XMMATRIX  m_matWorld;
        Quaternion m_rot;


        float		        m_mass;
        DirectX::XMMATRIX	m_boxInertia;
        DirectX::XMMATRIX	m_boxInvInertia;
        DirectX::XMMATRIX	m_invInertia;

        float3 m_linVelocity;
        float3 m_angVelocity;

        float3 m_forces;
        float3 m_torques;


        float		    m_radius;
        float		    m_rwaMotion;
        bool		    m_awake;


        Cube()
        {
            //Cube( float3(0,0,0),  float3(0,0,0),		 float3(5,5,5), 1.0f);
        }

        Cube(float3 pos, float3 rot, float3 size, float mass)
        {
            m_c = pos;
            m_e = size;
            m_e *= 0.5f;


            Quaternion QX(float3(1, 0, 0), rot.GetX());// = DirectX::XMQuaternionRotationAxis(float3(1, 0, 0), rot.GetX());
            Quaternion QY(float3(0, 1, 0), rot.GetY()); //D3DXQUATERNIONRotationAxis(&QY, &float3(0,1,0), rot.y);
            Quaternion QZ(float3(0, 0, 1), rot.GetZ()); //D3DXQUATERNIONRotationAxis(&QZ, &float3(0,0,1), rot.z);
            m_rot = QZ;
            m_rot *= QY;
            m_rot *= QX;
            m_rot = DirectX::XMQuaternionNormalize(m_rot);
            // D3DXQUATERNIONNormalize(&m_rot, &m_rot);

            m_mass = mass;
            m_linVelocity = float3(0, 0, 0);
            m_angVelocity = float3(0, 0, 0);

            m_forces = float3(0, 0, 0);
            m_torques = float3(0, 0, 0);


            // If we want our objects to start awake or sleeping
            if (g_startSleeping)
            {
                m_rwaMotion = g_sleepEpsilon;
                m_awake = false;
            }
            else
            {
                m_rwaMotion = 2 * g_sleepEpsilon;
                m_awake = true;
            }
            m_radius = float3::Magnitude(size) + 0.4f;
            //m_radius = sqrtf(size.x*size.x + size.y*size.y + size.z*size.z) + 0.4f;

            UpdateMatrix();
        }


        //void AddRotateX(float xRotInc)
        //{
        //	Quaternion QX; D3DXQUATERNIONRotationAxis(&QX, &float3(1,0,0), xRotInc);
        //	m_rot *= QX;
     //       D3DXQUATERNIONNormalize(&m_rot, &m_rot);
        //	UpdateMatrix();
        //}

        //void AddRotateZ(float zRotInc)
        //{
        //	Quaternion QZ; D3DXQUATERNIONRotationAxis(&QZ, &float3(0,0,1), zRotInc);
        //	m_rot *= QZ;
     //       D3DXQUATERNIONNormalize(&m_rot, &m_rot);
        //	UpdateMatrix();
        //}

        //void AddRotateY(float yRotInc)
        //{
        //	Quaternion QY; D3DXQUATERNIONRotationAxis(&QY, &float3(0,1,0), yRotInc);
        //	m_rot *= QY;
     //       D3DXQUATERNIONNormalize(&m_rot, &m_rot);
        //	UpdateMatrix();
        //}


        void UpdateMatrix()
        {
            DirectX::XMMATRIX matR;
            matR = m_rot;
            //DirectX::XMMATRIXRotationQuaternion(&matR, &m_rot);

            m_u[0] = float3(1, 0, 0);
            m_u[1] = float3(0, 1, 0);
            m_u[2] = float3(0, 0, 1);

            m_u[0] *= matR;
            m_u[1] *= matR;
            m_u[2] *= matR;
            //D3DXVec3TransformCoord(&m_u[0], &m_u[0], &matR);
            //D3DXVec3TransformCoord(&m_u[1], &m_u[1], &matR);
            //D3DXVec3TransformCoord(&m_u[2], &m_u[2], &matR);

            DirectX::XMMATRIX matT = DirectX::XMMatrixTranslationFromVector(m_c);
            //DirectX::XMMATRIXTranslation(&matT, m_c.x, m_c.y, m_c.z);
            m_matWorld = matR * matT;


            float3 size = m_e * 2.0f;

            float3 s = size * size;

            //float x2 = (size.x * size.x);
            //float y2 = (size.y * size.y);
            //float z2 = (size.z * size.z);
            float ix = (s.GetY() + s.GetZ()) * m_mass / 12.0f;
            float iy = (s.GetX() + s.GetZ()) * m_mass / 12.0f;
            float iz = (s.GetX() + s.GetY()) * m_mass / 12.0f;
            //float ix = (y2 + z2) * m_mass / 12.0f;
            //float iy = (x2 + z2) * m_mass / 12.0f;
            //float iz = (x2 + y2) * m_mass / 12.0f;


            m_boxInertia = DirectX::XMMATRIX(ix, 0, 0, 0,
                0, iy, 0, 0,
                0, 0, iz, 0,
                0, 0, 0, 1);


            m_invInertia = Inverse(matR) * Inverse(m_boxInertia) * matR;
        }

        //void Render(IDirect3DDevice9* pDevice);
        void UpdatePos(float dt);
        void UpdateVel(float dt);



        void AddForce(float3& p, float3& f)
        {
            if (m_mass > 1000.0f || !m_awake) return;

            m_forces += f;
            m_torques += Cross((p - m_c), f);
        }


        //----------------------------------------------------------
        // 
        // j =			                           -(1+Cor)(relv.norm)
        //	     -----------------------------------------------------------------------------------
        //	     norm.norm(1/Mass0 + 1/Mass1) + (sqr(r0 x norm) / Inertia0) + (sqr(r1 x norm) / Inertia1)
        //
        //----------------------------------------------------------

        static void AddCollisionImpulse(Cube& c0,
            Cube& c1,
            float3& hitPoint,
            float3& normal,
            float dt,
            float penetration)
        {

            // Some simple check code.
            if (dt <= 0.0) { return; }

            float invMass0 = (c0.m_mass > 1000.0f) ? 0.0f : (1.0f / c0.m_mass);
            float invMass1 = (c1.m_mass > 1000.0f) ? 0.0f : (1.0f / c1.m_mass);

            invMass0 = (!c0.m_awake) ? 0.0f : invMass0;
            invMass1 = (!c1.m_awake) ? 0.0f : invMass1;

            // Both objects are non movable
            if ((invMass0 + invMass1) == 0.0) return;


            float3 r0 = hitPoint - c0.m_c;
            float3 r1 = hitPoint - c1.m_c;

            float3 v0 = c0.m_linVelocity + Cross(c0.m_angVelocity, r0);
            float3 v1 = c1.m_linVelocity + Cross(c1.m_angVelocity, r1);

            // Relative Velocity
            float3 dv = v0 - v1;

            // NORMAL Impulse Code

            // Compute Normal Impulse
            float vn = Dot(dv, normal);

            // Works out the bias to prevent Prevents sinking!
            const float allowedPenetration = 0.1f;
            const float biasFactor = 0.1f; // 0.1 to 0.3
            float biasFactorValue = g_positionCorrection ? biasFactor : 0.0f;

            float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;
            auto val = penetration - allowedPenetration;
            float bias = biasFactorValue * inv_dt * fmax(0.0f, val);


            float kNormal = invMass0 + invMass1 +

                Dot(normal,
                    Cross((Cross(r0, normal) * c0.m_invInertia), r0) +
                    Cross((Cross(r1, normal) * c1.m_invInertia), r1)
                );

            float massNormal = 1.0f / kNormal;


            float dPn = massNormal * (-vn + bias);

            dPn = fmax(dPn, 0.0f);


            // Apply normal contact impulse
            float3 P = dPn * normal;

            c0.m_linVelocity += invMass0 * P;
            c0.m_angVelocity += Cross(r0, P) * c0.m_invInertia;

            c1.m_linVelocity -= invMass1 * P;
            c1.m_angVelocity -= Cross(r1, P) * c1.m_invInertia;
            // NORMAL



            // TANGENT Impulse Code
            {
                // Work out our tangent vector, with is perpendicular
                // to our collision normal
                float3 tangent = float3(0, 0, 0);
                tangent = dv - (Dot(dv, normal) * normal);
                tangent = Normalize(tangent);

                float kTangent = invMass0 + invMass1 +

                    Dot(tangent,
                        Cross((Cross(r0, tangent) * c0.m_invInertia), r0) +
                        Cross((Cross(r1, tangent) * c1.m_invInertia), r1)
                    );

                float massTangent = 1.0f / kTangent;


                float vt = Dot(dv, tangent);
                float dPt = massTangent * (-vt);


                float maxPt = g_friction * dPn;
                dPt = Clamp(dPt, -maxPt, maxPt);


                // Apply contact impulse
                float3 P = dPt * tangent;

                c0.m_linVelocity += invMass0 * P;
                c0.m_angVelocity += Cross(r0, P) * c0.m_invInertia;

                c1.m_linVelocity -= invMass1 * P;
                c1.m_angVelocity -= Cross(r1, P) * c1.m_invInertia;
            }
            // TANGENT

        }
    };

}

#endif //CUBE_H




