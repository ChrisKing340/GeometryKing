/***************************************************************************/
/*                                                                         */
/*  File: Cube.cpp                                                         */
/*  Date: 24-10-07                                                         */
/*                                                                         */
/***************************************************************************/
/*
	See .h file
*/
/***************************************************************************/
#include "CK_Cube.h"

namespace King {
    //
    //#include <windows.h>
    //
    //#include <stdio.h>					// sprintf(..)
    //#include <d3dx9.h>					// Direct X Header Files
    //#pragma comment(lib, "d3d9.lib")	// Direct X Library Files	
    //#pragma comment(lib, "d3dx9.lib")


// bool g_startSleeping = false;


//void Cube::Render(IDirect3DDevice9* pDevice)
//{
//	struct TLVERTEX
//	{
//		float x,y,z;
//		float nx,ny,nz;
//		unsigned int colour;
//		enum{ FVF_TLVERTEX = D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_DIFFUSE };
//	};
//
//	pDevice->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
//    pDevice->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
//    pDevice->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
//
//	pDevice->SetRenderState(D3DRS_CULLMODE, D3DCULL_CW);
//	//pDevice->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
//	pDevice->SetRenderState(D3DRS_LIGHTING, TRUE);
//	pDevice->SetTextureStageState(0,D3DTSS_COLOROP,		D3DTOP_SELECTARG1);
//	pDevice->SetTextureStageState(0,D3DTSS_COLORARG1,	D3DTA_DIFFUSE);
//	float w = m_e.x;
//	float h = m_e.y;
//	float d = m_e.z;
//
//	DWORD sleepFade = m_awake?0xffffffff:0xaa888888;
//	DWORD col = m_colour & sleepFade;
//
//	TLVERTEX Vertex[36] = 
//	{
//		//   x     y     z     normal		colour
//		// Front
//		{   -w,   -h,	-d,	   0,0,-1,    col},
//		{   +w,   +h,	-d,    0,0,-1,    col},
//		{   -w,   +h,	-d,    0,0,-1,    col},
//
//		{   -w,   -h,	-d,    0,0,-1,    col},
//		{   +w,   -h,	-d,    0,0,-1,    col},
//		{   +w,   +h,	-d,    0,0,-1,    col},
//
//		// Back
//		{   -w,   -h,	+d,    0,0, 1,    col},
//		{   -w,   +h,	+d,    0,0, 1,    col},
//		{   +w,   +h,	+d,    0,0, 1,    col},
//
//		{   -w,   -h,	+d,    0,0, 1,    col},
//		{   +w,   +h,	+d,    0,0, 1,    col},
//		{   +w,   -h,	+d,    0,0, 1,    col},
//
//		// Top
//		{   -w,   +h,	-d,    0,1,0,     col},
//		{   +w,   +h,	+d,    0,1,0,     col},
//		{   -w,   +h,	+d,    0,1,0,     col},
//
//		{   -w,   +h,	-d,    0,1,0,     col},
//		{   +w,   +h,	-d,    0,1,0,     col},
//		{   +w,   +h,	+d,    0,1,0,     col},
//
//		// Bottom
//		{   -w,   -h,	-d,    0,-1,0,     col},
//		{   -w,   -h,	+d,    0,-1,0,     col},
//		{   +w,   -h,	+d,    0,-1,0,     col},
//							   
//		{   -w,   -h,	-d,    0,-1,0,     col},
//		{   +w,   -h,	+d,    0,-1,0,     col},
//		{   +w,   -h,	-d,    0,-1,0,     col},
//
//		// Left
//		{   -w,   -h,	-d,    -1,0,0,     col},
//		{   -w,   +h,	-d,    -1,0,0,     col},
//		{   -w,   +h,	+d,    -1,0,0,     col},
//
//		{   -w,   -h,	-d,    -1,0,0,     col},
//		{   -w,   +h,	+d,    -1,0,0,     col},
//		{   -w,   -h,	+d,    -1,0,0,     col},
//
//		// Right
//		{   +w,   -h,	-d,    1,0,0,     col},
//		{   +w,   +h,	+d,    1,0,0,     col},
//		{   +w,   +h,	-d,    1,0,0,     col},
//							   
//		{   +w,   -h,	-d,    1,0,0,     col},
//		{   +w,   -h,	+d,    1,0,0,     col},
//		{   +w,   +h,	+d,    1,0,0,     col},
//	};
//	pDevice->SetTexture( 0, NULL);
//
//
//	#if(DIRECT3D_VERSION >= 0x0900)
//	pDevice->SetFVF( TLVERTEX::FVF_TLVERTEX );
//	#else
//	pDevice->SetVertexShader( TLVERTEX::FVF_TLVERTEX );
//	#endif // DIRECT3D_VERSION
//
//
//	pDevice->SetTransform(D3DTS_WORLD, &m_matWorld);
//
//	HRESULT hr = 
//	pDevice->DrawPrimitiveUP( D3DPT_TRIANGLELIST, 12, Vertex, sizeof( TLVERTEX ) );
//
//	pDevice->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
//
//	UpdateMatrix();
//}// End Render(..)



//float Limit(float val, float min, float max)
//{
//	if (val > max)
//	{
//		val = max;
//	}
//	if (val < min)
//	{
//		val = min;
//	}
//	return val;
//}

//D3DXVECTOR3 Limit(D3DXVECTOR3 val, float min, float max)
//{
//	val.x = Limit(val.x, min, max);
//	val.y = Limit(val.y, min, max);
//	val.z = Limit(val.z, min, max);
//	return val;
//}


    void Cube::UpdatePos(float dt)
    {
        if (m_mass > 1000.0f || !m_awake)
        {
            return;
        }

        m_c += (m_linVelocity * dt);

        DirectX::XMFLOAT3 angVel = m_angVelocity;

        quat Qvel = (m_rot * 0.5f) * quat(angVel.x, angVel.y, angVel.z);
        m_rot += Qvel * dt;
        m_rot.MakeNormalize(); // D3DXQuaternionNormalize(&m_rot, &m_rot);


        m_forces = float3(0, 0, 0);
        m_torques = float3(0, 0, 0);


        UpdateMatrix();
    }



    void Cube::UpdateVel(float dt)
    {
        if (m_mass > 1000.0f || !m_awake)
        {
            return;
        }

        if (m_linVelocity.GetMagnitude() < 0.0001f)  m_linVelocity = float3(0, 0, 0);
        if (m_angVelocity.GetMagnitude() < 0.0001f)  m_angVelocity = float3(0, 0, 0);



        // Add Gravity
        auto F = float3(0, -9.8f * m_mass, 0);
        AddForce(m_c, F);


        // Update Angular
        m_angVelocity += ((m_torques * m_invInertia) * dt);

        // Update Linear
        m_linVelocity += ((m_forces / m_mass) * dt);


        // Bit Damping
        const float damping = 0.98f;

        m_linVelocity *= powf(damping, dt);
        m_angVelocity *= powf(damping, dt);



        UpdateMatrix();


        // Update the kinetic energy store, and possibly put the body to sleep.
        // can sleep bool?
        {
            float motion = Dot(m_linVelocity, m_linVelocity) +
                Dot(m_angVelocity, m_angVelocity);

            float bias = 0.96f;
            m_rwaMotion = bias * m_rwaMotion + (1 - bias) * motion;
            if (m_rwaMotion > 50.0f) m_rwaMotion = 5.0f;

            /*
            if (m_rwaMotion < g_sleepEpsilon)
            {
                m_awake = false;
                m_linVelocity = D3DXVECTOR3(0,0,0);
                m_angVelocity = D3DXVECTOR3(0,0,0);
            }
            else if (m_rwaMotion > 10 * g_sleepEpsilon)
            {
                m_rwaMotion = 10 * g_sleepEpsilon;
                m_awake = true;
            }
            */
        }

        {
            //	float vsmall = 0.00001f;
            //	if ( abs(c0.m_linVelocity.x) < vsmall ) c0.m_linVelocity.x=0.0f;
            //	if ( abs(c0.m_linVelocity.y) < vsmall ) c0.m_linVelocity.y=0.0f;
            //	if ( abs(c0.m_linVelocity.z) < vsmall ) c0.m_linVelocity.z=0.0f;

            //	if ( abs(c1.m_linVelocity.x) < vsmall ) c1.m_linVelocity.x=0.0f;
            //	if ( abs(c1.m_linVelocity.y) < vsmall ) c1.m_linVelocity.y=0.0f;
            //	if ( abs(c1.m_linVelocity.z) < vsmall ) c1.m_linVelocity.z=0.0f;

            //	if ( abs(c0.m_angVelocity.x) < vsmall ) c0.m_angVelocity.x=0.0f;
            //	if ( abs(c0.m_angVelocity.y) < vsmall ) c0.m_angVelocity.y=0.0f;
            //	if ( abs(c0.m_angVelocity.z) < vsmall ) c0.m_angVelocity.z=0.0f;

            //	if ( abs(c1.m_angVelocity.x) < vsmall ) c1.m_angVelocity.x=0.0f;
            //	if ( abs(c1.m_angVelocity.y) < vsmall ) c1.m_angVelocity.y=0.0f;
            //	if ( abs(c1.m_angVelocity.z) < vsmall ) c1.m_angVelocity.z=0.0f;



            //	/*
            //	if (D3DXVec3Length(&c0.m_linVelocity)<0.001) c0.m_linVelocity = D3DXVECTOR3(0,0,0);
            //	if (D3DXVec3Length(&c1.m_linVelocity)<0.001) c1.m_linVelocity = D3DXVECTOR3(0,0,0);

            //	if (D3DXVec3Length(&c0.m_angVelocity)<0.001) c0.m_angVelocity = D3DXVECTOR3(0,0,0);
            //	if (D3DXVec3Length(&c1.m_angVelocity)<0.001) c1.m_angVelocity = D3DXVECTOR3(0,0,0);
            //	*/
        }
    }

}


















