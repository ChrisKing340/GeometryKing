/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:			MathSIMD

Description:	Data parallelism accelerated code through implementing SIMD 
				(single instruction, multiple data) math libraries.
				Made easy with operator overloads to behave as a single data
				type.  Uses the DirectXMath library for intrinsics and fall
				back code if not supported on platform.

				This code base was originally written without SIMD. Therefore,
				not all of the multiple data classes (such as int types) have
				been converted.  Float types have been resulting in a dramatic
				decrease in 3D math calculations time in my projects. Releasing
				so other may benefit from it as well without having to learn
				the DirectXMath library.

Usage:			Use the typedef keywords in your applications as a generic
				use of the library.  Most code is inline and won't be included
				in your code if not referenced.  This code is intended to
				be complied for 64 bit operating systems.  It may work on 32 bit
				but assurances must be made that data storage is 16 bit aligned.  
				I have provided for that but must be maintained in inheritence
				if you derive your own classes from this libraries.

				For a less comprehensive but both 32 & 64 bit tested code, use
				the SimpleMath library from the Microsoft DirectX ToolKit
				referenced in the link below.  It uses separate storage variables
				that do not have to be 16 bit aligned, however there is extra 
				processing for load/store each use of the data and fewer operators
				are delevoped than my implementation.  We do provide conversions
				to the storage vaiables from each of our data types making integration
				seemless if you are migrating to our library for increased efficiency
				and functionality.  I will/have release my 2D and 3D geometry
				library and engineering and 3D game code engines which foundation
				is on MathSIMD.  A great start to your own library of code. 

Contact:		ChrisKing340@gmail.com

(c) Copyrighted 2019 Christopher H. King all rights reserved.

References:		https://msdn.microsoft.com/en-us/library/windows/desktop/hh437833(v=vs.85).aspx


MIT License

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

#ifndef __cplusplus
#error MathSIMD requires C++
#endif

#if defined(_MSC_VER) && (_MSC_VER < 1900)
#error MathSIMD requires Visual C++ 2015 or later.
#endif

#include <windows.h>
#include <memory>
#include <vector>
#include <DirectXMath.h>
#include <emmintrin.h> 
#include <functional>
#include <ostream>
#include <istream>
#include <sal.h>

using namespace std;
using namespace DirectX;

namespace King {
	// unique data types built on Single Instruction Multiple Data, SIMD, DirectXMath library of intrinsics for speed and simple implementation
	class UIntPoint2; // not accelerated
	class IntPoint2; // not accelerated
	class IntPoint3; // not accelerated
	class FloatPoint2; // SIMD
	class FloatPoint3; // SIMD
	class FloatPoint4; // SIMD
	class Quaternion; // SIMD

	// type defines to use in our generic code base and easily change out when moving to different libraries
	// for example, commonly used Vector3 could be used below.  I prefer the below similar to HLSL.
	typedef UIntPoint2		uint2;
	typedef IntPoint2		int2;
	typedef IntPoint3		int3;
	typedef FloatPoint2		float2;
	typedef FloatPoint3		float3;
	typedef FloatPoint4		float4;
	typedef Quaternion		quat;
	
	/******************************************************************************
	*	UIntPoint2
	******************************************************************************/
	__declspec(align(16)) class UIntPoint2
	{
		/* variables */
	public:

	protected:

	private:
		unsigned int		u[2];
		/* methods */
	public:
		// Creation/Life cycle
		void * operator new (size_t size) { return _aligned_malloc(size, 16); }
		void   operator delete (void *p) { _aligned_free(static_cast<UIntPoint2*>(p)); }

		inline UIntPoint2() { SetZero(); }
		inline UIntPoint2(const unsigned long &xy) { Set(xy); }
		inline UIntPoint2(const long &xy) { Set(xy); }
		inline UIntPoint2(const unsigned long &x, const unsigned long &y) { Set(x, y); }
		inline UIntPoint2(const unsigned int &x, const unsigned int &y) { Set(x, y); }
		inline UIntPoint2(const long &x, const long &y) { Set(x, y); }
		inline UIntPoint2(const float &x, const float &y) { Set(x, y); }
		inline UIntPoint2(const UIntPoint2 &in) = default; // copy
		inline UIntPoint2(const DirectX::XMUINT2 &in) { Set(in); }
		inline UIntPoint2(const IntPoint2 & in);
		inline UIntPoint2(const FloatPoint2 & in);
		inline UIntPoint2(UIntPoint2 &&in) = default; // move
		inline UIntPoint2(const DirectX::FXMVECTOR &vecIn) { DirectX::XMStoreInt2(u, vecIn); }
		virtual ~UIntPoint2() = default;
		// Operators 
		inline UIntPoint2 & operator= (const UIntPoint2 &in) = default; // copy assignment
		inline UIntPoint2 & operator= (const DirectX::XMUINT2 &in) { Set(in); return *this; } // copy assignment
		inline UIntPoint2 & operator= (const DirectX::XMVECTOR &vecIn) { XMUINT2 d; auto v = DirectX::XMConvertVectorFloatToUInt(vecIn, 0); DirectX::XMStoreUInt2(&d, vecIn); Set(d); return *this; } // untested
		inline UIntPoint2 & operator= (UIntPoint2 &&in) = default; // move assignment
		// Conversions
		inline unsigned int& operator[](int idx) { return GetPtr()[idx]; }
		inline operator DirectX::XMVECTOR() const { DirectX::XMVECTORU32 r; r.u[0] = u[0]; r.u[1] = u[1]; r.u[2] = r.u[3] = 0; return r; }
		inline operator DirectX::XMUINT2() const { return Get_XMUINT2(); }
		inline operator DirectX::XMFLOAT2() const { return Get_XMFLOAT2(); }
		// Math Operators
		inline UIntPoint2 operator+ (const UIntPoint2 p) const { return UIntPoint2(u[0] + p.u[0], u[1] + p.u[1]); }
		inline UIntPoint2 operator- (const UIntPoint2 p) const { return UIntPoint2(u[0] - p.u[0], u[1] - p.u[1]); }
		inline UIntPoint2 operator* (const UIntPoint2 p) const { return UIntPoint2(u[0] * p.u[0], u[1] * p.u[1]); }
		inline UIntPoint2 operator/ (const UIntPoint2 p) const { return UIntPoint2(u[0] / p.u[0], u[1] / p.u[1]); }

		inline UIntPoint2 & operator+= (const unsigned int s) { u[0] = u[0] + s; u[1] = u[1] + s; return *this; }
		inline UIntPoint2 & operator-= (const unsigned int s) { u[0] = u[0] - s; u[1] = u[1] - s; return *this; }
		inline UIntPoint2 & operator*= (const unsigned int s) { u[0] = u[0] * s; u[1] = u[1] * s; return *this; }
		inline UIntPoint2 & operator/= (const unsigned int s) { u[0] = u[0] / s; u[1] = u[1] / s; return *this; }

		inline UIntPoint2 & operator+= (const UIntPoint2 &p) { u[0] = u[0] + p.u[0]; u[1] = u[1] + p.u[1]; return *this; }
		inline UIntPoint2 & operator-= (const UIntPoint2 &p) { u[0] = u[0] - p.u[0]; u[1] = u[1] - p.u[1]; return *this; }
		inline UIntPoint2 & operator*= (const UIntPoint2 &p) { u[0] = u[0] * p.u[0]; u[1] = u[1] * p.u[1]; return *this; }
		inline UIntPoint2 & operator/= (const UIntPoint2 &p) { u[0] = u[0] / p.u[0]; u[1] = u[1] / p.u[1]; return *this; }

		inline UIntPoint2 operator+  (unsigned int s) const { return UIntPoint2(u[0] + static_cast<UINT>(s), u[1] + static_cast<UINT>(s)); }
		inline UIntPoint2 operator-  (unsigned int s) const { return UIntPoint2(u[0] - static_cast<UINT>(s), u[1] - static_cast<UINT>(s)); }
		inline UIntPoint2 operator*  (unsigned int s) const { return UIntPoint2(u[0] * static_cast<UINT>(s), u[1] * static_cast<UINT>(s)); }
		inline UIntPoint2 operator/  (unsigned int s) const { return UIntPoint2(u[0] / static_cast<UINT>(s), u[1] / static_cast<UINT>(s)); }

		inline UIntPoint2 operator+  (unsigned long s) const { return UIntPoint2(u[0] + static_cast<UINT>(s), u[1] + static_cast<UINT>(s)); }
		inline UIntPoint2 operator-  (unsigned long s) const { return UIntPoint2(u[0] - static_cast<UINT>(s), u[1] - static_cast<UINT>(s)); }
		inline UIntPoint2 operator*  (unsigned long s) const { return UIntPoint2(u[0] * static_cast<UINT>(s), u[1] * static_cast<UINT>(s)); }
		inline UIntPoint2 operator/  (unsigned long s) const { return UIntPoint2(u[0] / static_cast<UINT>(s), u[1] / static_cast<UINT>(s)); }

		inline UIntPoint2 operator+  (float s) const { return UIntPoint2(static_cast<float>(u[0]) + (s), static_cast<float>(u[1]) + (s)); }
		inline UIntPoint2 operator-  (float s) const { return UIntPoint2(static_cast<float>(u[0]) - (s), static_cast<float>(u[1]) - (s)); }
		inline UIntPoint2 operator*  (float s) const { return UIntPoint2(static_cast<float>(u[0]) * (s), static_cast<float>(u[1]) * (s)); }
		inline UIntPoint2 operator/  (float s) const { return UIntPoint2(static_cast<float>(u[0]) / (s), static_cast<float>(u[1]) / (s)); }

		// Assignments
		inline void								SetZero(void) { u[0] = u[1] = 0; }
		inline void								SetX(const unsigned long x) { u[0] = static_cast<unsigned int>(x); }
		inline void								SetY(const unsigned long y) { u[1] = static_cast<unsigned int>(y); }
		inline void								Set(const unsigned long xy) { u[0] = u[1] = static_cast<unsigned int>(xy); }
		inline void								Set(const long xy) { u[0] = u[1] = static_cast<unsigned int>(xy); }
		inline void								Set(const float xy) { u[0] = u[1] = static_cast<unsigned int>(xy); }
		inline void								Set(const unsigned long x, const unsigned long y) { u[0] = x; u[1] = y; }
		inline void								Set(const unsigned int x, const unsigned int y) { u[0] = static_cast<unsigned int>(x); u[1] = static_cast<unsigned int>(y); }
		inline void								Set(const long x, const long y) { u[0] = static_cast<unsigned int>(x); u[1] = static_cast<unsigned int>(y); }
		inline void								Set(const float x, const float y) { u[0] = static_cast<unsigned int>(x); u[1] = static_cast<unsigned int>(y); }
		inline void								Set(const UIntPoint2 &in) { *this = in; }
		inline void								Set(const DirectX::XMUINT2 &point) { u[0] = point.x; u[1] = point.y; }
		inline void								Set(const POINT &point) { u[0] = static_cast<unsigned int>(point.x); u[1] = static_cast<unsigned int>(point.y); }
		// Accessors
		unsigned int*							GetPtr() { return reinterpret_cast<unsigned int*>(this); }
		inline const POINT						Get_POINT() const { POINT pt = { static_cast<long>(u[0]), static_cast<long>(u[1]) }; return pt; }
		inline const DirectX::XMINT2			Get_XMINT2() const { DirectX::XMINT2 rtn; rtn = { static_cast<int>(u[0]), static_cast<int>(u[1]) }; return rtn; }
		inline const DirectX::XMUINT2			Get_XMUINT2() const { DirectX::XMUINT2 rtn; rtn = { static_cast<unsigned int>(u[0]), static_cast<unsigned int>(u[1]) }; return rtn; }
		inline const DirectX::XMFLOAT2			Get_XMFLOAT2() const { DirectX::XMFLOAT2 rtn; rtn = { static_cast<float>(u[0]), static_cast<float>(u[1]) }; return rtn; }
		inline const unsigned long				GetX() const { return (unsigned long)u[0]; }
		inline const unsigned long				GetY() const { return (unsigned long)u[1]; }
		// Functionality
		inline void 							Min(const UIntPoint2 & in) { u[0] = u[0] < in.u[0] ? u[0] : in.u[1]; u[1] = u[1] < in.u[1] ? u[1] : in.u[1]; }
		inline void 							Max(const UIntPoint2 & in) { u[0] = u[0] > in.u[0] ? u[0] : in.u[1]; u[1] = u[1] > in.u[1] ? u[1] : in.u[1]; }
		//Statics
		static const float						Magnitude(const UIntPoint2 &pIn) { return (float)std::sqrt(pIn.u[0] * pIn.u[0] + pIn.u[1] * pIn.u[1]); }
	};
	/******************************************************************************
	*	IntPoint2
	******************************************************************************/
	__declspec(align(16)) class IntPoint2
	{
		/* variables */
	public:

	protected:

	private:
		int			i[2];
		/* methods */
	public:
		// Creation/Life cycle
		void * operator new (size_t size) { return _aligned_malloc(size, 16); }
		void   operator delete (void *p) { _aligned_free(static_cast<IntPoint2*>(p)); }

		inline IntPoint2() { SetZero(); }
		inline IntPoint2(const long xy) { Set(xy); }
		inline IntPoint2(const long x, const long y) { Set(x, y); }
		inline IntPoint2(const int x, const int y) { Set(x, y); }
		inline IntPoint2(const unsigned int x, const unsigned int y) { Set(x, y); }
		inline IntPoint2(const float x, const float y) { Set(x, y); }
		inline IntPoint2(const unsigned long x, const unsigned long y) { Set(x, y); }
		inline IntPoint2(const IntPoint2 &in) = default; // copy
		inline IntPoint2(const UIntPoint2 &in) { Set(in); }
		inline IntPoint2(const FloatPoint2 &in);
		inline IntPoint2(const POINT &in) { Set(in); }
		inline IntPoint2(const DirectX::XMINT2 &in) { Set(in); }
		inline IntPoint2(IntPoint2 &&in) = default; // move
		inline explicit IntPoint2(const DirectX::XMVECTOR vecIn) { DirectX::XMINT2 t; DirectX::XMStoreSInt2(&t, vecIn); i[0] = t.x; i[1] = t.y; } // integer vector input NOT float
		virtual ~IntPoint2() = default;
		// Operators 
		inline IntPoint2 & operator= (const IntPoint2 &in) = default; // copy assignment
		inline IntPoint2 & operator= (const DirectX::XMINT2 &in) { Set(in); return *this; } // copy assignment
		inline IntPoint2 & operator= (const DirectX::XMVECTOR &vecIn) { unsigned int d[2]; auto v = DirectX::XMConvertVectorFloatToInt(vecIn, 0); DirectX::XMStoreInt2(d, vecIn); i[0]=(long)d[0]; i[1] = (long)d[1]; return *this; } // untested
		inline IntPoint2 & operator= (IntPoint2 &&in) = default; // move assignment
		// Conversions
		inline int& operator[](int idx) { return GetPtr()[idx]; }
		inline operator DirectX::XMVECTOR() const { DirectX::XMVECTORI32 r; r.i[0] = i[0]; r.i[1] = i[1]; r.i[2] = r.i[3] = 0; return r.v; }
		inline operator DirectX::XMINT2() const { return Get_XMINT2(); }
		inline operator DirectX::XMFLOAT2() const { return Get_XMFLOAT2(); }
		// Math Operators
		inline IntPoint2 operator- () const { return IntPoint2(-1 * i[0], -1 * i[1]); }
		inline IntPoint2 operator+ (const IntPoint2 p) const { return IntPoint2(i[0] + p.i[0], i[1] + p.i[1]); }
		inline IntPoint2 operator- (const IntPoint2 p) const { return IntPoint2(i[0] - p.i[0], i[1] - p.i[1]); }
		inline IntPoint2 operator* (const IntPoint2 p) const { return IntPoint2(i[0] * p.i[0], i[1] * p.i[1]); }
		inline IntPoint2 operator/ (const IntPoint2 p) const { return IntPoint2(i[0] / p.i[0], i[1] / p.i[1]); }

		inline IntPoint2 & operator+= (const int s) { i[0] = i[0] + s; i[1] = i[1] + s; return *this; }
		inline IntPoint2 & operator-= (const int s) { i[0] = i[0] - s; i[1] = i[1] - s; return *this; }
		inline IntPoint2 & operator*= (const int s) { i[0] = i[0] * s; i[1] = i[1] * s; return *this; }
		inline IntPoint2 & operator/= (const int s) { i[0] = i[0] / s; i[1] = i[1] / s; return *this; }

		inline IntPoint2 & operator+= (const IntPoint2 &p) { i[0] = i[0] + p.i[0]; i[1] = i[1] + p.i[1];; return *this; }
		inline IntPoint2 & operator-= (const IntPoint2 &p) { i[0] = i[0] - p.i[0]; i[1] = i[1] - p.i[1];; return *this; }
		inline IntPoint2 & operator*= (const IntPoint2 &p) { i[0] = i[0] * p.i[0]; i[1] = i[1] * p.i[1];; return *this; }
		inline IntPoint2 & operator/= (const IntPoint2 &p) { i[0] = i[0] / p.i[0]; i[1] = i[1] / p.i[1];; return *this; }

		inline IntPoint2 operator+  (int s) const { return IntPoint2(i[0] + static_cast<long>(s), i[1] + static_cast<long>(s)); }
		inline IntPoint2 operator-  (int s) const { return IntPoint2(i[0] - static_cast<long>(s), i[1] - static_cast<long>(s)); }
		inline IntPoint2 operator*  (int s) const { return IntPoint2(i[0] * static_cast<long>(s), i[1] * static_cast<long>(s)); }
		inline IntPoint2 operator/  (int s) const { return IntPoint2(i[0] / static_cast<long>(s), i[1] / static_cast<long>(s)); }

		inline IntPoint2 operator+  (long s) const { return IntPoint2(i[0] + (s), i[1] + (s)); }
		inline IntPoint2 operator-  (long s) const { return IntPoint2(i[0] - (s), i[1] - (s)); }
		inline IntPoint2 operator*  (long s) const { return IntPoint2(i[0] * (s), i[1] * (s)); }
		inline IntPoint2 operator/  (long s) const { return IntPoint2(i[0] / (s), i[1] / (s)); }

		inline IntPoint2 operator+  (float s) const { return IntPoint2(static_cast<float>(i[0]) + (s), static_cast<float>(i[1]) + (s)); }
		inline IntPoint2 operator-  (float s) const { return IntPoint2(static_cast<float>(i[0]) - (s), static_cast<float>(i[1]) - (s)); }
		inline IntPoint2 operator*  (float s) const { return IntPoint2(static_cast<float>(i[0]) * (s), static_cast<float>(i[1]) * (s)); }
		inline IntPoint2 operator/  (float s) const { return IntPoint2(static_cast<float>(i[0]) / (s), static_cast<float>(i[1]) / (s)); }

		inline IntPoint2 & operator+= (const float s) { i[0] = static_cast<int>(static_cast<float>(i[0]) + s); i[1] = static_cast<int>(static_cast<float>(i[1]) + s); return *this; }
		inline IntPoint2 & operator-= (const float s) { i[0] = static_cast<int>(static_cast<float>(i[0]) - s); i[1] = static_cast<int>(static_cast<float>(i[1]) - s); return *this; }
		inline IntPoint2 & operator*= (const float s) { i[0] = static_cast<int>(static_cast<float>(i[0]) * s); i[1] = static_cast<int>(static_cast<float>(i[1]) * s); return *this; }
		inline IntPoint2 & operator/= (const float s) { i[0] = static_cast<int>(static_cast<float>(i[0]) / s); i[1] = static_cast<int>(static_cast<float>(i[1]) / s); return *this; }

		// Assignments
		inline void								SetZero(void) { i[0] = i[1] = 0; }
		inline void								SetX(const long x) { i[0] = static_cast<int>(x); }
		inline void								SetY(const long y) { i[1] = static_cast<int>(y); }
		inline void								Set(const long xy) { i[0] = i[1] = static_cast<int>(xy); }
		inline void								Set(const float xy) { i[0] = i[1] = static_cast<int>(xy); }
		inline void								Set(const long x, const long y) { i[0] = x; i[1] = y; }
		inline void								Set(const int x, const int y) { i[0] = x; i[1] = y; }
		inline void								Set(const unsigned int x, const unsigned int y) { i[0] = static_cast<int>(x); i[1] = static_cast<int>(y); }
		inline void								Set(const float x, const float y) { i[0] = static_cast<int>(x); i[1] = static_cast<int>(y); }
		inline void								Set(const unsigned long x, const unsigned long y) { i[0] = static_cast<int>(x); i[1] = static_cast<int>(y); }
		inline void								Set(const IntPoint2 &in) { *this = in; }
		inline void								Set(const UIntPoint2 &in) { i[0] = static_cast<int>(in.GetX()); i[1] = static_cast<int>(in.GetY()); }
		inline void								Set(const DirectX::XMINT2 &point) { i[0] = point.x; i[1] = point.y; }
		inline void								Set(const POINT &point) { i[0] = static_cast<int>(point.x); i[1] = static_cast<int>(point.y); }
		// Accessors
		int*									GetPtr() { return reinterpret_cast<int*>(this); }
		inline const POINT						Get_POINT() const { POINT pt = { static_cast<long>(i[0]), static_cast<long>(i[1]) }; return pt; }
		inline const DirectX::XMINT2			Get_XMINT2() const { DirectX::XMINT2 rtn; rtn = { static_cast<int>(i[0]), static_cast<int>(i[1]) }; return rtn; }
		inline const DirectX::XMUINT2			Get_XMUINT2() const { DirectX::XMUINT2 rtn; rtn = { static_cast<unsigned int>(i[0]), static_cast<unsigned int>(i[1]) }; return rtn; }
		inline const DirectX::XMFLOAT2			Get_XMFLOAT2() const { DirectX::XMFLOAT2 rtn; rtn = { static_cast<float>(i[0]), static_cast<float>(i[1]) }; return rtn; }
		inline const int						GetX() const { return (int)i[0]; }
		inline const int						GetY() const { return (int)i[1]; }
		// Functionality
		inline void 							Min(const IntPoint2 & in) { i[0] = i[0] < in.i[0] ? i[0] : in.i[0]; i[1] = i[1] < in.i[1] ? i[1] : in.i[1]; }
		inline void 							Max(const IntPoint2 & in) { i[0] = i[0] > in.i[0] ? i[0] : in.i[0]; i[1] = i[1] > in.i[1] ? i[1] : in.i[1]; }
		inline void								MakeAbsolute() { i[0] = std::abs(i[0]); i[1] = std::abs(i[1]); }
		// Statics
		static const float						Magnitude(const IntPoint2 &pIn) { return (float)std::sqrt(pIn.i[0] * pIn.i[0] + pIn.i[1] * pIn.i[1]); }
	};
	/******************************************************************************
	*	IntPoint3
	******************************************************************************/
	__declspec(align(16)) class IntPoint3
	{
		/* variables */
	public:

	protected:

	private:
		int			i[3];
		/* methods */
	public:
		// Creation/Life cycle
		void * operator new (size_t size) { return _aligned_malloc(size, 16); }
		void   operator delete (void *p) { _aligned_free(static_cast<IntPoint3*>(p)); }

		inline IntPoint3() { SetZero(); }
		inline IntPoint3(const long xyz) { Set(xyz); }
		inline IntPoint3(const long x, const long y, const long z) { Set(x, y, z); }
		inline IntPoint3(const int x, const int y, const int z) { Set(x, y, z); }
		inline IntPoint3(const unsigned int x, const unsigned int y, const unsigned int z) { Set(x, y, z); }
		inline IntPoint3(const float x, const float y, const float z) { Set(x, y, z); }
		inline IntPoint3(const unsigned long x, const unsigned long y, const unsigned long z) { Set(x, y, z); }
		inline IntPoint3(const IntPoint3 &in) = default; // copy
		inline IntPoint3(const FloatPoint3 &in);
		inline IntPoint3(const DirectX::XMINT3 &in) { Set(in); }
		inline IntPoint3(IntPoint3 &&in) = default; // move 
		inline explicit IntPoint3(const DirectX::XMVECTOR vecIn) { DirectX::XMINT3 t; DirectX::XMStoreSInt3(&t, vecIn); i[0] = t.x; i[1] = t.y; i[2] = t.z; } // integer vector input NOT float
		virtual ~IntPoint3() = default;
		// Operators 
		inline IntPoint3 & operator= (const IntPoint3 &in) = default; // copy assignment
		inline IntPoint3 & operator= (const DirectX::XMINT3 &in) { Set(in); return *this; } // copy assignment
		inline IntPoint3 & operator= (const DirectX::XMVECTOR &vecIn) { unsigned int d[3]; auto v = DirectX::XMConvertVectorFloatToInt(vecIn, 0); DirectX::XMStoreInt3(d, vecIn); i[0] = (long)d[0]; i[1] = (long)d[1]; i[2] = (long)d[2]; return *this; } // untested
		inline IntPoint3 & operator= (IntPoint3 &&in) = default; // move assignment
		// Conversions
		inline int& operator[](int idx) { return GetPtr()[idx]; }
		inline operator DirectX::XMVECTOR() const { DirectX::XMVECTORI32 r; r.i[0] = i[0]; r.i[1] = i[1]; r.i[2] = i[2]; r.i[3] = 0; return r.v; }
		inline operator DirectX::XMINT3() const { return Get_XMINT3(); }
		inline operator DirectX::XMFLOAT3() const { return Get_XMFLOAT3(); }
		// Math Operators
		inline IntPoint3 operator- () const { return IntPoint3(-1 * i[0], -1 * i[1], -1 * i[2]); }
		inline IntPoint3 operator+ (const IntPoint3 p) const { return IntPoint3(i[0] + p.i[0], i[1] + p.i[1], i[2] + p.i[2]); }
		inline IntPoint3 operator- (const IntPoint3 p) const { return IntPoint3(i[0] - p.i[0], i[1] - p.i[1], i[2] - p.i[2]); }
		inline IntPoint3 operator* (const IntPoint3 p) const { return IntPoint3(i[0] * p.i[0], i[1] * p.i[1], i[2] * p.i[2]); }
		inline IntPoint3 operator/ (const IntPoint3 p) const { return IntPoint3(i[0] / p.i[0], i[1] / p.i[1], i[2] / p.i[2]); }

		inline IntPoint3 & operator+= (const int s) { i[0] = i[0] + s; i[1] = i[1] + s; i[2] = i[2] + s; return *this; }
		inline IntPoint3 & operator-= (const int s) { i[0] = i[0] - s; i[1] = i[1] - s; i[2] = i[2] - s; return *this; }
		inline IntPoint3 & operator*= (const int s) { i[0] = i[0] * s; i[1] = i[1] * s; i[2] = i[2] * s; return *this; }
		inline IntPoint3 & operator/= (const int s) { i[0] = i[0] / s; i[1] = i[1] / s; i[2] = i[2] / s; return *this; }

		inline IntPoint3 & operator+= (const IntPoint3 &p) { i[0] = i[0] + p.i[0]; i[1] = i[1] + p.i[1]; i[2] = i[2] + p.i[2]; return *this; }
		inline IntPoint3 & operator-= (const IntPoint3 &p) { i[0] = i[0] - p.i[0]; i[1] = i[1] - p.i[1]; i[2] = i[2] - p.i[2]; return *this; }
		inline IntPoint3 & operator*= (const IntPoint3 &p) { i[0] = i[0] * p.i[0]; i[1] = i[1] * p.i[1]; i[2] = i[2] * p.i[2]; return *this; }
		inline IntPoint3 & operator/= (const IntPoint3 &p) { i[0] = i[0] / p.i[0]; i[1] = i[1] / p.i[1]; i[2] = i[2] / p.i[2]; return *this; }

		inline IntPoint3 operator+  (int s) const { return IntPoint3(i[0] + static_cast<long>(s), i[1] + static_cast<long>(s), i[2] + static_cast<long>(s)); }
		inline IntPoint3 operator-  (int s) const { return IntPoint3(i[0] - static_cast<long>(s), i[1] - static_cast<long>(s), i[2] - static_cast<long>(s)); }
		inline IntPoint3 operator*  (int s) const { return IntPoint3(i[0] * static_cast<long>(s), i[1] * static_cast<long>(s), i[2] * static_cast<long>(s)); }
		inline IntPoint3 operator/  (int s) const { return IntPoint3(i[0] / static_cast<long>(s), i[1] / static_cast<long>(s), i[2] / static_cast<long>(s)); }

		inline IntPoint3 operator+  (long s) const { return IntPoint3(i[0] + (s), i[1] + (s), i[2] + (s)); }
		inline IntPoint3 operator-  (long s) const { return IntPoint3(i[0] - (s), i[1] - (s), i[2] - (s)); }
		inline IntPoint3 operator*  (long s) const { return IntPoint3(i[0] * (s), i[1] * (s), i[2] * (s)); }
		inline IntPoint3 operator/  (long s) const { return IntPoint3(i[0] / (s), i[1] / (s), i[2] / (s)); }

		inline IntPoint3 operator+  (float s) const { return IntPoint3(static_cast<float>(i[0]) + (s), static_cast<float>(i[1]) + (s), static_cast<float>(i[2]) + (s)); }
		inline IntPoint3 operator-  (float s) const { return IntPoint3(static_cast<float>(i[0]) - (s), static_cast<float>(i[1]) - (s), static_cast<float>(i[2]) - (s)); }
		inline IntPoint3 operator*  (float s) const { return IntPoint3(static_cast<float>(i[0]) * (s), static_cast<float>(i[1]) * (s), static_cast<float>(i[2]) * (s)); }
		inline IntPoint3 operator/  (float s) const { return IntPoint3(static_cast<float>(i[0]) / (s), static_cast<float>(i[1]) / (s), static_cast<float>(i[2]) / (s)); }

		inline IntPoint3 & operator+= (const float s) { i[0] = static_cast<int>(static_cast<float>(i[0]) + s); i[1] = static_cast<int>(static_cast<float>(i[1]) + s); i[2] = static_cast<int>(static_cast<float>(i[2]) + s); return *this; }
		inline IntPoint3 & operator-= (const float s) { i[0] = static_cast<int>(static_cast<float>(i[0]) - s); i[1] = static_cast<int>(static_cast<float>(i[1]) - s); i[2] = static_cast<int>(static_cast<float>(i[2]) - s); return *this; }
		inline IntPoint3 & operator*= (const float s) { i[0] = static_cast<int>(static_cast<float>(i[0]) * s); i[1] = static_cast<int>(static_cast<float>(i[1]) * s); i[2] = static_cast<int>(static_cast<float>(i[2]) * s); return *this; }
		inline IntPoint3 & operator/= (const float s) { i[0] = static_cast<int>(static_cast<float>(i[0]) / s); i[1] = static_cast<int>(static_cast<float>(i[1]) / s); i[2] = static_cast<int>(static_cast<float>(i[2]) / s); return *this; }
		// Assignments
		inline void								SetZero(void) { i[0] = i[1] = i[2] = 0; }
		inline void								SetX(const long x) { i[0] = static_cast<int>(x); }
		inline void								SetY(const long y) { i[1] = static_cast<int>(y); }
		inline void								SetZ(const long z) { i[2] = static_cast<int>(z); }
		inline void								Set(const long xyz) { i[0] = i[1] = i[2] = static_cast<int>(xyz); }
		inline void								Set(const float xyz) { i[0] = i[1] = i[2] = static_cast<int>(xyz); }
		inline void								Set(const long x, const long y, const long z) { i[0] = static_cast<int>(x); i[1] = static_cast<int>(y); i[2] = static_cast<int>(z); }
		inline void								Set(const int x, const int y, const int z) { i[0] = x; i[1] = y; i[2] = z; }
		inline void								Set(const unsigned int x, const unsigned int y, const unsigned int z) { i[0] = static_cast<int>(x); i[1] = static_cast<int>(y); i[2] = static_cast<int>(z); }
		inline void								Set(const float x, const float y, const float z) { i[0] = static_cast<int>(x); i[1] = static_cast<int>(y); i[2] = static_cast<int>(z); }
		inline void								Set(const unsigned long x, const unsigned long y, const unsigned long z) { i[0] = static_cast<int>(x); i[1] = static_cast<int>(y); i[2] = static_cast<int>(z); }
		inline void								Set(const IntPoint3 &in) { *this = in; }
//		inline void								Set(const UIntPoint3 &in) { i[0] = static_cast<int>(in.u[0]); i[1] = static_cast<int>(in.u[1]); i[2] = static_cast<int>(in.u[2]); }
		inline void								Set(const DirectX::XMINT3 &point) { i[0] = point.x; i[1] = point.y; i[2] = point.z;  }
		// Accessors
		int*									GetPtr() { return reinterpret_cast<int*>(this); }
		inline const DirectX::XMINT3			Get_XMINT3() const { DirectX::XMINT3 rtn; rtn = { static_cast<int>(i[0]), static_cast<int>(i[1]), static_cast<int>(i[2]) }; return rtn; }
		inline const DirectX::XMUINT3			Get_XMUINT3() const { DirectX::XMUINT3 rtn; rtn = { static_cast<unsigned int>(i[0]), static_cast<unsigned int>(i[1]), static_cast<unsigned int>(i[2]) }; return rtn; }
		inline const DirectX::XMFLOAT3			Get_XMFLOAT3() const { DirectX::XMFLOAT3 rtn; rtn = { static_cast<float>(i[0]), static_cast<float>(i[1]), static_cast<float>(i[2]) }; return rtn; }
		inline const int						GetX() const { return (int)i[0]; }
		inline const int						GetY() const { return (int)i[1]; }
		inline const int						GetZ() const { return (int)i[2]; }
		// Functionality
		inline void 							Max(const IntPoint3 & in) { i[0] = i[0] > in.i[0] ? i[0] : in.i[0]; i[1] = i[1] > in.i[1] ? i[1] : in.i[1]; i[2] = i[2] > in.i[2] ? i[2] : in.i[2]; }
		inline void 							Min(const IntPoint3 & in) { i[0] = i[0] < in.i[0] ? i[0] : in.i[0]; i[1] = i[1] < in.i[1] ? i[1] : in.i[1]; i[2] = i[2] < in.i[2] ? i[2] : in.i[2]; }
		inline void								MakeAbsolute() { i[0] = std::abs(i[0]); i[1] = std::abs(i[1]); i[2] = std::abs(i[2]); }
		// Statics
		static const float						Magnitude(const IntPoint3 &pIn) { return (float)std::sqrt(pIn.i[0] * pIn.i[0] + pIn.i[1] * pIn.i[1] + pIn.i[2] * pIn.i[2]); }
	};
	/******************************************************************************
	*	FloatPoint2
	******************************************************************************/
	__declspec(align(16)) class FloatPoint2 : public XMVECTORF32
	{
		/* variables */
	public:

	protected:

	private:

		/* methods */
	public:
		// Creation/Life cycle
		static std::shared_ptr<FloatPoint2>	Create() { return std::make_shared<FloatPoint2>(); }
		static std::unique_ptr<FloatPoint2>	CreateUnique() { return std::make_unique<FloatPoint2>(); }
		inline FloatPoint2() { SetZero(); }
		inline FloatPoint2(const float xy) { Set(xy); }
		inline FloatPoint2(const float x, const float y) { Set(x, y); }
		inline FloatPoint2(const int x, const int y) { Set(x, y); }
		inline FloatPoint2(const unsigned int x, const unsigned int y) { Set(x, y); }
		inline FloatPoint2(const long x, const long y) { Set(x, y); }
		inline FloatPoint2(const unsigned long x, unsigned const long y) { Set(x, y); }
		inline FloatPoint2(const FloatPoint2 &in) { v = in.v; } // copy
		inline FloatPoint2(const DirectX::XMFLOAT2 &in) { Set(in); }
		inline FloatPoint2(const DirectX::XMINT2 &in) { Set(in); }
		inline FloatPoint2(const IntPoint2 &in) { Set(in.Get_XMFLOAT2()); }
		inline FloatPoint2(const UIntPoint2 &in) { Set(in.Get_XMFLOAT2()); }
		inline FloatPoint2(FloatPoint2 &&in) { v = std::move(in.v); } // move
		inline FloatPoint2(const DirectX::XMVECTOR &vecIn) { v = vecIn; }
		inline FloatPoint2(uint8_t *bytesIn) { auto fp = reinterpret_cast<DirectX::XMFLOAT2*>(bytesIn); v = DirectX::XMLoadFloat2(fp); }
		inline FloatPoint2(float *floatIn) { auto fp = reinterpret_cast<DirectX::XMFLOAT2*>(floatIn); v = DirectX::XMLoadFloat2(fp); }
		virtual ~FloatPoint2() = default;
		// Operators 
		inline FloatPoint2 & operator= (const FloatPoint2 &in) = default; // copy assignment
		inline FloatPoint2 & operator= (const DirectX::XMFLOAT2 &in) { Set(in); return *this; } // copy assignment
		inline FloatPoint2 & operator= (const DirectX::XMVECTOR &vecIn) { v = vecIn; return *this; }
		inline FloatPoint2 & operator= (FloatPoint2 &&in) = default; // move assignment
		// Conversions
		inline operator DirectX::XMFLOAT2() const { return Get_XMFLOAT2(); }
		inline operator DirectX::XMFLOAT2A() const { return Get_XMFLOAT2A(); }
		inline operator DirectX::XMINT2() const { return Get_XMINT2(); }
		inline operator DirectX::XMUINT2() const { return Get_XMUINT2(); }
		// Math Operators
		inline FloatPoint2 operator- () { return FloatPoint2(DirectX::XMVectorNegate(v)); }
		inline FloatPoint2 operator+ (const FloatPoint2 &p) const { return FloatPoint2(DirectX::XMVectorAdd(v, p.v)); }
		inline FloatPoint2 operator- (const FloatPoint2 &p) const { return FloatPoint2(DirectX::XMVectorSubtract(v, p.v)); }
		inline FloatPoint2 operator* (const FloatPoint2 &p) const { return FloatPoint2(DirectX::XMVectorMultiply(v, p.v)); }
		inline FloatPoint2 operator/ (const FloatPoint2 &p) const { return FloatPoint2(DirectX::XMVectorDivide(v, p.v)); }
		inline XMVECTOR & operator+= (const FloatPoint2 &p) { return v = DirectX::XMVectorAdd(v, p.v); }
		inline XMVECTOR & operator-= (const FloatPoint2 &p) { return v = DirectX::XMVectorSubtract(v, p.v); }
		inline XMVECTOR & operator*= (const FloatPoint2 &p) { return v = DirectX::XMVectorMultiply(v, p.v); }
		inline XMVECTOR & operator/= (const FloatPoint2 &p) { return v = DirectX::XMVectorDivide(v, p.v); }
		inline FloatPoint2 & operator+= (const XMVECTOR &vecIn) { v = DirectX::XMVectorAdd(v, vecIn); return *this; }
		inline FloatPoint2 & operator-= (const XMVECTOR &vecIn) { v = DirectX::XMVectorSubtract(v, vecIn); return *this; }
		inline FloatPoint2 & operator*= (const XMVECTOR &vecIn) { v = DirectX::XMVectorMultiply(v, vecIn); return *this; }
		inline FloatPoint2 & operator/= (const XMVECTOR &vecIn) { v = DirectX::XMVectorDivide(v, vecIn); return *this; }
		inline FloatPoint2 operator+ (float s) const { return FloatPoint2(DirectX::XMVectorAdd(v, FloatPoint2(s))); }
		inline FloatPoint2 operator- (float s) const { return FloatPoint2(DirectX::XMVectorSubtract(v, FloatPoint2(s))); }
		inline FloatPoint2 operator* (float s) const { return FloatPoint2(DirectX::XMVectorMultiply(v, FloatPoint2(s))); }
		inline FloatPoint2 operator/ (float s) const { return FloatPoint2(DirectX::XMVectorDivide(v, FloatPoint2(s))); }
		inline XMVECTOR & operator+= (float s) { return v = DirectX::XMVectorAdd(v, FloatPoint2(s)); }
		inline XMVECTOR & operator-= (float s) { return v = DirectX::XMVectorSubtract(v, FloatPoint2(s)); }
		inline XMVECTOR & operator*= (float s) { return v = DirectX::XMVectorScale(v, s); }
		inline XMVECTOR & operator/= (float s) { return v = DirectX::XMVectorDivide(v, FloatPoint2(s)); }
		// Comparators
		inline bool operator<  (const FloatPoint2 &rhs) { return DirectX::XMVector2Less(v, rhs); }
		inline bool operator<= (const FloatPoint2 &rhs) { return DirectX::XMVector2LessOrEqual(v, rhs); }
		inline bool operator>  (const FloatPoint2 &rhs) { return DirectX::XMVector2Greater(v, rhs); }
		inline bool operator>= (const FloatPoint2 &rhs) { return DirectX::XMVector2GreaterOrEqual(v, rhs); }
		inline bool operator== (const FloatPoint2 &rhs) { return DirectX::XMVector2Equal(v, rhs); }
		inline bool operator!= (const FloatPoint2 &rhs) { return DirectX::XMVector2NotEqual(v, rhs); }
		// Accessors
		float*									GetPtr() { return reinterpret_cast<float*>(this); } // returns a float
		inline const DirectX::XMFLOAT2			Get_XMFLOAT2() const { DirectX::XMFLOAT2 rtn; DirectX::XMStoreFloat2(&rtn, v); return rtn; }
		inline const DirectX::XMFLOAT2A			Get_XMFLOAT2A() const { DirectX::XMFLOAT2A rtn; DirectX::XMStoreFloat2A(&rtn, v); return rtn; }
		inline const DirectX::XMINT2			Get_XMINT2() const { DirectX::XMINT2 rtn; rtn.x = static_cast<int>(f[0]); rtn.y = static_cast<int>(f[1]); return rtn; }
		inline const DirectX::XMUINT2			Get_XMUINT2() const { DirectX::XMUINT2 rtn; rtn.x = static_cast<unsigned int>(f[0]); rtn.y = static_cast<unsigned int>(f[1]); return rtn; }
		inline const float						GetX() const { return (float)DirectX::XMVectorGetX(v); }
		inline const float						GetY() const { return (float)DirectX::XMVectorGetY(v); }
		inline DirectX::XMVECTOR &				GetVec() { return v; } // modifiable type
		inline const DirectX::XMVECTOR &		GetVecConst() const { return v; } // constant type
		// Assignments
		inline void								SetZero(void) { v = DirectX::XMVectorZero(); }
		void									SetX(const float x) { v = DirectX::XMVectorSetX(v, x); }
		void									SetY(const float y) { v = DirectX::XMVectorSetY(v, y); }
		inline virtual void						Set(const float xy) { v = DirectX::XMVectorSet(xy, xy, 0.f, 0.f); }
		inline void								Set(const float x, const float y) { v = DirectX::XMVectorSet(x, y, 0.f, 0.f); }
		inline void								Set(const int x, const int y) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), 0.f, 0.f); }
		inline void								Set(const unsigned int x, const unsigned int y) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), 0.f, 0.f); }
		inline void								Set(const long x, const long y) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), 0.f, 0.f); }
		inline void								Set(const unsigned long x, const unsigned long y) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), 0.f, 0.f); }
		inline void								Set(const IntPoint2 &in) { Set(in.Get_XMFLOAT2()); }
		inline void								Set(const UIntPoint2 &in) { Set(in.Get_XMFLOAT2()); }
		inline void								Set(const DirectX::XMFLOAT2 &point) { v = DirectX::XMLoadFloat2(&point); }
		inline void								Set(const DirectX::XMUINT2 &point) { v = DirectX::XMLoadUInt2(&point); v = DirectX::XMConvertVectorUIntToFloat(v, 0); }
		inline void								Set(const DirectX::XMINT2 &point) { v = DirectX::XMLoadSInt2(&point); v = DirectX::XMConvertVectorIntToFloat(v, 0); }
		// Functionality
		void									MakeAbsolute() { v = DirectX::XMVectorAbs(v); }
		inline virtual void						MakeNormalize() { v = DirectX::XMVector2Normalize(v); }
		// Statics
		static FloatPoint2 __vectorcall			Absolute(const FloatPoint2 &point2In) { return FloatPoint2(DirectX::XMVectorAbs(point2In)); }
		static FloatPoint2 __vectorcall			Normal(const FloatPoint2 &point2In) { return FloatPoint2(DirectX::XMVector2Normalize(point2In)); }
		static const float __vectorcall			Magnitude(const FloatPoint2 &point2In) { return DirectX::XMVectorGetX(DirectX::XMVector2Length(point2In)); }
		static FloatPoint2 __vectorcall			DotProduct(const FloatPoint2 &vec1In, const FloatPoint2 &vec2In) { return DirectX::XMVector2Dot(vec1In, vec2In); } // order does not mater A•B = B•A
		static FloatPoint2 __vectorcall			CrossProduct(const FloatPoint2 &vec1In, const FloatPoint2 &vec2In) { return DirectX::XMVector2Cross(vec1In, vec2In); } // order does mater AxB = -(BxA)
		static float __vectorcall				SumComponents(const FloatPoint2 &vec1In) { return DirectX::XMVectorGetX(DirectX::XMVectorSum(vec1In)); }
		static FloatPoint2 __vectorcall			MultiplyAdd(const FloatPoint2 &vec1MulIn, const FloatPoint2 &vec2MulIn, const FloatPoint2 &vec3AddIn) { return DirectX::XMVectorMultiplyAdd(vec1MulIn, vec2MulIn, vec3AddIn); }
		static FloatPoint2						Average(const std::vector<FloatPoint2> &arrayIn) { assert(arrayIn.size()); FloatPoint2 ave; for (const auto & each : arrayIn) ave += each; ave /= (float)arrayIn.size(); return ave; }
};
	/******************************************************************************
	*	FloatPoint3
	******************************************************************************/
	__declspec(align(16)) class FloatPoint3 : public FloatPoint2
	{
		/* variables */
	public:

	protected:

	private:

		/* methods */
	public:
		// Creation/Life cycle
		static std::shared_ptr<FloatPoint3>	Create() { return std::make_shared<FloatPoint3>(); }
		static std::unique_ptr<FloatPoint3>	CreateUnique() { return std::make_unique<FloatPoint3>(); }
		inline FloatPoint3() { SetZero(); }
		inline FloatPoint3(float xyz) { Set(xyz); }
		inline FloatPoint3(float x, float y, float z) { Set(x, y, z); }
		inline FloatPoint3(const FloatPoint3 &in) { v = in.v; } // copy
		inline FloatPoint3(const IntPoint3 &in) { Set(in.Get_XMFLOAT3()); }
		inline FloatPoint3(const DirectX::XMFLOAT3 &in) { Set(in); }
		inline FloatPoint3(FloatPoint3 &&in) { v = std::move(in.v); } // move
		inline FloatPoint3(const DirectX::XMVECTOR &vecIn) { v = DirectX::XMVectorSetW(vecIn, 0.f);}
		inline FloatPoint3(FloatPoint4 vecIn);
		inline FloatPoint3(uint8_t *bytesIn) { auto fp = reinterpret_cast<float*>(bytesIn); v = DirectX::XMLoadFloat3(&DirectX::XMFLOAT3(fp)); }
		inline FloatPoint3(float *floatIn) { v = DirectX::XMLoadFloat3(&DirectX::XMFLOAT3(floatIn)); }
		virtual ~FloatPoint3() = default;
		// Operators 
		inline FloatPoint3 & operator= (const FloatPoint3 &in) = default; // copy assignment
		inline FloatPoint3 & operator= (const DirectX::XMFLOAT3 &in) { Set(in); return *this; } // copy assignment
		inline FloatPoint3 & operator= (const DirectX::XMVECTOR &vecIn) { v = vecIn; return *this; }
		inline FloatPoint3 & operator= (FloatPoint3 &&in) = default; // move assignment
		// Conversions
		inline operator DirectX::XMFLOAT3() const { return Get_XMFLOAT3(); }
		inline operator DirectX::XMFLOAT3A() const { return Get_XMFLOAT3A(); }
		// Math Operators
		inline FloatPoint3 operator- () { return FloatPoint3(DirectX::XMVectorNegate(v)); }
		inline FloatPoint3 operator+ (const FloatPoint3 s) const { return FloatPoint3(DirectX::XMVectorAdd(v, s.v)); }
		inline FloatPoint3 operator- (const FloatPoint3 s) const { return FloatPoint3(DirectX::XMVectorSubtract(v, s.v)); }
		inline FloatPoint3 operator* (const FloatPoint3 s) const { return FloatPoint3(DirectX::XMVectorMultiply(v, s.v)); }
		inline FloatPoint3 operator/ (const FloatPoint3 s) const { return FloatPoint3(DirectX::XMVectorDivide(v, s.v)); }
		inline XMVECTOR & operator+= (const FloatPoint3 &s) { return v = DirectX::XMVectorAdd(v, s.v); }
		inline XMVECTOR & operator-= (const FloatPoint3 &s) { return v = DirectX::XMVectorSubtract(v, s.v); }
		inline XMVECTOR & operator*= (const FloatPoint3 &s) { return v = DirectX::XMVectorMultiply(v, s.v); }
		inline XMVECTOR & operator/= (const FloatPoint3 &s) { return v = DirectX::XMVectorDivide(v, s.v); }
		inline FloatPoint3 operator+ (const XMVECTOR vecIn) const { return FloatPoint3(DirectX::XMVectorAdd(v, vecIn)); }
		inline FloatPoint3 operator- (const XMVECTOR vecIn) const { return FloatPoint3(DirectX::XMVectorSubtract(v, vecIn)); }
		inline FloatPoint3 operator* (const XMVECTOR vecIn) const { return FloatPoint3(DirectX::XMVectorMultiply(v, vecIn)); }
		inline FloatPoint3 operator/ (const XMVECTOR vecIn) const { return FloatPoint3(DirectX::XMVectorDivide(v, vecIn)); }
		inline FloatPoint3 & operator+= (const XMVECTOR &vecIn) { v = DirectX::XMVectorAdd(v, vecIn); return *this; }
		inline FloatPoint3 & operator-= (const XMVECTOR &vecIn) { v = DirectX::XMVectorSubtract(v, vecIn); return *this; }
		inline FloatPoint3 & operator*= (const XMVECTOR &vecIn) { v = DirectX::XMVectorMultiply(v, vecIn); return *this; }
		inline FloatPoint3 & operator/= (const XMVECTOR &vecIn) { v = DirectX::XMVectorDivide(v, vecIn); return *this; }
		inline FloatPoint3 operator+ (float s) const { XMVECTOR sv = _mm_set_ps1(s); return FloatPoint3(DirectX::XMVectorAdd(v, sv)); }
		inline FloatPoint3 operator- (float s) const { XMVECTOR sv = _mm_set_ps1(s); return FloatPoint3(DirectX::XMVectorSubtract(v, sv)); }
		inline FloatPoint3 operator* (float s) const { return FloatPoint3(DirectX::XMVectorScale(v, s)); }
		inline FloatPoint3 operator/ (float s) const { XMVECTOR sv = _mm_set_ps1(s); return FloatPoint3(DirectX::XMVectorDivide(v, sv)); }
		inline XMVECTOR & operator+= (float s) { XMVECTOR sv = _mm_set_ps1(s); return v = DirectX::XMVectorAdd(v, sv); }
		inline XMVECTOR & operator-= (float s) { XMVECTOR sv = _mm_set_ps1(s); return v = DirectX::XMVectorSubtract(v, sv); }
		inline XMVECTOR & operator*= (float s) { return v = DirectX::XMVectorScale(v, s); }
		inline XMVECTOR & operator/= (float s) { XMVECTOR sv = _mm_set_ps1(s); return v = DirectX::XMVectorDivide(v, sv); }
		inline XMVECTOR operator* (const XMMATRIX &m) { return DirectX::XMVector3Transform(v, m); }
		inline XMVECTOR & operator*= (const XMMATRIX &m) { return v = DirectX::XMVector3Transform(v, m); }
		// Comparators
		inline bool operator<  (const FloatPoint3 &rhs) { return DirectX::XMVector3Less(v, rhs.GetVecConst()); }
		inline bool operator<= (const FloatPoint3 &rhs) { return DirectX::XMVector3LessOrEqual(v, rhs.GetVecConst()); }
		inline bool operator>  (const FloatPoint3 &rhs) { return DirectX::XMVector3Greater(v, rhs.GetVecConst()); }
		inline bool operator>= (const FloatPoint3 &rhs) { return DirectX::XMVector3GreaterOrEqual(v, rhs.GetVecConst()); }
		inline bool operator== (const FloatPoint3 &rhs) { return DirectX::XMVector3Equal(v, rhs.GetVecConst()); }
		inline bool operator!= (const FloatPoint3 &rhs) { return DirectX::XMVector3NotEqual(v, rhs.GetVecConst()); }
		// Accessors
		inline const DirectX::XMFLOAT3			Get_XMFLOAT3() const { DirectX::XMFLOAT3 rtn; DirectX::XMStoreFloat3(&rtn, v); return rtn; }
		inline const DirectX::XMFLOAT3A			Get_XMFLOAT3A() const { DirectX::XMFLOAT3A rtn; DirectX::XMStoreFloat3A(&rtn, v); return rtn; }
		inline const float						GetZ() const { return (float)DirectX::XMVectorGetZ(v); }
		// Assignments
		inline void								SetZ(const float z) { v = DirectX::XMVectorSetZ(v, z); }

		inline virtual void						Set(const float xyz) { v = DirectX::XMVectorSet(xyz, xyz, xyz, 0.f); }
		inline void								Set(const float x, const float y, const float z) { v = DirectX::XMVectorSet(x, y, z, 0.f); }
		inline void								Set(const int x, const int y, const int z) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 0.f); }
		inline void								Set(const unsigned int x, const unsigned int y, const unsigned int z) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 0.f); }
		inline void								Set(const long x, const long y, const long z) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 0.f); }
		inline void								Set(const unsigned long x, const unsigned long y, const unsigned long z) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), 0.f); }

		inline void								Set(const DirectX::XMFLOAT3 &point) { v = DirectX::XMLoadFloat3(&point); }
		inline void								Set(const DirectX::XMUINT3 &point) { v = DirectX::XMLoadUInt3(&point); v = DirectX::XMConvertVectorUIntToFloat(v, 0); }
		inline void								Set(const DirectX::XMINT3 &point) { v = DirectX::XMLoadSInt3(&point); v = DirectX::XMConvertVectorIntToFloat(v, 0); }
		// Functionality
		inline virtual void						MakeNormalize() { v = DirectX::XMVector3Normalize(v); }
		float __vectorcall						DotProduct(const FloatPoint3 &vecIn) { return (float)DirectX::XMVectorGetX(DirectX::XMVector3Dot(v, vecIn)); } // order does not mater A•B = B•A
		FloatPoint3 __vectorcall				CrossProduct(const FloatPoint3 &vecIn) { return DirectX::XMVector3Cross(v, vecIn); } // order does matter AxB = -(BxA)
		// Statics
		static FloatPoint3 __vectorcall			Absolute(const FloatPoint3 &point3In) { return FloatPoint3(DirectX::XMVectorAbs(point3In.GetVecConst())); }
		static FloatPoint3 __vectorcall			Normal(const FloatPoint3 &point3In) { return FloatPoint3(DirectX::XMVector3Normalize(point3In.GetVecConst())); }
		static const float __vectorcall			Magnitude(const FloatPoint3 &point3In) { return DirectX::XMVectorGetX(DirectX::XMVector3Length(point3In.GetVecConst())); }
		static FloatPoint3 __vectorcall			DotProduct(const FloatPoint3 &vec1In, const FloatPoint3 &vec2In) { return DirectX::XMVector3Dot(vec1In, vec2In); } // order does not mater A•B = B•A
		static FloatPoint3 __vectorcall			CrossProduct(const FloatPoint3 &vec1In, const FloatPoint3 &vec2In) { return DirectX::XMVector3Cross(vec1In, vec2In); } // order does mater AxB = -(BxA)
		static float __vectorcall				SumComponents(const FloatPoint3 &vec1In) { return DirectX::XMVectorGetX(DirectX::XMVectorSum(vec1In)); }
		static FloatPoint3 __vectorcall			MultiplyAdd(const FloatPoint3 &vec1MulIn, const FloatPoint3 &vec2MulIn, const FloatPoint3 &vec3AddIn) { return DirectX::XMVectorMultiplyAdd(vec1MulIn, vec2MulIn, vec3AddIn); }
		static FloatPoint3						Average(const std::vector<FloatPoint3> &arrayIn) { assert(arrayIn.size()); FloatPoint2(); FloatPoint3 ave; for (const auto & each : arrayIn) ave += each; ave /= (float)arrayIn.size(); return ave; }
};
	/******************************************************************************
	*	FloatPoint4
	******************************************************************************/
	__declspec(align(16)) class FloatPoint4 : public FloatPoint3
	{
		/* variables */
	public:

	protected:

	private:

		/* methods */
	public:
		// Creation/Life cycle
		static std::shared_ptr<FloatPoint4>	Create() { return std::make_shared<FloatPoint4>(); }
		static std::unique_ptr<FloatPoint4>	CreateUnique() { return std::make_unique<FloatPoint4>(); }
		inline FloatPoint4() { SetZero(); }
		inline FloatPoint4(float xyzw) { Set(xyzw); }
		inline FloatPoint4(FloatPoint3 in, float w) { Set(in,w); }
		inline FloatPoint4(float x, float y, float z, float w) { Set(x, y, z, w); }
		inline FloatPoint4(const FloatPoint4 &in) { v = in.v; } // copy
		inline FloatPoint4(const DirectX::XMFLOAT4 &in) { Set(in); }
		inline FloatPoint4(FloatPoint4 &&in) { v = std::move(in.v); } // move
		inline FloatPoint4(const DirectX::FXMVECTOR &vecIn) { v = vecIn; }
		inline FloatPoint4(uint8_t *bytesIn) { auto fp = reinterpret_cast<DirectX::XMFLOAT4*>(bytesIn); v = DirectX::XMLoadFloat4(fp); }
		inline FloatPoint4(float *floatIn) { auto fp = reinterpret_cast<DirectX::XMFLOAT4*>(floatIn); v = DirectX::XMLoadFloat4(fp); }
		virtual ~FloatPoint4() = default;
		// Operators 
		inline FloatPoint4 & operator= (const FloatPoint4 &in) = default; // copy assignment
		inline FloatPoint4 & operator= (const DirectX::XMFLOAT4 &in) { Set(in); return *this; } // copy assignment
		inline FloatPoint4 & operator= (const DirectX::XMVECTOR &vecIn) { v = vecIn; return *this; }
		inline FloatPoint4 & operator= (FloatPoint4 &&in) = default; // move assignment
		// Conversions
		inline operator DirectX::XMFLOAT4() const { return Get_XMFLOAT4(); }
		inline operator DirectX::XMFLOAT4A() const { return Get_XMFLOAT4A(); }
		// Math Operators
		inline FloatPoint4 operator- () { return FloatPoint4(DirectX::XMVectorNegate(v)); }
		inline FloatPoint4 operator+ (const FloatPoint4 &s) { return FloatPoint4(DirectX::XMVectorAdd(v, s.v)); }
		inline FloatPoint4 operator- (const FloatPoint4 &s) { return FloatPoint4(DirectX::XMVectorSubtract(v, s.v)); }
		inline FloatPoint4 operator* (const FloatPoint4 &s) { return FloatPoint4(DirectX::XMVectorMultiply(v, s.v)); }
		inline FloatPoint4 operator/ (const FloatPoint4 &s) { return FloatPoint4(DirectX::XMVectorDivide(v, s.v)); }
		inline XMVECTOR & operator+= (const FloatPoint4 &s) { return v = DirectX::XMVectorAdd(v, s.v); }
		inline XMVECTOR & operator-= (const FloatPoint4 &s) { return v = DirectX::XMVectorSubtract(v, s.v); }
		inline XMVECTOR & operator*= (const FloatPoint4 &s) { return v = DirectX::XMVectorMultiply(v, s.v); }
		inline XMVECTOR & operator/= (const FloatPoint4 &s) { return v = DirectX::XMVectorDivide(v, s.v); }
		inline FloatPoint4 operator+ (const XMVECTOR &vecIn) { return FloatPoint4(DirectX::XMVectorAdd(v, vecIn)); }
		inline FloatPoint4 operator- (const XMVECTOR &vecIn) { return FloatPoint4(DirectX::XMVectorSubtract(v, vecIn)); }
		inline FloatPoint4 operator* (const XMVECTOR &vecIn) { return FloatPoint4(DirectX::XMVectorMultiply(v, vecIn)); }
		inline FloatPoint4 operator/ (const XMVECTOR &vecIn) { return FloatPoint4(DirectX::XMVectorDivide(v, vecIn)); }
		inline FloatPoint4 & operator+= (const XMVECTOR &vecIn) { v = DirectX::XMVectorAdd(v, vecIn); return *this; }
		inline FloatPoint4 & operator-= (const XMVECTOR &vecIn) { v = DirectX::XMVectorSubtract(v, vecIn); return *this; }
		inline FloatPoint4 & operator*= (const XMVECTOR &vecIn) { v = DirectX::XMVectorMultiply(v, vecIn); return *this; }
		inline FloatPoint4 & operator/= (const XMVECTOR &vecIn) { v = DirectX::XMVectorDivide(v, vecIn); return *this; }
		inline FloatPoint4 operator+ (float s) { return FloatPoint4(DirectX::XMVectorAdd(v, DirectX::XMVectorReplicate(s))); }
		inline FloatPoint4 operator- (float s) { return FloatPoint4(DirectX::XMVectorSubtract(v, DirectX::XMVectorReplicate(s))); }
		inline FloatPoint4 operator* (float s) { return FloatPoint4(DirectX::XMVectorScale(v, s)); }
		inline FloatPoint4 operator/ (float s) { return FloatPoint4(DirectX::XMVectorDivide(v, DirectX::XMVectorReplicate(s))); }
		inline XMVECTOR & operator+= (float s) { return v = DirectX::XMVectorAdd(v, DirectX::XMVectorReplicate(s)); }
		inline XMVECTOR & operator-= (float s) { return v = DirectX::XMVectorSubtract(v, DirectX::XMVectorReplicate(s)); }
		inline XMVECTOR & operator*= (float s) { return v = DirectX::XMVectorScale(v, s); }
		inline XMVECTOR & operator/= (float s) { return v = DirectX::XMVectorDivide(v, DirectX::XMVectorReplicate(s)); }
		inline FloatPoint4 operator* (const XMMATRIX &m) { return FloatPoint4(DirectX::XMVector3Transform(v, m)); }
		//inline XMVECTOR operator* (const XMMATRIX &m) { return XMVector3Transform(v, m); }
		inline XMVECTOR & operator*=  (const XMMATRIX &m) { return v = DirectX::XMVector3Transform(v, m); }
		// Comparators
		inline bool operator<  (const FloatPoint4 &rhs) { return DirectX::XMVector4Less(v, rhs.GetVecConst()); }
		inline bool operator<= (const FloatPoint4 &rhs) { return DirectX::XMVector4LessOrEqual(v, rhs.GetVecConst()); }
		inline bool operator>  (const FloatPoint4 &rhs) { return DirectX::XMVector4Greater(v, rhs.GetVecConst()); }
		inline bool operator>= (const FloatPoint4 &rhs) { return DirectX::XMVector4GreaterOrEqual(v, rhs.GetVecConst()); }
		inline bool operator== (const FloatPoint4 &rhs) { return DirectX::XMVector4Equal(v, rhs.GetVecConst()); }
		inline bool operator!= (const FloatPoint4 &rhs) { return DirectX::XMVector4NotEqual(v, rhs.GetVecConst()); }
		// Accessors
		inline const DirectX::XMFLOAT4			Get_XMFLOAT4() const { DirectX::XMFLOAT4 rtn; DirectX::XMStoreFloat4(&rtn, v); return rtn; }
		inline const DirectX::XMFLOAT4A			Get_XMFLOAT4A() const { DirectX::XMFLOAT4A rtn; DirectX::XMStoreFloat4A(&rtn, v); return rtn; }
		inline const float						GetW() const { return (float)DirectX::XMVectorGetW(v); }
		// Assignments
		inline virtual void						SetW(const float w) { v = DirectX::XMVectorSetW(v, w); }

		inline virtual void						Set(const float xyzw) { v = DirectX::XMVectorReplicate(xyzw); }
		inline void								Set(FloatPoint3 in, float w) { v = XMVectorSetW(in, w); }
		inline void								Set(const float x, const float y, const float z, const float w) { v = DirectX::XMVectorSet(x, y, z, w); }
		inline void								Set(const int x, const int y, const int z, const int w) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w)); }
		inline void								Set(const unsigned int x, const unsigned int y, const unsigned int z, const unsigned int w) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w)); }
		inline void								Set(const long x, const long y, const long z, const long w) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w)); }
		inline void								Set(const unsigned long x, const unsigned long y, const unsigned long z, const unsigned long w) { v = DirectX::XMVectorSet(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w)); }

		inline void								Set(const DirectX::XMFLOAT4 &point) { v = DirectX::XMLoadFloat4(&point); }
		inline void								Set(const DirectX::XMUINT4 &point) { v = DirectX::XMLoadUInt4(&point); v = DirectX::XMConvertVectorUIntToFloat(v, 0); }
		inline void								Set(const DirectX::XMINT4 &point) { v = DirectX::XMLoadSInt4(&point); v = DirectX::XMConvertVectorIntToFloat(v, 0); }
		// Functionality
		inline virtual void						MakeNormalize() { v = DirectX::XMVector4Normalize(v); }
		float __vectorcall						DotProduct(const FloatPoint4 &vecIn) { return (float)DirectX::XMVectorGetX(DirectX::XMVector4Dot(v, vecIn)); } // order does not mater A•B = B•A
		// Statics
		static FloatPoint4 __vectorcall			Absolute(const FloatPoint4 &point4In) { return FloatPoint4(DirectX::XMVectorAbs(point4In.GetVecConst())); }
		static FloatPoint4 __vectorcall			Normal(const FloatPoint4 &point4In) { return FloatPoint4(DirectX::XMVector4Normalize(point4In.GetVecConst())); }
		static const float __vectorcall			Magnitude(const FloatPoint4 &point4In) { return DirectX::XMVectorGetX(DirectX::XMVector4Length(point4In.GetVecConst())); }
		static FloatPoint4 __vectorcall			DotProduct(const FloatPoint4 &vec1In, const FloatPoint4 &vec2In) { return DirectX::XMVector4Dot(vec1In, vec2In); } // order does not mater A•B = B•A
		static FloatPoint4 __vectorcall			CrossProduct(const FloatPoint4 &vec1In, const FloatPoint4 &vec2In, const FloatPoint4 &vec3In) { return DirectX::XMVector4Cross(vec1In, vec2In, vec3In); } // order does mater AxB = -(BxA)
		static float __vectorcall				SumComponents(const FloatPoint4 &vec1In) { return DirectX::XMVectorGetX(DirectX::XMVectorSum(vec1In)); }
		static FloatPoint4 __vectorcall			MultiplyAdd(const FloatPoint4 &vec1MulIn, const FloatPoint4 &vec2MulIn, const FloatPoint4 &vec3AddIn) { return DirectX::XMVectorMultiplyAdd(vec1MulIn, vec2MulIn, vec3AddIn); }
		static FloatPoint4						Average(const std::vector<FloatPoint4> &arrayIn) { assert(arrayIn.size()); FloatPoint4 ave; for (const auto & each : arrayIn) ave += each; ave /= (float)arrayIn.size(); return ave; }

	};

	/******************************************************************************
	*	Conversions
	******************************************************************************/
	inline UIntPoint2::UIntPoint2(const IntPoint2 & in) { auto temp = in.Get_XMUINT2(); u[0] = temp.x; u[1] = temp.y; }
	inline UIntPoint2::UIntPoint2(const FloatPoint2 & in) { auto temp = in.Get_XMUINT2(); u[0] = temp.x; u[1] = temp.y; }
	inline IntPoint2::IntPoint2(const FloatPoint2 & in) { auto temp = in.Get_XMINT2(); i[0] = temp.x; i[1] = temp.y; }

	/******************************************************************************
	*	Streams
	******************************************************************************/
	std::ostream& operator<< (std::ostream &os, const UIntPoint2 &in);
	std::ostream& operator<< (std::ostream &os, const IntPoint2 &in);
	std::ostream& operator<< (std::ostream &os, const IntPoint3 &in);
	std::ostream& operator<< (std::ostream& os, const FloatPoint2 &in);
	std::ostream& operator<< (std::ostream &os, const FloatPoint3 &in);
	std::ostream& operator<< (std::ostream &os, const FloatPoint4 &in);

	std::istream& operator>> (std::istream &is, King::UIntPoint2 &in);
	std::istream& operator>> (std::istream &is, King::IntPoint2 &in);
	std::istream& operator>> (std::istream &is, King::FloatPoint2 &in);
	std::istream& operator>> (std::istream &is, King::FloatPoint3 &in);
	std::istream& operator>> (std::istream &is, King::FloatPoint4 &in);

	/******************************************************************************
	*	Math functions
	*		Easy duplication
	******************************************************************************/
#define MAKE_SIMD_FUNCS( Type ) \
	inline Type __vectorcall Sqrt( Type s ) { return Type(XMVectorSqrt(s)); } \
	inline Type __vectorcall Recip( Type s ) { return Type(XMVectorReciprocal(s)); } \
	inline Type __vectorcall RecipSqrt( Type s ) { return Type(XMVectorReciprocalSqrt(s)); } \
	inline Type __vectorcall Floor( Type s ) { return Type(XMVectorFloor(s)); } \
	inline Type __vectorcall Ceiling( Type s ) { return Type(XMVectorCeiling(s)); } \
	inline Type __vectorcall Round( Type s ) { return Type(XMVectorRound(s)); } \
	inline Type __vectorcall Abs( Type s ) { return Type(XMVectorAbs(s)); } \
	inline Type __vectorcall Exp( Type s ) { return Type(XMVectorExp(s)); } \
	inline Type __vectorcall Pow( Type b, Type e ) { return Type(XMVectorPow(b, e)); } \
	inline Type __vectorcall Max( Type a, Type b ) { return Type(XMVectorMax(a, b)); } \
	inline Type __vectorcall Min( Type a, Type b ) { return Type(XMVectorMin(a, b)); } \
	inline Type __vectorcall Clamp( Type v, Type a, Type b ) { return Min(Max(v, a), b); } \
	inline Type __vectorcall Lerp( Type a, Type b, Type t ) { return Type(XMVectorLerpV(a, b, t)); }

	MAKE_SIMD_FUNCS(FloatPoint2);
	MAKE_SIMD_FUNCS(FloatPoint3);
	MAKE_SIMD_FUNCS(FloatPoint4);
#undef MAKE_SIMD_FUNCS
	inline float __vectorcall Dot(const FloatPoint2 &vec1In, const FloatPoint2 &vec2In) { return FloatPoint2(DirectX::XMVector2Dot(vec1In, vec2In)).GetX(); } // order does not mater A•B = B•A
	inline float __vectorcall Dot(const FloatPoint3 &vec1In, const FloatPoint3 &vec2In) { return FloatPoint3(DirectX::XMVector3Dot(vec1In, vec2In)).GetX(); }
	inline float __vectorcall Dot(const FloatPoint4 &vec1In, const FloatPoint4 &vec2In) { return FloatPoint4(DirectX::XMVector4Dot(vec1In, vec2In)).GetX(); }

	inline FloatPoint2 __vectorcall Cross(const FloatPoint2 &vec1In, const FloatPoint2 &vec2In) { return FloatPoint2(DirectX::XMVector2Cross(vec1In, vec2In)); } // order does mater AxB = -(BxA)
	inline FloatPoint3 __vectorcall Cross(const FloatPoint3 &vec1In, const FloatPoint3 &vec2In) { return FloatPoint3(DirectX::XMVector3Cross(vec1In, vec2In)); }
	inline FloatPoint4 __vectorcall Cross(const FloatPoint4 &vec1In, const FloatPoint4 &vec2In, const FloatPoint4 &vec3In) { return FloatPoint4(DirectX::XMVector4Cross(vec1In, vec2In, vec3In)); }

	/******************************************************************************
	*	Quaternion
	*
	*	a + b*i + c*j + d*k
	*	i^2 = j^2 = k^2 = ijk = -1
	*	where a, b, c, and d are real numbers, and i, j, and k are the fundamental quaternion units. 
	*	https://en.wikipedia.org/wiki/Quaternion
	******************************************************************************/
	__declspec(align(16)) class Quaternion : public FloatPoint4
	{
	public:
		// Construction/Destruction
		inline explicit Quaternion() { v = DirectX::XMQuaternionIdentity(); }
		inline explicit Quaternion(const FloatPoint3 axis, const float angle) { v = DirectX::XMQuaternionRotationAxis(axis, angle); }
		inline explicit Quaternion(float pitch, float yaw, float roll) { v = DirectX::XMQuaternionRotationRollPitchYaw(pitch, yaw, roll); }
		inline explicit Quaternion(const DirectX::XMMATRIX& matrix) { v = DirectX::XMQuaternionRotationMatrix(matrix); }
		inline explicit Quaternion(const DirectX::XMVECTOR &vec) { v = vec; }
		inline explicit Quaternion(const FloatPoint4 q) { v = q; }
		inline Quaternion(const Quaternion &in) = default; // copy 
		inline Quaternion(Quaternion &&in) = default; // move
		virtual ~Quaternion() = default;
		// Operators 
		inline Quaternion & operator= (const Quaternion &in) = default; // copy assignment
		inline Quaternion & operator= (Quaternion &&in) = default; // move assignment
		inline Quaternion & operator= (const FXMVECTOR &in) { v = in; };

		inline Quaternion operator~ (void) const { return Quaternion(DirectX::XMQuaternionConjugate(v)); }
		inline Quaternion operator- (void) const { return Quaternion(DirectX::XMVectorNegate(v)); }
		inline Quaternion operator* (Quaternion rhs) const { return Quaternion(DirectX::XMQuaternionMultiply(rhs, v)); }
		inline FloatPoint3 operator* (FloatPoint3 rhs) const { return FloatPoint3(DirectX::XMVector3Rotate(rhs, v)); }
		inline Quaternion & operator*= (Quaternion rhs) { *this = *this * rhs; return *this; }

		virtual DirectX::XMFLOAT3		GetRotationInEulerAngles();
	};
}
