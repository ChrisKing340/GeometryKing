/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:            Position    

Description:    

Usage:            

Contact:        ChrisKing340@gmail.com

References:        

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
// needed for C++17 which had breaking change for std::make_shared
#ifndef _ENABLE_EXTENDED_ALIGNED_STORAGE
#define _ENABLE_EXTENDED_ALIGNED_STORAGE
#endif
// std namespace
#include <vector>
#include <memory>
#include <string>
// King namespace
#include "..\MathSIMD\MathSIMD.h"
#include "..\Physics\UnitOfMeasure.h"
#include "..\Physics\Acceleration.h"
#include "..\Physics\Velocity.h"
#include "..\Physics\Distance.h"

/******************************************************************************
*    Physic Basics
*
*    Newton's 1st Law:
*        An object at rest stays at rest and an object in motion stays in motion
*        unless acted upon by a net sum external force. Known as Law of inertia.
*
******************************************************************************/

namespace King {

    /******************************************************************************
    *    Position
    ******************************************************************************/
    class Position;

    class alignas(16) Position
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::Length               _magnitude; // absolute, >= 0
        float3                              _unit_direction;
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Position> Create() { return std::make_shared<Position>(); }
        Position() : _magnitude(0), _unit_direction(0.f, 0.f, 0.f) { ; }
        Position(const float &magIn, const FloatPoint3 &dirIn) { _magnitude = abs(magIn); _unit_direction = FloatPoint3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit Position(const UnitOfMeasure::Length &l, const FloatPoint3 &dirIn) { _magnitude = abs(l); _unit_direction = FloatPoint3::Normal(dirIn); if (_magnitude != l) { _unit_direction = -_unit_direction; }; }
        explicit inline Position(const DirectX::XMVECTOR &vIn) { Set(FloatPoint3(vIn)); }
        explicit inline Position(const DirectX::XMVECTORF32 &vIn) { Set(FloatPoint3(vIn)); }
        explicit inline Position(const FloatPoint3 &vectorIn) { Set(vectorIn); }
        explicit inline Position(const FloatPoint4 &vectorIn) { Set(static_cast<FloatPoint3>(vectorIn)); }
        explicit inline Position(const Distance &distanceIn) { Set(static_cast<FloatPoint3>(distanceIn)); }
        Position(const Position &in) { *this = in; } // forward to copy assignment
        Position(Position &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~Position() { ; }

        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::Length() const { return _magnitude; }
        inline explicit operator DirectX::XMVECTOR() const { return GetFloatPoint3().GetVecConst(); }
        inline operator FloatPoint3() const { return GetFloatPoint3(); } // allow implicit for a default behavior
        inline explicit operator FloatPoint4() const { return FloatPoint4(GetFloatPoint3(),1.0f); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Position*>(p)); }
        //inline Position & operator= (const XMVECTOR &in) { Set(FloatPoint3(in)); return *this; } // copy assignment
        //inline Position & operator= (const XMVECTORF32 &in) { Set(FloatPoint3(in)); return *this; } // copy assignment
        //inline Position & operator= (const FloatPoint3 &in) { Set(in); return *this; } // copy assignment
        //inline Position & operator= (const FloatPoint4 &in) { Set(static_cast<FloatPoint3>(in)); return *this; } // copy assignment
        inline Position & operator= (const Position &other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; return *this; } // copy assign
        inline Position & operator= (Position &&other) { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline Position operator- () const { return Position(-_unit_direction); }
        inline Position operator+ (const Position & in) const { return Position(GetFloatPoint3() + in.GetFloatPoint3()); }
        inline Position operator- (const Position & in) const { return Position(GetFloatPoint3() - in.GetFloatPoint3()); }
        inline Position operator* (const Position & in) const { return Position(GetFloatPoint3() * in.GetFloatPoint3()); }
        inline Position operator/ (const Position & in) const { return Position(GetFloatPoint3() / in.GetFloatPoint3()); }
        inline Position & operator+= (const Position & in) { *this = *this + in; return *this; } 
        inline Position & operator-= (const Position & in) { *this = *this - in; return *this; }
        inline Position & operator*= (const Position & in) { *this = *this * in; return *this; }
        inline Position & operator/= (const Position & in) { *this = *this / in; return *this; }
        inline Position operator* (const float & in) const { return Position(_magnitude * in, _unit_direction); }

        inline Position operator- (const Distance &rhs) const { Position ptOut(GetFloatPoint3() - (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator+ (const Distance &rhs) const { Position ptOut(GetFloatPoint3() + (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator* (const Distance &rhs) const { Position ptOut(GetFloatPoint3() * (FloatPoint3)(rhs)); return ptOut; }
        inline Position operator/ (const Distance &rhs) const { Position ptOut(GetFloatPoint3() / (FloatPoint3)(rhs)); return ptOut; }
        inline Position & operator+= (const Distance & in) { *this = *this + in; return *this; }
        inline Position & operator-= (const Distance & in) { *this = *this - in; return *this; }
        inline Position & operator*= (const Distance & in) { *this = *this * in; return *this; }
        inline Position & operator/= (const Distance & in) { *this = *this / in; return *this; }

        inline Position operator- (const FloatPoint3 &rhs) const { Position ptOut(GetFloatPoint3() - (rhs)); return ptOut; }
        inline Position operator+ (const FloatPoint3 &rhs) const { Position ptOut(GetFloatPoint3() + (rhs)); return ptOut; }
        inline Position operator* (const FloatPoint3 &rhs) const { Position ptOut(GetFloatPoint3() * (rhs)); return ptOut; }
        inline Position operator/ (const FloatPoint3 &rhs) const { Position ptOut(GetFloatPoint3() / (rhs)); return ptOut; }
        inline Position & operator-= (const FloatPoint3 &rhs) { *this = *this - rhs; return *this; }
        inline Position & operator+= (const FloatPoint3 &rhs) { *this = *this + rhs; return *this; }
        inline Position & operator*= (const FloatPoint3 &rhs) { *this = *this * rhs; return *this; }
        inline Position & operator/= (const FloatPoint3 &rhs) { *this = *this / rhs; return *this; }

        inline Position & operator*= (const DirectX::XMMATRIX &m) { _unit_direction = DirectX::XMVector4Transform(_unit_direction, m); return *this; }
        friend Position operator* (Position lhs, const DirectX::XMMATRIX &m) { lhs *= m; return lhs; } // invokes std::move(lhs)

        // Init/Start/Stop/Destroy
        // Functionality
        // Accessors
        const auto &                        Get_magnitude() const { return _magnitude; }
        const auto &                        Get_unit_direction() const { return _unit_direction; }
        const FloatPoint3                   GetFloatPoint3() const { return _unit_direction * _magnitude; }
        FloatPoint3                         GetFloatPoint3() { return _unit_direction * _magnitude; }
        DirectX::XMVECTOR                   GetVec() { return (_unit_direction * _magnitude).GetVec(); }
        DirectX::XMVECTOR                   GetVecConst() const { return (_unit_direction * _magnitude).GetVecConst(); }
        // Assignments
        void __vectorcall                   Set(const float3 & positionIn) { _magnitude = float3::Magnitude(positionIn); _unit_direction = float3::Normal(positionIn); }
        void                                Set_magnitude(const float &_magnitude_IN) { _magnitude = _magnitude_IN; }
        void __vectorcall                   Set_unit_direction(const float3 &_unit_direction_IN) { _unit_direction = _unit_direction_IN; }
    };
}