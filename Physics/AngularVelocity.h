﻿/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          AngularVelocity    

Description:    Euler angles around each axis kept in a float3 in radians.
Symbol:         𝜔 in pitch, yaw, roll 𝜔(𝒾, 𝒿, 𝓀)

Usage:          AngularVelocity(const UnitOfMeasure::Time &t, const AngularAcceleration & accIn)             

Contact:        ChrisKing340@gmail.com

References:        

MIT License

Copyright (c) 2020 Christopher H. King

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
#include <iomanip>
// King namespace
#include "..\MathSIMD\MathSIMD.h"
#include "..\Physics\UnitOfMeasure.h"
#include "..\Physics\Force.h"
#include "..\Physics\Distance.h"
#include "..\Physics\Acceleration.h"
#include "..\Physics\AngularAcceleration.h"
// 3rdPart namespace
#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

/******************************************************************************
*   Physic Basics
*
*   Newtons's 2nd Law:
*      The acceleration of an object as produced by a net force is directly
*      proportional to the magnitude of the net force, in the same direction
*	   as the net force, and inversely proportional to the mass of the object.
*      Known as the law of motion.
*
*   Kinematics
*       𝜔f - 𝜔i = 𝛼 • 𝛥t ; radians / sec    
*       𝜃f - 𝜃i = 𝜔 • 𝛥t + 1/2 • 𝛼 • 𝛥t^2; radians with constant 𝛼
*
*   Back to Linear:
*       ͢an = r • 𝜔^2 ; normal acceleration in m/s^2
*
******************************************************************************/

namespace King {

    /******************************************************************************
    *    AngularVelocity
    ******************************************************************************/
    class AngularVelocity;
    class Distance;
    AngularVelocity operator*(const UnitOfMeasure::Time &t, const AngularAcceleration & accIn); 
    AngularVelocity operator*(const AngularAcceleration & accIn, const UnitOfMeasure::Time &t); 
    Acceleration operator*(const Distance& rIn, const AngularVelocity& angularVelIn); // normal acceleration
    Acceleration operator*(const AngularVelocity& angularVelIn, const Distance& rIn); // normal acceleration

    class alignas(16) AngularVelocity
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::AngularSpeed         _magnitude; // absolute, >= 0
        float3                              _unit_direction; // Euler angles normalized
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<AngularVelocity> Create() { return std::make_shared<AngularVelocity>(); }
        AngularVelocity() = default;
        explicit AngularVelocity(const float &magIn, const float3 &axisIn) { _magnitude = abs(magIn); _unit_direction = float3::Normal(axisIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit AngularVelocity(const UnitOfMeasure::AngularSpeed&s, const float3 & axisIn) { _magnitude = abs(s); _unit_direction = float3::Normal(axisIn); if (_magnitude != s) { _unit_direction = -_unit_direction; }; }
        AngularVelocity(const float3 &vectorIn) { _magnitude = float3::Magnitude(vectorIn); _unit_direction = float3::Normal(vectorIn); }
        explicit AngularVelocity(const AngularAcceleration& accIn, const UnitOfMeasure::Time &t) { _magnitude = accIn.Get_magnitude() * t; _unit_direction = accIn.Get_unit_direction(); }
        explicit AngularVelocity(const std::vector<AngularAcceleration> & accelIn, const UnitOfMeasure::Time &t) { float3 sum; for (const auto & e : accelIn) sum += e.GetVector(); *this = AngularAcceleration(sum) * t; }
        explicit AngularVelocity(const float& x, const float& y, const float& z) : AngularVelocity(float3(x, y, z)) { ; }
        AngularVelocity(const AngularVelocity &in) { *this = in; } // forward to copy assignment
        AngularVelocity(AngularVelocity &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~AngularVelocity() { ; }

        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::AngularSpeed() const { return _magnitude; }
        inline explicit operator float3() const { return GetVector(); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<AngularVelocity*>(p)); }
        inline AngularVelocity & operator= (const AngularVelocity &other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; return *this; } // copy assign
        inline AngularVelocity & operator= (AngularVelocity &&other) noexcept { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline AngularVelocity operator- () const { return AngularVelocity(-_unit_direction); }
        inline AngularVelocity operator+ (const AngularVelocity& in) const { return AngularVelocity(GetVector() + in.GetVector()); }
        inline AngularVelocity operator- (const AngularVelocity& in) const { return AngularVelocity(GetVector() - in.GetVector()); }
        inline AngularVelocity operator* (const AngularVelocity& in) const { return AngularVelocity(GetVector() * in.GetVector()); }
        inline AngularVelocity operator/ (const AngularVelocity& in) const { return AngularVelocity(GetVector() / in.GetVector()); }
        inline AngularVelocity & operator+= (const AngularVelocity & in) { *this = *this + in; return *this; } 
        inline AngularVelocity & operator-= (const AngularVelocity & in) { *this = *this - in; return *this; }
        inline AngularVelocity & operator*= (const AngularVelocity & in) { *this = *this * in; return *this; } // AngularSpeedSq
        inline AngularVelocity & operator/= (const AngularVelocity & in) { *this = *this / in; return *this; }
        inline AngularVelocity operator* (const float & in) const { return AngularVelocity(_magnitude * in, _unit_direction); }
        // Init/Start/Stop/Destroy
        // Functionality
        // Accessors
        const auto&                         Get_magnitude() const { return _magnitude; }
        auto&                               Get_magnitude() { return _magnitude; }
        const auto&                         Get_unit_direction() const { return _unit_direction; }
        auto&                               Get_unit_direction() { return _unit_direction; }
        const float3                        GetVector() const { return _unit_direction * _magnitude; }
        float                               GetValueEN() const { return UnitOfMeasure::mPerSecToftPerSec * _magnitude; }
        float                               GetValueSI() const { return UnitOfMeasure::mPerSec * _magnitude; }
        // Assignments
        // Note: set unit direction before magnitude in case sign of magnitude is switched
        void                                Set_magnitude(const float &_magnitude_IN) { _magnitude = abs(_magnitude_IN); if (_magnitude != _magnitude_IN) { _unit_direction = -_unit_direction; }; }
        void __vectorcall                   Set_unit_direction(const float3 &_unit_direction_IN) { _unit_direction = float3::Normal(_unit_direction_IN); }
        // Input & Output functions that can have access to protected & private data
        friend std::ostream& operator<< (std::ostream& os, const AngularVelocity& in);
        friend std::istream& operator>> (std::istream& is, AngularVelocity& out);
        friend std::wostream& operator<< (std::wostream& os, const AngularVelocity& in);
        friend std::wistream& operator>> (std::wistream& is, AngularVelocity& out);
        friend void to_json(json& j, const AngularVelocity& from);
        friend void from_json(const json& j, AngularVelocity& to);
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const AngularVelocity& in);
    std::istream& operator>> (std::istream& is, AngularVelocity& out);
    std::wostream& operator<< (std::wostream& os, const AngularVelocity& in);
    std::wistream& operator>> (std::wistream& is, AngularVelocity& out);
    void to_json(json& j, const AngularVelocity& from);
    void from_json(const json& j, AngularVelocity& to);
}
