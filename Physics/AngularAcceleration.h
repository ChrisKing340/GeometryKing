/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          AngularAcceleration    

Description:   
Symbol:         𝛼 in pitch, yaw, roll 𝛼(𝒾, 𝒿, 𝓀)

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
#include <iomanip>
// King namespace
#include "..\MathSIMD\MathSIMD.h"
#include "..\Physics\UnitOfMeasure.h"
#include "..\Physics\Force.h"
#include "..\Physics\Torque.h"
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
*    Linear:
*	     ͢a = ∑ ͢F / m
*    Angular:  
*        𝛼 = ∑ 𝜏 / I ; radians/s^2
*        ∑ 𝜏 = I • 𝛼 ;  
*
*        I = m • r^2 ; for a point mass rotating around an axis a distance r away.
*    
*    Back to Linear:
*        ͢at = r • 𝛼 ; tangential acceleration in m/s^2 in planar motion
*
******************************************************************************/

namespace King {
    /******************************************************************************
    *    AngularAcceleration
    ******************************************************************************/
    class AngularAcceleration;
    // Angular acceleration
    AngularAcceleration operator/(const Torque& tIn, const UnitOfMeasure::Inertia& inertiaIn);
    // torque about an axis
    Torque operator*(const UnitOfMeasure::Inertia& inertiaIn, const AngularAcceleration& angAccelIn);
    Torque operator*(const AngularAcceleration& angAccelIn, const UnitOfMeasure::Inertia& inertiaIn);
    // linear acceleration normal (in the direction of) the radius, rotation about an axis
    Acceleration operator*(const AngularAcceleration& alphaIn, const Distance& rIn); 
    Acceleration operator*(const Distance& rIn, const AngularAcceleration& alphaIn);

    class alignas(16) AngularAcceleration
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::AngularAccel         _magnitude; // absolute, >= 0
        float3                              _unit_direction; // proportion about each axis
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<AngularAcceleration> Create() { return std::make_shared<AngularAcceleration>(); }
        AngularAcceleration() = default;
        explicit AngularAcceleration(const UnitOfMeasure::AngularAccel aIn) : _magnitude(abs(aIn)) { if (_magnitude != aIn) { _unit_direction = -_unit_direction; }; }
        AngularAcceleration(const float3 &vectorIn) { _magnitude = float3::Magnitude(vectorIn); _unit_direction = float3::Normal(vectorIn); }
        explicit AngularAcceleration(const float &magIn, const float3 &dirIn) { _magnitude = abs(magIn); _unit_direction = float3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit AngularAcceleration(const UnitOfMeasure::AngularAccel& aIn, const float3 &rotXYZIn) { _magnitude = abs(aIn); _unit_direction = float3::Normal(rotXYZIn); if (_magnitude != aIn) { _unit_direction = -_unit_direction; }; }
        explicit AngularAcceleration(const float& x, const float& y, const float& z) : AngularAcceleration(float3(x,y,z)) { ; }
        AngularAcceleration(const AngularAcceleration &in) { *this = in; } // forward to copy assignment
        AngularAcceleration(AngularAcceleration &&in) noexcept { *this = std::move(in); } // forward to move assignment

        ~AngularAcceleration() { ; }

        static const std::string Unit() { return UnitOfMeasure::AngularAccel::_unit; }
        static const std::wstring UnitW() { return UnitOfMeasure::AngularAccel::_wunit; }

        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::AngularAccel() const { return _magnitude; }
        inline explicit operator float3() const { return GetVector(); }
        // Comparators
        inline bool operator<  (const float& rhs) const { return _magnitude < rhs; }
        inline bool operator<= (const float& rhs) const { return _magnitude <= rhs; }
        inline bool operator>  (const float& rhs) const { return _magnitude > rhs; }
        inline bool operator>= (const float& rhs) const { return _magnitude >= rhs; }
        inline bool operator== (const float& rhs) const { return _magnitude == rhs; }
        inline bool operator!= (const float& rhs) const { return _magnitude != rhs; }
        inline bool operator<  (const Force& rhs) const { return DirectX::XMVector3Less(GetVector(), rhs.GetVector()); }
        inline bool operator<= (const Force& rhs) const { return DirectX::XMVector3LessOrEqual(GetVector(), rhs.GetVector()); }
        inline bool operator>  (const Force& rhs) const { return DirectX::XMVector3Greater(GetVector(), rhs.GetVector()); }
        inline bool operator>= (const Force& rhs) const { return DirectX::XMVector3GreaterOrEqual(GetVector(), rhs.GetVector()); }
        inline bool operator== (const Force& rhs) const { return DirectX::XMVector3Equal(GetVector(), rhs.GetVector()); }
        inline bool operator!= (const Force& rhs) const { return DirectX::XMVector3NotEqual(GetVector(), rhs.GetVector()); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<AngularAcceleration*>(p)); }
        inline AngularAcceleration & operator= (const AngularAcceleration &other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; return *this; } // copy assign
        inline AngularAcceleration & operator= (AngularAcceleration &&other) noexcept { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline AngularAcceleration operator- () const { return AngularAcceleration(-_unit_direction); }
        inline AngularAcceleration operator+ (const AngularAcceleration & in) const { return AngularAcceleration(GetVector() + in.GetVector()); }
        inline AngularAcceleration operator- (const AngularAcceleration & in) const { return AngularAcceleration(GetVector() - in.GetVector()); }
        inline AngularAcceleration operator* (const AngularAcceleration & in) const { return AngularAcceleration(GetVector() * in.GetVector()); }
        inline AngularAcceleration operator/ (const AngularAcceleration & in) const { return AngularAcceleration(GetVector() / in.GetVector()); }
        inline AngularAcceleration & operator+= (const AngularAcceleration & in) { *this = *this + in; return *this; } 
        inline AngularAcceleration & operator-= (const AngularAcceleration & in) { *this = *this - in; return *this; }
        inline AngularAcceleration & operator*= (const AngularAcceleration & in) { *this = *this * in; return *this; }
        inline AngularAcceleration & operator/= (const AngularAcceleration & in) { *this = *this / in; return *this; }
        inline AngularAcceleration operator* (const float & in) const { return AngularAcceleration(_magnitude * in, _unit_direction); }
        inline AngularAcceleration operator/ (const float& in) const { return AngularAcceleration(_magnitude / in, _unit_direction); }

        // Init/Start/Stop/Destroy
        // Functionality
        bool                                IsZero() const { return _magnitude == 0.f; }
        bool                                IsOrNearZero() const { return _magnitude <= 1.0e-5f; }
        // Accessors
        const auto&                         Get_magnitude() const { return _magnitude; }
        auto&                               Get_magnitude() { return _magnitude; }
        const auto&                         Get_unit_direction() const { return _unit_direction; }
        auto&                               Get_unit_direction() { return _unit_direction; }
        const float3                        GetVector() const { return _unit_direction * _magnitude; }
        float                               GetValueEN() const { return UnitOfMeasure::mPerSecSqToftPerSecSq * _magnitude; }
        float                               GetValueSI() const { return UnitOfMeasure::mPerSecSq * _magnitude; }
        // Assignments
        // Note: set unit direction before magnitude in case sign of magnitude is switched
        void                                Set_magnitude(const float &_magnitude_IN) { _magnitude = abs(_magnitude_IN); if (_magnitude != _magnitude_IN) { _unit_direction = -_unit_direction; }; }
        void __vectorcall                   Set_unit_direction(const float3 &_unit_direction_IN) { _unit_direction = float3::Normal(_unit_direction_IN); }
        // Input & Output functions that can have access to protected & private data
        friend std::ostream& operator<< (std::ostream& os, const AngularAcceleration& in);
        friend std::istream& operator>> (std::istream& is, AngularAcceleration& out);
        friend std::wostream& operator<< (std::wostream& os, const AngularAcceleration& in);
        friend std::wistream& operator>> (std::wistream& is, AngularAcceleration& out);
        friend void to_json(json& j, const AngularAcceleration& from);
        friend void from_json(const json& j, AngularAcceleration& to);
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const AngularAcceleration& in);
    std::istream& operator>> (std::istream& is, AngularAcceleration& out);
    std::wostream& operator<< (std::wostream& os, const AngularAcceleration& in);
    std::wistream& operator>> (std::wistream& is, AngularAcceleration& out);
    void to_json(json& j, const AngularAcceleration& from);
    void from_json(const json& j, AngularAcceleration& to);
}