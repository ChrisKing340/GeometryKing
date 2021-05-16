/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:            Acceleration    

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
#include <iomanip>
// King namespace
#include "..\MathSIMD\MathSIMD.h"
#include "..\Physics\UnitOfMeasure.h"
#include "..\Physics\Force.h"
// 3rdPart namespace
#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

/******************************************************************************
*    Physic Basics
*
*    Newton's 2nd Law:
*        The acceleration of an object as produced by a net force is directly
*        proportional to the magnitude of the net force, in the same direction
*        as the net force, and inversely proportional to the mass of the object.
*            a = ∑ ͢F / m
*            ∑ ͢F = m • ͢a
*    
******************************************************************************/

namespace King {
    /******************************************************************************
    *    Acceleration
    ******************************************************************************/
    class Acceleration;
    Acceleration operator/(const Force & f, const UnitOfMeasure::Mass & m); // a = ∑ ͢F / m

    class alignas(16) Acceleration
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::Accel                _magnitude; // absolute, >= 0
        float3                              _unit_direction;
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Acceleration> Create() { return std::make_shared<Acceleration>(); }
        Acceleration() = default;
        explicit Acceleration(const UnitOfMeasure::Accel a) : _magnitude(abs(a)) { if (_magnitude != a) { _unit_direction = -_unit_direction; }; }
        Acceleration(const float3 &vectorIn) { _magnitude = float3::Magnitude(vectorIn); _unit_direction = float3::Normal(vectorIn); }
        explicit Acceleration(const float &magIn, const float3 &dirIn) { _magnitude = abs(magIn); _unit_direction = float3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit Acceleration(const UnitOfMeasure::Accel &a, const float3 &dirIn) { _magnitude = a; _unit_direction = float3::Normal(dirIn); }
        explicit Acceleration(const UnitOfMeasure::Mass &m, const Force & forceIn) { *this = forceIn / m; }
        explicit Acceleration(const UnitOfMeasure::Mass &m, const std::vector<Force> & forcesIn) { float3 sum; for (const auto & e : forcesIn) sum += float3(e); *this = Force(sum) / m; }
        explicit Acceleration(const float& x, const float& y, const float& z) : Acceleration(float3(x, y, z)) { ; }
        Acceleration(const Acceleration &in) { *this = in; } // forward to copy assignment
        Acceleration(Acceleration &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~Acceleration() { ; }

        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::Accel() const { return _magnitude; }
        inline explicit operator float3() const { return GetVector(); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Acceleration*>(p)); }
        inline Acceleration & operator= (const Acceleration &other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; return *this; } // copy assign
        inline Acceleration & operator= (Acceleration &&other) noexcept { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline Acceleration operator- () const { return Acceleration(-_unit_direction); }
        inline Acceleration operator+ (const Acceleration & in) const { return Acceleration(GetVector() + in.GetVector()); }
        inline Acceleration operator- (const Acceleration & in) const { return Acceleration(GetVector() - in.GetVector()); }
        inline Acceleration operator* (const Acceleration & in) const { return Acceleration(GetVector() * in.GetVector()); }
        inline Acceleration operator/ (const Acceleration & in) const { return Acceleration(GetVector() / in.GetVector()); }
        inline Acceleration & operator+= (const Acceleration & in) { *this = *this + in; return *this; } 
        inline Acceleration & operator-= (const Acceleration & in) { *this = *this - in; return *this; }
        inline Acceleration & operator*= (const Acceleration & in) { *this = *this * in; return *this; }
        inline Acceleration & operator/= (const Acceleration & in) { *this = *this / in; return *this; }
        inline Acceleration operator* (const float & in) const { return Acceleration(_magnitude * in, _unit_direction); }
        inline Acceleration operator/ (const float& in) const { return Acceleration(_magnitude / in, _unit_direction); }

        // Init/Start/Stop/Destroy
        // Functionality
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
        friend std::ostream& operator<< (std::ostream& os, const Acceleration& in);
        friend std::istream& operator>> (std::istream& is, Acceleration& out);
        friend std::wostream& operator<< (std::wostream& os, const Acceleration& in);
        friend std::wistream& operator>> (std::wistream& is, Acceleration& out);
        friend void to_json(json& j, const Acceleration& from);
        friend void from_json(const json& j, Acceleration& to);
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const Acceleration& in);
    std::istream& operator>> (std::istream& is, Acceleration& out);
    std::wostream& operator<< (std::wostream& os, const Acceleration& in);
    std::wistream& operator>> (std::wistream& is, Acceleration& out);
    void to_json(json& j, const Acceleration& from);
    void from_json(const json& j, Acceleration& to);
}