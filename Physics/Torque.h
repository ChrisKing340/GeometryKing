/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          Torque    

Description:    torque = radius x force
                The magnitude is r F sin𝜃
                𝜃 = atan2(sqrt(dot(cross(a, b), cross(a, b))), dot(a, b))

                The direction is perpendicular to both the radius from the axis 
                and to the force. It is conventional to choose it in the right 
                hand rule direction along the axis of rotation.

Symbol:         𝜏 =  ͢r ⨯ ͢F 

Usage:                / projected angle of ͢F from point of application
                     /
                    /
                   /
                  /
                 /𝜃   ͢r
                ⊙------>• (point of application)
                𝜏       ^
                        |
                        | ͢F
                        |
                        |

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
// 3rdPart namespace
#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

/******************************************************************************
*    Physic Basics
*
*    Newton's 1st Law:
*        An object at rest stays at rest and an object in motion stays in motion
*        UNLESS ACTED UPON BY A NET SUM EXTERNAL FORCE. Known as Law of inertia.
*
*        𝜏 = r ⨯ ͢F ; torque in N m
*    
******************************************************************************/

namespace King {
    /******************************************************************************
    *    Torque
    ******************************************************************************/
    class Torque;

    class alignas(16) Torque
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::AngularStrength      _magnitude; // absolute, >= 0, and r F sin𝜃 = 𝜏
        float3                              _unit_direction;
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Torque> Create() { return std::make_shared<Torque>(); }
        Torque() = default;
        explicit Torque(const UnitOfMeasure::AngularStrength s) : _magnitude(abs(s)) { if (_magnitude != s) { _unit_direction = -_unit_direction; }; }
        explicit Torque(const float &magIn, const float3 &dirIn) : _magnitude(abs(magIn)) { _unit_direction = float3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit Torque(const UnitOfMeasure::AngularStrength&s, const float3 &dirIn) { _magnitude = abs(s); _unit_direction = float3::Normal(dirIn); if (_magnitude != s) { _unit_direction = -_unit_direction; }; }
        Torque(const float3& r, const float3& F) { SetVector(float3::CrossProduct(r, F)); }
        explicit Torque(const Distance& r, const Force& F) { SetVector(float3::CrossProduct( float3(r), float3(F) )); }
        Torque(const float3 &vectorIn) { SetVector(vectorIn); }
        Torque(const std::vector<Torque> & torquesIn) { float3 vec; for (const auto & e : torquesIn) vec += float3(e); *this = Torque(vec); }
        Torque(const Torque &in) { *this = in; } // forward to copy assignment
        Torque(Torque &&in) noexcept { *this = std::move(in); } // forward to move assignment

        ~Torque() { ; }

        static const std::string Unit() { return UnitOfMeasure::AngularStrength::_unit; }
        static const std::wstring UnitW() { return UnitOfMeasure::AngularStrength::_wunit; }

        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::AngularStrength() const { return _magnitude; }
        inline operator float3() const { return GetVector(); }
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
        void   operator delete (void *p) { _aligned_free(static_cast<Torque*>(p)); }
        inline Torque & operator= (const Torque &other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; return *this; } // copy assign
        inline Torque & operator= (Torque &&other) noexcept { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline Torque operator- () const { return Torque(-_unit_direction); }
        inline Torque operator+ (const float& in) const { return Torque(_magnitude+in, _unit_direction); }
        inline Torque operator- (const float& in) const { return Torque(_magnitude-in, _unit_direction); }
        inline Torque operator* (const float& in) const { return Torque(_magnitude*in, _unit_direction); }
        inline Torque operator/ (const float& in) const { return Torque(_magnitude/in, _unit_direction); }
        inline Torque operator+ (const Torque & in) const { return Torque(GetVector() + in.GetVector()); }
        inline Torque operator- (const Torque & in) const { return Torque(GetVector() - in.GetVector()); }
        inline Torque operator* (const Torque & in) const { return Torque(GetVector() * in.GetVector()); }
        inline Torque operator/ (const Torque & in) const { return Torque(GetVector() / in.GetVector()); }
        inline Torque & operator+= (const float& in) { *this = *this + in; return *this; }
        inline Torque & operator-= (const float& in) { *this = *this - in; return *this; }
        inline Torque & operator*= (const float& in) { *this = *this * in; return *this; }
        inline Torque & operator/= (const float& in) { *this = *this / in; return *this; }
        inline Torque & operator+= (const Torque & in) { *this = *this + in; return *this; } 
        inline Torque & operator-= (const Torque & in) { *this = *this - in; return *this; }
        inline Torque & operator*= (const Torque & in) { *this = *this * in; return *this; }
        inline Torque & operator/= (const Torque & in) { *this = *this / in; return *this; }
        // Init/Start/Stop/Destroy
        // Functionality
        bool                                IsZero() const { return _magnitude == 0.f; }
        bool                                IsOrNearZero() const { return _magnitude <= 1.0e-5f; }
        // Accessors
        const auto &                        Get_magnitude() const { return _magnitude; }
        auto &                              Get_magnitude() { return _magnitude; }
        const auto &                        Get_unit_direction() const { return _unit_direction; }
        auto &                              Get_unit_direction() { return _unit_direction; }
        const float3                        GetVector() const { return _unit_direction * _magnitude; }
        float                               GetValueEN() const { return UnitOfMeasure::Ntolbf * _magnitude; }
        float                               GetValueSI() const { return UnitOfMeasure::N * _magnitude; }
        // Assignments
        // Note: set unit direction before magnitude in case sign of magnitude is switched
        void __vectorcall                   SetVector(const float3& vecotrIN){ _magnitude = float3::Magnitude(vecotrIN); _unit_direction = float3::Normal(vecotrIN); }
        void                                Set_magnitude(const float &_magnitude_IN) { _magnitude = abs(_magnitude_IN); if (_magnitude != _magnitude_IN) { _unit_direction = -_unit_direction; }; }
        void __vectorcall                   Set_unit_direction(const float3 &_unit_direction_IN) { _unit_direction = float3::Normal(_unit_direction_IN); }
    
        // Input & Output functions that can have access to protected & private data
        friend std::ostream& operator<< (std::ostream& os, const Torque& in);
        friend std::istream& operator>> (std::istream& is, Torque& out);
        friend std::wostream& operator<< (std::wostream& os, const Torque& in);
        friend std::wistream& operator>> (std::wistream& is, Torque& out);
        friend void to_json(json& j, const Torque& from);
        friend void from_json(const json& j, Torque& to);
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const Torque& in);
    std::istream& operator>> (std::istream& is, Torque& out);
    std::wostream& operator<< (std::wostream& os, const Torque& in);
    std::wistream& operator>> (std::wistream& is, Torque& out);
    void to_json(json& j, const Torque& from);
    void from_json(const json& j, Torque& to);
}

