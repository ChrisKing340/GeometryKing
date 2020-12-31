/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:            Distance    

Description:    Class to keep distance in 3 dimensions.

Usage:            Distance(const UnitOfMeasure::Time &t, const Velocity & velIn)             

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

#pragma once // needed for C++17 which had breaking change for std::make_shared
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
#include "..\Physics\Acceleration.h"
#include "..\Physics\Velocity.h"
// 3rdPart namespace
#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

/******************************************************************************
*    Kinematics
*
*   d = Pf - Pi
*   d = vi * t + 1/2 * a * t^2
*   d = vf * t - 1/2 * a * t^2
*
******************************************************************************/

namespace King {

    /******************************************************************************
    *    Distance
    ******************************************************************************/
    class Distance;
    Distance operator*(const UnitOfMeasure::Time &t, const Velocity & velIn); // d = t * v
    Distance operator*(const Velocity & velIn, const UnitOfMeasure::Time &t); // d = v * t 
    UnitOfMeasure::Energy operator*(const Force& fIn, const Distance& dIn);
    UnitOfMeasure::Energy operator*(const Distance& dIn, const Force& fIn);

    class alignas(16) Distance
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
        static std::shared_ptr<Distance> Create() { return std::make_shared<Distance>(); }
        Distance() = default;
        explicit Distance(const float &magIn, const float3 &dirIn) { _magnitude = abs(magIn); _unit_direction = float3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit Distance(const UnitOfMeasure::Length &l, const float3 &dirIn) { _magnitude = abs(l); _unit_direction = float3::Normal(dirIn); if (_magnitude != l) { _unit_direction = -_unit_direction; }; }
        Distance(const float3 &vectorIn) { _magnitude = float3::Magnitude(vectorIn); _unit_direction = float3::Normal(vectorIn); }
        Distance(const float3& pt1In, const float3& pt2In) { auto AB=pt2In-pt1In; _magnitude = float3::Magnitude(AB); _unit_direction = float3::Normal(AB); }
        explicit Distance(const Acceleration & accIn, const UnitOfMeasure::Time &t) { _magnitude = UnitOfMeasure::Length(static_cast<float>(accIn.Get_magnitude()) * 0.5f * (float)t * (float)t); _unit_direction = accIn.Get_unit_direction(); }
        explicit Distance(const Velocity & velIn, const UnitOfMeasure::Time &t) { _magnitude = UnitOfMeasure::Length(static_cast<float>(velIn.Get_magnitude()) * t); _unit_direction = velIn.Get_unit_direction(); }
        Distance(const Distance &in) { *this = in; } // forward to copy assignment
        Distance(Distance &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~Distance() { ; }

        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::Length() const { return _magnitude; }
        inline operator float3() const { return GetVector(); }  // allow implicit for a default behavior
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Distance*>(p)); }
        inline Distance & operator= (const Distance &other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; return *this; } // copy assign
        inline Distance & operator= (Distance &&other) noexcept { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline Distance operator- () const { return Distance(_magnitude, -_unit_direction); }
        inline Distance operator+ (const Distance & in) const { return Distance(GetVector() + in.GetVector()); }
        inline Distance operator- (const Distance & in) const { return Distance(GetVector() - in.GetVector()); }
        inline Distance operator* (const Distance & in) const { return Distance(GetVector() * in.GetVector()); }
        inline Distance operator/ (const Distance & in) const { return Distance(GetVector() / in.GetVector()); }
        inline Distance & operator+= (const Distance & in) { *this = *this + in; return *this; }
        inline Distance & operator-= (const Distance & in) { *this = *this - in; return *this; }
        inline Distance & operator*= (const Distance & in) { *this = *this * in; return *this; } // Distance squared; we added an operator 4/18 to return Area, so delete the * operator here?
        inline Distance & operator/= (const Distance & in) { *this = *this / in; return *this; }
        inline Distance operator* (const float & in) const { return Distance(_magnitude * in, _unit_direction); }
        // Init/Start/Stop/Destroy
        // Functionality
        // Accessors
        const auto&                         Get_magnitude() const { return _magnitude; }
        auto&                               Get_magnitude() { return _magnitude; }
        const auto&                         Get_unit_direction() const { return _unit_direction; }
        auto&                               Get_unit_direction() { return _unit_direction; }
        const float3                        GetVector() const { return _unit_direction * _magnitude; }
        float                               GetValueEN() const { return UnitOfMeasure::mToft * _magnitude; }
        float                               GetValueSI() const { return UnitOfMeasure::m * _magnitude; }
        // Assignments
        // Note: set unit direction before magnitude in case sign of magnitude is switched
        void                                Set_magnitude(const float &_magnitude_IN_m) { _magnitude = abs(_magnitude_IN_m); if (_magnitude != _magnitude_IN_m) { _unit_direction = -_unit_direction; }; }
        void __vectorcall                   Set_unit_direction(const float3 &_unit_direction_IN) { _unit_direction = float3::Normal(_unit_direction_IN); }
        // Input & Output functions that can have access to protected & private data
        friend std::ostream& operator<< (std::ostream& os, const Distance& in);
        friend std::istream& operator>> (std::istream& is, Distance& out);
        friend std::wostream& operator<< (std::wostream& os, const Distance& in);
        friend std::wistream& operator>> (std::wistream& is, Distance& out);
        friend void to_json(json& j, const Distance& from);
        friend void from_json(const json& j, Distance& to);
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const Distance& in);
    std::istream& operator>> (std::istream& is, Distance& out);
    std::wostream& operator<< (std::wostream& os, const Distance& in);
    std::wistream& operator>> (std::wistream& is, Distance& out);
    void to_json(json& j, const Distance& from);
    void from_json(const json& j, Distance& to);
}
