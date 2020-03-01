/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:            Force    

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

/******************************************************************************
*    Physic Basics
*
*    Newton's 1st Law:
*        An object at rest stays at rest and an object in motion stays in motion
*        UNLESS ACTED UPON BY A NET SUM EXTERNAL FORCE. Known as Law of inertia.
*    
******************************************************************************/

namespace King {

    /******************************************************************************
    *    Force
    ******************************************************************************/
    class alignas(16) Force
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::Strength             _magnitude; // absolute, >= 0
        float3                              _unit_direction;
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Force> Create() { return std::make_shared<Force>(); }
        Force() = default;
        explicit Force(const UnitOfMeasure::Strength s) : _magnitude(abs(s)) { if (_magnitude != s) { _unit_direction = -_unit_direction; }; }
        explicit Force(const float &magIn, const float3 &dirIn) { _magnitude = abs(magIn); _unit_direction = float3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; }
        explicit Force(const UnitOfMeasure::Strength &s, const float3 &dirIn) { _magnitude = abs(s); _unit_direction = float3::Normal(dirIn); if (_magnitude != s) { _unit_direction = -_unit_direction; }; }
        Force(const float3 &vectorIn) { _magnitude = float3::Magnitude(vectorIn); _unit_direction = float3::Normal(vectorIn); }
        Force(const std::vector<Force> & forcesIn) { float3 vec; for (const auto & e : forcesIn) vec += float3(e); *this = Force(vec); }
        Force(const Force &in) { *this = in; } // forward to copy assignment
        Force(Force &&in) { *this = std::move(in); } // forward to move assignment

        virtual ~Force() { ; }

        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::Strength() const { return _magnitude; }
        inline explicit operator float3() const { return GetVector(); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Force*>(p)); }
        inline Force & operator= (const Force &other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; return *this; } // copy assign
        inline Force & operator= (Force &&other) { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline Force operator- () const { return Force(-_unit_direction); }
        inline Force operator+ (const Force & in) const { return Force(GetVector() + in.GetVector()); }
        inline Force operator- (const Force & in) const { return Force(GetVector() - in.GetVector()); }
        inline Force operator* (const Force & in) const { return Force(GetVector() * in.GetVector()); }
        inline Force operator/ (const Force & in) const { return Force(GetVector() / in.GetVector()); }
        inline Force & operator+= (const Force & in) { *this = *this + in; return *this; } 
        inline Force & operator-= (const Force & in) { *this = *this - in; return *this; }
        inline Force & operator*= (const Force & in) { *this = *this * in; return *this; }
        inline Force & operator/= (const Force & in) { *this = *this / in; return *this; }
        // Init/Start/Stop/Destroy
        // Functionality
        // Accessors
        const auto &                        Get_magnitude() const { return _magnitude; }
        const auto &                        Get_unit_direction() const { return _unit_direction; }
        const float3                        GetVector() const { return _unit_direction * _magnitude; }
        float                               GetValueEN() const { return UnitOfMeasure::Ntolbf * _magnitude; }
        float                               GetValueSI() const { return UnitOfMeasure::N * _magnitude; }
        // Assignments
        // Note: set unit direction before magnitude in case sign of magnitude is switched
        void                                Set_magnitude(const float &_magnitude_IN) { _magnitude = abs(_magnitude_IN); if (_magnitude != _magnitude_IN) { _unit_direction = -_unit_direction; }; }
        void __vectorcall                   Set_unit_direction(const float3 &_unit_direction_IN) { _unit_direction = float3::Normal(_unit_direction_IN); }
    };
}