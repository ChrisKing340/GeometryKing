/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          Rotation    

Description:    Class to keep distance in 3 dimensions.
Symbol:         𝜃 or 𝛷

Usage:          Rotation(const UnitOfMeasure::Time &t, const AngularVelocity & velIn)             

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
#include "..\Physics\AngularAcceleration.h"
#include "..\Physics\AngularVelocity.h"
// 3rdPart namespace
#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

namespace King {

    /******************************************************************************
    *    Rotation
    ******************************************************************************/
    class Rotation;
    Rotation operator*(const UnitOfMeasure::Time &t, const AngularVelocity & velIn); 
    Rotation operator*(const AngularVelocity & velIn, const UnitOfMeasure::Time &t); 

    class alignas(16) Rotation
    {
        /* variables */
    public:
    protected:
        UnitOfMeasure::Angle                _magnitude; // absolute, >= 0
        float3                              _unit_direction; // RotXYZ
    private:
        Quaternion                          _rotation;
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<Rotation> Create() { return std::make_shared<Rotation>(); }
        Rotation() = default;
        explicit Rotation(const AngularAcceleration alphaIn, const UnitOfMeasure::TimeSq dtSqIn) { SetFrom(alphaIn, dtSqIn); }
        explicit Rotation(const AngularVelocity omegaIn, const UnitOfMeasure::Time dtIn) { SetFrom(omegaIn, dtIn); }
        explicit Rotation(const Quaternion& quatIn) { Set(quatIn); }
        explicit Rotation(const float& magIn, const float3& dirIn) { _magnitude = abs(magIn); _unit_direction = float3::Normal(dirIn); if (_magnitude != magIn) { _unit_direction = -_unit_direction; }; CalculateQuat(); }
        explicit Rotation(const UnitOfMeasure::Angle& l, const float3& eulerIn) { _magnitude = abs(l); _unit_direction = float3::Normal(eulerIn); if (_magnitude != l) { _unit_direction = -_unit_direction; }; CalculateQuat(); }
        Rotation(const float3& vectorIn) { _magnitude = float3::Magnitude(vectorIn); _unit_direction = float3::Normal(vectorIn); CalculateQuat(); }
        Rotation(const Rotation &in) { *this = in; } // forward to copy assignment
        Rotation(Rotation &&in) noexcept { *this = std::move(in); } // forward to move assignment

        ~Rotation() { ; }

        static const std::string Unit() { return UnitOfMeasure::Angle::_unit; }
        static const std::wstring UnitW() { return UnitOfMeasure::Angle::_wunit; }

        // Conversions
        inline explicit operator float() const { return _magnitude; }
        inline explicit operator UnitOfMeasure::Angle() const { return _magnitude; }
        inline operator Quaternion() const { return _rotation; }
        inline operator float3() const { return GetVector(); }  // allow implicit for a default behavior
        inline operator DirectX::XMFLOAT3() const { return GetVector().Get_XMFLOAT3(); }
        inline operator DirectX::XMFLOAT3A() const { return GetVector().Get_XMFLOAT3A(); }
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<Rotation*>(p)); }
        inline Rotation& operator= (const Rotation& other) { _magnitude = other._magnitude; _unit_direction = other._unit_direction; _rotation = other._rotation; return *this; } // copy assign
        inline Rotation & operator= (Rotation &&other) noexcept { std::swap(_magnitude, other._magnitude); std::swap(_unit_direction, other._unit_direction); std::swap(_rotation, other._rotation); return *this; } // move assign
        explicit operator bool() const { return (bool)_magnitude && (bool)_unit_direction; } // valid
        bool operator !() const { return !(bool)_magnitude || !(bool)_unit_direction; } // invalid
        // Math Operators
        inline Rotation operator- () const { return Rotation(-_unit_direction); }
        inline Rotation operator+ (const Rotation & in) const { return Rotation(GetQuaternion() + in.GetQuaternion()); }
        inline Rotation operator- (const Rotation & in) const { return Rotation(GetQuaternion() - in.GetQuaternion()); }
        inline Rotation operator* (const Rotation & in) const { return Rotation(GetQuaternion() * in.GetQuaternion()); }
        inline Rotation operator/ (const Rotation & in) const { return Rotation(GetQuaternion() / in.GetQuaternion()); }
        inline Rotation operator+ (const Quaternion& in) const { return Rotation(GetQuaternion() + in); }
        inline Rotation operator* (const Quaternion& in) const { return Rotation(GetQuaternion() * in); }
        inline Rotation & operator+= (const Rotation & in) { *this = *this + in; return *this; }
        inline Rotation & operator-= (const Rotation & in) { *this = *this - in; return *this; }
        inline Rotation & operator*= (const Rotation & in) { *this = *this * in; return *this; } // Rotation squared; we added an operator 4/18 to return Area, so delete the * operator here?
        inline Rotation & operator/= (const Rotation & in) { *this = *this / in; return *this; }
        inline Rotation operator* (const float & in) const { return Rotation(_magnitude * in, _unit_direction); }
        // Comparators
        inline bool operator== (const Rotation& rhs) const { return DirectX::XMVector4Equal(_rotation, rhs._rotation); }
        inline bool operator!= (const Rotation& rhs) const { return DirectX::XMVector4NotEqual(_rotation, rhs._rotation); }
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
        const Quaternion&                   GetQuaternion() const { return _rotation; }
        Quaternion&                         GetQuaternion() { return _rotation; }
        float                               GetValueEN() const { return UnitOfMeasure::mToft * _magnitude; }
        float                               GetValueSI() const { return UnitOfMeasure::m * _magnitude; }
        // Assignments
        // Note: set unit direction before magnitude in case sign of magnitude is switched
        void                                Set(const Quaternion& quatIn) { _rotation = quatIn; float3 eulerXYZ(quatIn.GetEulerAngles()); _magnitude = float3::Magnitude(eulerXYZ); _unit_direction = float3::Normal(eulerXYZ); }
        void                                Set_magnitude(const float& _magnitude_IN_m) { _magnitude = abs(_magnitude_IN_m); if (_magnitude != _magnitude_IN_m) { _unit_direction = -_unit_direction; }; CalculateQuat(); }
        void __vectorcall                   Set_unit_direction(const float3& _unit_direction_IN) { _unit_direction = float3::Normal(_unit_direction_IN); CalculateQuat(); }
        
        void                                SetFrom(const AngularAcceleration alphaIn, const UnitOfMeasure::TimeSq tSqIn); // double integration
        void                                SetFrom(const AngularVelocity omegaIn, const UnitOfMeasure::Time tIn); // single integration

        // Input & Output functions that can have access to protected & private data
        friend std::ostream& operator<< (std::ostream& os, const Rotation& in);
        friend std::istream& operator>> (std::istream& is, Rotation& out);
        friend std::wostream& operator<< (std::wostream& os, const Rotation& in);
        friend std::wistream& operator>> (std::wistream& is, Rotation& out);
        friend void to_json(json& j, const Rotation& from);
        friend void from_json(const json& j, Rotation& to);
    private:
        inline void                         CalculateQuat() { _rotation = Quaternion(float3(_magnitude * _unit_direction).Get_XMFLOAT3()); }
    };
    // Input & Output Function forward declarations
    std::ostream& operator<< (std::ostream& os, const Rotation& in);
    std::istream& operator>> (std::istream& is, Rotation& out);
    std::wostream& operator<< (std::wostream& os, const Rotation& in);
    std::wistream& operator>> (std::wistream& is, Rotation& out);
    void to_json(json& j, const Rotation& from);
    void from_json(const json& j, Rotation& to);
}
