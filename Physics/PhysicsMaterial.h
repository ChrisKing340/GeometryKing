/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:  PhysicsMaterial    

Description:    Basic properties for physics simulation of a material

Usage:      Call Set_...(float ) methods to define specific properties
            default properties are for steel

Contact:    ChrisKing340@gmail.com

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
#include <memory>
#include <string>
#include <iomanip>
// King namespace
#include "..\MathSIMD\MathSIMD.h"
#include "..\Physics\UnitOfMeasure.h"
#include "..\Physics\Acceleration.h"
#include "..\Physics\Velocity.h"
#include "..\Physics\Distance.h"
#include "..\Physics\Position.h"
// 3rdPart namespace
#include "..\..\json\single_include\nlohmann\json.hpp"
using json = nlohmann::json; // for convenience

namespace King {

    class alignas(16) PhysicsMaterial
    {
        /* variables */
    public:
    protected:
        float _density                       = 7870.0f; // Iron in kg/m^3
        float _coefficientOfRestitution      = 0.65f; // 0.65 Steel
        float _coefficientOfStaticFriction   = 0.25f; // 0.51 Steel
        float _coefficientOfKineticFriction  = 0.20f; // 0.44 Steel
        float _coefficientOfDampening        = 0.0f;
        
    private:
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<PhysicsMaterial> Create() { return std::make_shared<PhysicsMaterial>(); }
        PhysicsMaterial() = default;
        PhysicsMaterial(const PhysicsMaterial &in) { *this = in; } // forward to copy assignment
        PhysicsMaterial(PhysicsMaterial &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~PhysicsMaterial() { ; }

        // Conversions
        // Comparators
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<PhysicsMaterial*>(p)); }
        inline PhysicsMaterial& operator= (const PhysicsMaterial& other) = default; // copy assign
        inline PhysicsMaterial& operator= (PhysicsMaterial&& other) = default; // move assign
        // Math Operators
        // Init/Start/Stop/Destroy
        // Functionality
        // Accessors
        auto& Get_density() const { return _density; }
        auto& Get_coefficientOfRestitution() const { return _coefficientOfRestitution; }
        auto& Get_coefficientOfStaticFriction() const { return _coefficientOfStaticFriction; }
        auto& Get_coefficientOfKineticFriction() const { return _coefficientOfKineticFriction; }
        auto& Get_coefficientOfDampening() const { return _coefficientOfDampening; }
        // Assignments
        void Set_density(const float& densityIn) { _density = densityIn; }
        void Set_coefficientOfRestitution(const float& coeffIn) { _coefficientOfRestitution = coeffIn; }
        void Set_coefficientOfStaticFriction(const float& coeffIn) { _coefficientOfStaticFriction = coeffIn; }
        void Set_coefficientOfKineticFriction(const float& coeffIn) { _coefficientOfKineticFriction = coeffIn; }
        void Set_coefficientOfDampening(const float& coeffIn) { _coefficientOfDampening = coeffIn; }
        // Friends
        friend void to_json(json& j, const PhysicsMaterial& from);
        friend void from_json(const json& j, PhysicsMaterial& to);
    protected:
        // Internal Helpers
        //𝜇
    };
    // I/O Functions
    std::ostream& operator<< (std::ostream& os, const PhysicsMaterial& in);
    std::wostream& operator<< (std::wostream& os, const PhysicsMaterial& in);
    std::istream& operator>> (std::istream& is, PhysicsMaterial& out);
    std::wistream& operator>> (std::wistream& is, PhysicsMaterial& out);
    void to_json(json& j, const PhysicsMaterial& from);
    void from_json(const json& j, PhysicsMaterial& to);
}

inline std::ostream& King::operator<<(std::ostream& os, const PhysicsMaterial& in)
{
    return os << "{ den:" << in.Get_density() << ", res:" << in.Get_coefficientOfRestitution() << ", mu_s:" << in.Get_coefficientOfStaticFriction() << ", mu_k:" << in.Get_coefficientOfKineticFriction() << ", damp:" << in.Get_coefficientOfDampening() << " }"; // text out
}
inline std::wostream& King::operator<<(std::wostream& os, const PhysicsMaterial& in)
{
    return os << L"{ 𝜌:" << in.Get_density() << L", e:" << in.Get_coefficientOfRestitution() << L", 𝜇s:" << in.Get_coefficientOfStaticFriction() << L", 𝜇k:" << in.Get_coefficientOfKineticFriction() << L", ζ:" << in.Get_coefficientOfDampening() << L" }"; // text out
}
inline std::istream& King::operator>>(std::istream& is, PhysicsMaterial& out)
{
    float den, res, mu_s, mu_k, z;
    is >> den >> res >> mu_s >> mu_k >> z; // binary in

    out.Set_density(den);
    out.Set_coefficientOfRestitution(res);
    out.Set_coefficientOfStaticFriction(mu_s);
    out.Set_coefficientOfKineticFriction(mu_k);
    out.Set_coefficientOfDampening(z);

    return is;
}
inline std::wistream& King::operator>>(std::wistream& is, PhysicsMaterial& out)
{
    float 𝜌, e, 𝜇s, 𝜇k, ζ;
    is >> 𝜌 >> e >> 𝜇s >> 𝜇k >> ζ; // binary in

    out.Set_density(𝜌);
    out.Set_coefficientOfRestitution(e);
    out.Set_coefficientOfStaticFriction(𝜇s);
    out.Set_coefficientOfKineticFriction(𝜇k);
    out.Set_coefficientOfDampening(ζ);

    return is;
}
inline void King::to_json(json& j, const PhysicsMaterial& from)
{
    j = json { 
        {"den", from._density},
        {"res", from._coefficientOfRestitution},
        {"mu_s", from._coefficientOfStaticFriction},
        {"mu_k", from._coefficientOfKineticFriction},
        {"z", from._coefficientOfDampening}
    }; 
}
inline void King::from_json(const json& j, PhysicsMaterial& to)
{
    j.at("den").get_to(to._density);
    j.at("res").get_to(to._coefficientOfRestitution);
    j.at("mu_s").get_to(to._coefficientOfStaticFriction);
    j.at("mu_k").get_to(to._coefficientOfKineticFriction);
    j.at("z").get_to(to._coefficientOfDampening);
}