/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          PhysicsState    

Description:    

Usage:            

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
#include "UnitOfMeasure.h" // has own namespace King::UnitOfMesure::
#include "Force.h"
#include "Acceleration.h"
#include "AngularAcceleration.h"
#include "Velocity.h"
#include "AngularVelocity.h"
#include "Rotation.h"
#include "Distance.h"
#include "Position.h"
// 3rdPart namespace
#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

namespace King {

    class alignas(16) PhysicsState
    {
        friend class PhysicsRigidBody;
        /* variables */
    public:
    protected:
        Position                        _positionWorldSpace;
        Acceleration                    _linearAcceleration; // net of all forces
        Velocity                        _linearVelocity;
        Velocity                        _linearMomentum; // g * m/s

        Rotation                        _rotation; // passes through the center of mass.
        AngularAcceleration             _angularAcceleration; // net of all forces
        AngularVelocity                 _angularVelocity;
        AngularVelocity                 _angularMomentum; // I * rad/s
        Position                        _pointOnAxisOfRotationLocalSpace; // when a free body, will be the center of mass
        float3                          _principalMomentsOfInertia; // depends on geometry, use alone when symetry along principal axis and rotation through the principal axis
        float3                          _productsOfInertia; // depends on geometry and axis; vector component x is xy, y is yz, and z is zx;
    private:
        //UnitOfMeasure::Energy           _linearKineticEnergy;
        //UnitOfMeasure::Energy           _angularKineticEnergy;
        //UnitOfMeasure::Energy           _potentialEnergy;
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<PhysicsState> Create() { return std::make_shared<PhysicsState>(); }
        PhysicsState() = default;
        PhysicsState(const PhysicsState &in) { *this = in; } // forward to copy assignment
        PhysicsState(PhysicsState &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~PhysicsState() { ; }

        // Conversions
        // Comparators
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<PhysicsState*>(p)); }
        inline PhysicsState& operator= (const PhysicsState& other) = default; // copy assign
        inline PhysicsState& operator= (PhysicsState&& other) = default; // move assign
        // Math Operators
        // Init/Start/Stop/Destroy
        // Functionality
        virtual void Update(const UnitOfMeasure::Time& dt) { _positionWorldSpace += _linearVelocity * dt; }
        // Accessors
        float3 GetAxisOfRotation() const { return _rotation.GetQuaternion().GetAxis(); }
        // Assignments
        void Set_angularVelocity(const AngularVelocity& angularVelocity) { _angularVelocity = angularVelocity; }
        //void Set_angularMomentum(const AngularVelocity& angularMomentum) { _angularMomentum = angularMomentum; }
        void Set_rotation(const Rotation& rotation) { _rotation = rotation; }
        void Set_principalMomentsOfInertia(const float3& principalMomentsOfInertia) { _principalMomentsOfInertia = principalMomentsOfInertia; }
        void Set_productsOfInertia(const float3& productsOfInertia) { _productsOfInertia = productsOfInertia; }

        void Set_linearVelocity(const Velocity& linearVelocity) { _linearVelocity = linearVelocity; }
        //void Set_linearMomentum(const Velocity& linearMomentum) { _linearMomentum = linearMomentum; }
        void Set_positionWorldSpace(const Position& positionWorldSpace) { _positionWorldSpace = positionWorldSpace; }

        friend void to_json(json& j, const PhysicsState& from);
        friend void from_json(const json& j, PhysicsState& to);
    protected:
        // Internal Helpers
    };
    void to_json(json& j, const PhysicsState& from);
    void from_json(const json& j, PhysicsState& to);
}