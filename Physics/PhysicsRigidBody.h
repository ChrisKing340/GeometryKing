/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Title:          PhysicsRigidBody    

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
#include "PhysicsMaterial.h"
#include "PhysicsState.h"
// 3rdPart namespace
#include "..\3rdParty\json.hpp"
using json = nlohmann::json;

namespace King {

    class alignas(16) PhysicsRigidBody
    {
        /* variables */
    public:
        std::vector<std::pair<Force, Distance>> _forcesActingOnBody; // after update and moved into velocity, this vector is cleared
        // *** TO DO *** move HasDrag() and HasGravity() from MoveableObj to this Class as force vectors and use data from this class
    protected:
        bool                                    _stationary = false;

        std::shared_ptr<PhysicsMaterial>        _sp_material;
        float                                   _coefficientOfDrag = 0.f;
        float                                   _crossSectionalArea = 1.f; // in direction of motion
        float                                   _volume = 0.f; // m^3
        UnitOfMeasure::Mass                     _mass; // kg
        Position                                _centerOfMassLocalSpace;  // affinity transform, set Pose rotation center *** TO DO ***

    private:
        PhysicsState                            i; // initial before Update(const UnitOfMeasure::Time& dt)
        PhysicsState                            f; // final after Update(const UnitOfMeasure::Time& dt)
        /* methods */
    public:
        // Creation/Life cycle
        static std::shared_ptr<PhysicsRigidBody> Create() { return std::make_shared<PhysicsRigidBody>(); }
        PhysicsRigidBody() { /*TODO: define i*/; }
        PhysicsRigidBody(const PhysicsRigidBody &in) { *this = in; } // forward to copy assignment
        PhysicsRigidBody(PhysicsRigidBody &&in) noexcept { *this = std::move(in); } // forward to move assignment

        virtual ~PhysicsRigidBody() { ; }

        // Conversions
        // Comparators
        // Operators 
        void * operator new (size_t size) { return _aligned_malloc(size, 16); }
        void   operator delete (void *p) { _aligned_free(static_cast<PhysicsRigidBody*>(p)); }
        inline PhysicsRigidBody& operator= (const PhysicsRigidBody& other) = default; // copy assign
        inline PhysicsRigidBody& operator= (PhysicsRigidBody&& other) = default; // move assign
        // Math Operators
        // Init/Start/Stop/Destroy
        // Functionality
        virtual void Update(const UnitOfMeasure::Time& dt);
        // Accessors
        // Assignments
        void Set(const std::shared_ptr<PhysicsMaterial> sp_material) { _sp_material = sp_material; }

        void Set_volume(const float& volume) { _volume = volume; }
        void Set_crossSectionalArea(const float& crossSectionalArea) { _crossSectionalArea = crossSectionalArea; }
        void Set_coefficientOfDrag(const float& coefficientOfDrag) { _coefficientOfDrag = coefficientOfDrag; }
        void Set_mass(const UnitOfMeasure::Mass& mass) { _mass = mass; }
        void Set_centerOfMassLocalSpace(const Position& centerOfMassLocalSpace) { _centerOfMassLocalSpace = centerOfMassLocalSpace; }

    protected:
        // Internal Helpers
    };
}