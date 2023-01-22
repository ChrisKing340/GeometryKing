#pragma once
#include "..\3DGeometryKing\3DGeometry.h"

namespace King {
    /******************************************************************************
    *    WorldObject -
    *       Base object for games with position and orientation
    *       Optimized matrix updates on data change
    *       Physics
    ******************************************************************************/
    template <typename T>
    class WorldObject : public Pose
    {
    public:
        T                   _object;
        Box                 _AABB_modelSpace;
        PhysicsRigidBody    _rigidBody;
    protected:
        bool                _dirty = true;
        DirectX::XMMATRIX   _worldMatrix;
        Box                 _AABB_worldSpace;
        // *** TO DO *** Implement AABB tree to minimize collission tests, or use my quad tree? sample: https://github.com/JamesRandall/SimpleVoxelEngine/blob/master/voxelEngine/src/AABBTree.h 
        // for phase III collission tests, do we keep a _OBB, or an _OBB Hierarchy tree?
    public:
        void Set(const T& object) { _object = object; }
        T Get() const { return _object; }
        DirectX::XMMATRIX GetWorldMatrix()
        {
            if (_dirty)
            {
                SetRotation(_rigidBody.GetFinalState().Get_rotation().GetQuaternion());
                GetTranslation(_rigidBody.GetFinalState().Get_positionWorldSpace());
                _worldMatrix = DirectX::XMMatrixAffineTransformation(GetScale(), GetRotationOrigin(), GetRotation(), GetTranslation());
                _dirty = false;
                _AABB_worldSpace = _AABB_modelSpace;
                _AABB_worldSpace.SetAABBfromThisTransformedBox(_worldMatrix);
            }
            return _worldMatrix;
        }
        template <typename T> inline WorldObject& operator= (const WorldObject& in) 
        { 
            _object = in._object;
            _AABB_modelSpace = in._AABB_modelSpace;
            _rigidBody = in._AABB_modelSpace;
            _dirty = true; 
            return Pose::operator=(in); 
        }
        template <typename T> inline WorldObject& operator= (WorldObject&& in) noexcept// move assign
        {
            if (this != &in)
            {
                _object = std::move(in._object);
                _AABB_modelSpace = std::move(in._AABB_modelSpace);
                _rigidBody = std::move(in._AABB_modelSpace);
                _dirty = true;
                Pose::operator=(std::move(in));
            }
            return *this;
        }
        template <typename T> inline WorldObject operator+ (const WorldObject& in) const
        {
            _dirty = true;
            return Pose::operator+(in);
        }
        template <typename T> inline WorldObject operator- (const WorldObject& in) const
        {
            _dirty = true;
            return Pose::operator-(in);
        }
        template <typename T> inline WorldObject operator* (const WorldObject& in) const
        {
            _dirty = true;
            return Pose::operator*(in);
        }
        template <typename T> inline WorldObject& operator+= (const Pose& in) { _dirty = true; *this = *this + in; return *this; }
        template <typename T> inline WorldObject& operator-= (const Pose& in) { _dirty = true; *this = *this - in; return *this; }
        template <typename T> inline WorldObject& operator*= (const Pose& in) { _dirty = true; *this = *this * in; return *this; }
        template <typename T> inline WorldObject operator~ () const
        {
            _dirty = true;
            return Pose::operator~();
        }
        // rotate
        template <typename T> inline WorldObject operator* (const quat& in)
        {
            _dirty = true;
            return Pose::operator*(in);
        }
        template <typename T> inline WorldObject& operator*= (const quat& in)
        {
            _dirty = true;
            *this = *this * in;
            return *this;
        }
        // transform
        template <typename T> inline WorldObject operator* (const float& in) const
        {
            _dirty = true;
            return Pose::operator*(in);
        }
        template <typename T> inline WorldObject& operator*= (const float& in) { _dirty = true; *this = *this * in; return *this; }
        // translate
        template <typename T> inline WorldObject operator+ (const float3& in)
        {
            _dirty = true;
            return Pose::operator+(in);
        }
        template <typename T> inline WorldObject& operator+= (const float3& in) { _dirty = true; *this = *this + in; return *this; }
        template <typename T> inline WorldObject operator- (const float3& in)
        {
            _dirty = true;
            return Pose::operator-(in);
        }
        template <typename T> inline WorldObject& operator-= (const float3& in) { _dirty = true; *this = *this - in; return *this; }
        // vector transformation
        inline float3 operator* (float3 vec) const
        {
            _dirty = true;
            return Pose::operator*(in);
        }
        inline float4 operator* (float4 vec) const
        {
            _dirty = true;
            return Pose::operator*(in);
        }
        // Conversions
        inline operator DirectX::XMMATRIX() const { return GetWorldMatrix(); }
        inline operator DirectX::XMFLOAT4X4() const { return Get_XMFLOAT4X4(); }

        inline DirectX::XMMATRIX            Get_XMMATRIX() const { return GetWorldMatrix(); }
        inline DirectX::XMFLOAT4X4          Get_XMFLOAT4X4() const { DirectX::XMFLOAT4X4 rtn; DirectX::XMStoreFloat4x4(&rtn, GetWorldMatrix()); return rtn; }
        inline void                         Get_boundingBox() { if (_dirty) GetWorldMatrix(); return _boundingBox; }
        // Functionality
        virtual void                        Update(const UnitOfMeasure::Time& dt) { _rigidBody.Update(dt); _dirty = true; }
    };

} // King namespace
