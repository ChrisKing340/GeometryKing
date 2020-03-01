#include "MathSIMD.h"

using namespace King;
using namespace std;

/******************************************************************************
*   Streams
******************************************************************************/
std::ostream& King::operator<< (std::ostream& os, const King::UIntPoint2& in) { return os << "{ " << "x: " << in.GetX() << " y: " << in.GetY() << " }"; }
std::ostream& King::operator<< (std::ostream& os, const King::IntPoint2& in) { return os << "{ " << "x: " << in.GetX() << " y: " << in.GetY() << " }"; }
std::ostream& King::operator<< (std::ostream& os, const King::IntPoint3& in) { return os << "{ " << "x: " << in.GetX() << " y: " << in.GetY() << " z: " << in.GetZ() << " }"; }
std::ostream& King::operator<< (std::ostream& os, const King::FloatPoint2& in) { return os << "{ " << "x: " << setw(9) << in.f[0] << " y: " << setw(9) << in.f[1] << " }"; }
std::ostream& King::operator<< (std::ostream& os, const King::FloatPoint3& in) { return os << "{ " << "x: " << setw(9) << in.f[0] << " y: " << setw(9) << in.f[1] << " z: " << setw(9) << in.f[2] << " }"; }
std::ostream& King::operator<< (std::ostream& os, const King::FloatPoint4& in) { return os << "{ " << "x: " << setw(9) << in.f[0] << " y: " << setw(9) << in.f[1] << " z: " << setw(9) << in.f[2] << " w: " << setw(9) << in.f[3] << " }"; }

std::wostream& King::operator<< (std::wostream& os, const King::UIntPoint2& in) { return os << L"{ " << L"x: " << in.GetX() << L" y: " << in.GetY() << L" }"; }
std::wostream& King::operator<< (std::wostream& os, const King::IntPoint2& in) { return os << L"{ " << L"x: " << in.GetX() << L" y: " << in.GetY() << L" }"; }
std::wostream& King::operator<< (std::wostream& os, const King::IntPoint3& in) { return os << L"{ " << L"x: " << in.GetX() << L" y: " << in.GetY() << L" z: " << in.GetZ() << L" }"; }
std::wostream& King::operator<< (std::wostream& os, const King::FloatPoint2& in) { return os << L"{ " << L"x: " << setw(9) << in.f[0] << L" y: " << setw(9) << in.f[1] << L" }"; }
std::wostream& King::operator<< (std::wostream& os, const King::FloatPoint3& in) { return os << L"{ " << L"x: " << setw(9) << in.f[0] << L" y: " << setw(9) << in.f[1] << L" z: " << setw(9) << in.f[2] << L" }"; }
std::wostream& King::operator<< (std::wostream& os, const King::FloatPoint4& in) { return os << L"{ " << L"x: " << setw(9) << in.f[0] << L" y: " << setw(9) << in.f[1] << L" z: " << setw(9) << in.f[2] << L" w: " << setw(9) << in.f[3] << L" }"; }

std::istream& King::operator>> (std::istream& is, King::UIntPoint2& in) { unsigned int ui[2]; is >> ui[0] >> ui[1]; in.Set(ui[0], ui[1]); return is; }
std::istream& King::operator>> (std::istream& is, IntPoint2& in) { int i[2]; is >> i[0] >> i[1]; in.Set(i[0], i[1]); return is; }
std::istream& King::operator>> (std::istream& is, FloatPoint2& in) { DirectX::XMFLOAT2 f; is >> f.x >> f.y; in.Set(f); return is; }
std::istream& King::operator>> (std::istream& is, FloatPoint3& in) { DirectX::XMFLOAT3 f; is >> f.x >> f.y >> f.z; in.Set(f); return is; }
std::istream& King::operator>> (std::istream& is, FloatPoint4& in) { DirectX::XMFLOAT4 f; is >> f.x >> f.y >> f.z >> f.w; in.Set(f); return is; }

std::wistream& King::operator>> (std::wistream& is, King::UIntPoint2& in) { unsigned int ui[2]; is >> ui[0] >> ui[1]; in.Set(ui[0], ui[1]); return is; }
std::wistream& King::operator>> (std::wistream& is, IntPoint2& in) { int i[2]; is >> i[0] >> i[1]; in.Set(i[0], i[1]); return is; }
std::wistream& King::operator>> (std::wistream& is, FloatPoint2& in) { DirectX::XMFLOAT2 f; is >> f.x >> f.y; in.Set(f); return is; }
std::wistream& King::operator>> (std::wistream& is, FloatPoint3& in) { DirectX::XMFLOAT3 f; is >> f.x >> f.y >> f.z; in.Set(f); return is; }
std::wistream& King::operator>> (std::wistream& is, FloatPoint4& in) { DirectX::XMFLOAT4 f; is >> f.x >> f.y >> f.z >> f.w; in.Set(f); return is; }

/******************************************************************************
*   json
******************************************************************************/
void King::to_json(json& j, const UIntPoint2& from) { j = json{ {"x", from.u[0]}, {"y", from.u[1]} }; }
void King::to_json(json& j, const IntPoint2& from) { j = json{ {"x", from.i[0]}, {"y", from.i[1]} }; }
void King::to_json(json& j, const IntPoint3& from) { j = json{ {"x", from.i[0]}, {"y", from.i[1]}, {"z", from.i[2]} }; }
void King::to_json(json& j, const FloatPoint2& from) { j = json{ {"x", from.f[0]}, {"y", from.f[1]} }; }
void King::to_json(json& j, const FloatPoint3& from) { j = json{ {"x", from.f[0]}, {"y", from.f[1]}, {"z", from.f[2]} }; }
void King::to_json(json& j, const FloatPoint4& from) { j = json{ {"x", from.f[0]}, {"y", from.f[1]}, {"z", from.f[2]}, {"w", from.f[3]} }; }
void King::to_json(json& j, const Quaternion& from) { j = json{ {"x", from.f[0]}, {"y", from.f[1]}, {"z", from.f[2]}, {"w", from.f[3]} }; }

void King::from_json(const json& j, UIntPoint2& to) { j.at("x").get_to(to.u[0]); j.at("y").get_to(to.u[1]); }
void King::from_json(const json& j, IntPoint2& to) { j.at("x").get_to(to.i[0]); j.at("y").get_to(to.i[1]); }
void King::from_json(const json& j, IntPoint3& to) { j.at("x").get_to(to.i[0]); j.at("y").get_to(to.i[1]); j.at("z").get_to(to.i[2]); }
void King::from_json(const json& j, FloatPoint2& to) { j.at("x").get_to(to.f[0]); j.at("y").get_to(to.f[1]); }
void King::from_json(const json& j, FloatPoint3& to) { j.at("x").get_to(to.f[0]); j.at("y").get_to(to.f[1]); j.at("z").get_to(to.f[2]); }
void King::from_json(const json& j, FloatPoint4& to) { j.at("x").get_to(to.f[0]); j.at("y").get_to(to.f[1]); j.at("z").get_to(to.f[2]); j.at("w").get_to(to.f[3]); }
void King::from_json(const json& j, Quaternion& to) { j.at("x").get_to(to.f[0]); j.at("y").get_to(to.f[1]); j.at("z").get_to(to.f[2]); j.at("w").get_to(to.f[3]); }

/******************************************************************************
*   Math functions and methods
******************************************************************************/
inline King::IntPoint3::IntPoint3(const FloatPoint3& in)
{
    auto f = in.Get_XMFLOAT3();
    i[0] = static_cast<int>(f.x);
    i[1] = static_cast<int>(f.y);
    i[2] = static_cast<int>(f.z);
}

inline King::FloatPoint3::FloatPoint3(FloatPoint4 vecIn)
{
    v = DirectX::XMVectorSelect(DirectX::g_XMOne.v, vecIn, DirectX::g_XMSelect1110.v);
}

// Store the Euler angles in radians, credit to:
// http://www.gamedev.net/topic/597324-quaternion-to-euler-angles-and-back-why-is-the-rotation-changing/
DirectX::XMFLOAT3 King::Quaternion::GetEulerAngles() const
{
    // early exit test since many rotations in our engine will be zero
    float3 zero(DirectX::XMVectorZero());
    if (DirectX::XMVector3Equal(v, zero))
    {
        return zero;
    }

    DirectX::XMFLOAT3 pitchYawRoll; // output

    float4 sqVec = DirectX::XMVectorMultiply(v, v);
    DirectX::XMFLOAT4 sq = sqVec;

    DirectX::XMFLOAT4 q;
    DirectX::XMStoreFloat4(&q, v);

    // If quaternion is normalised the unit is one, otherwise it is the correction factor
    //float unit = sqx + sqy + sqz + sqw; // = 1.0f if was normalized
    float unit = float4::SumComponents(sqVec);
    float test = q.x * q.y + q.z * q.w;

    if (test > 0.499f * unit)
    {
        // Singularity at north pole
        pitchYawRoll.x = -DirectX::XM_PIDIV2;
        pitchYawRoll.y = -2.f * atan2(q.x, q.w);
        pitchYawRoll.z = 0.f;
        return pitchYawRoll;
    }
    else if (test < -0.499f * unit)
    {
        // Singularity at south pole
        pitchYawRoll.x = DirectX::XM_PIDIV2;
        pitchYawRoll.y = 2.f * atan2(q.x, q.w);
        pitchYawRoll.z = 0.f;
        return pitchYawRoll;
    }
    pitchYawRoll.x = -asin(2.f * test / unit);
    pitchYawRoll.y = atan2(2.f * (q.y * q.w - q.x * q.z), sq.x - sq.y - sq.z + sq.w);
    pitchYawRoll.z = -atan2(2.f * (q.x * q.w - q.y * q.z), -sq.x + sq.y - sq.z + sq.w);

    return pitchYawRoll;

}

DirectX::XMFLOAT3 King::Quaternion::CalculateAngularVelocity(const Quaternion prevRotation, float deltaTime) const
{
    // Calculate the angular velocity from a (delta rotation) / (unit time), credit to:
    // DavidSWu at https://forum.unity.com/threads/manually-calculate-angular-velocity-of-gameobject.289462/
    auto delta = *this - prevRotation;
    if (!delta) return delta; // zero

    float gain;
    float rotation = delta.GetW();
    // handle negative angles
    if (rotation < 0.0f)
    {
        const auto halfAngle = acos(-rotation);
        gain = -2.0f * halfAngle / (sin(halfAngle) * deltaTime);
    }
    else
    {
        const auto halfAngle = acos(rotation);
        gain = 2.0f * halfAngle / (sin(halfAngle) * deltaTime);
    }
    return float3(delta) * gain;
}

// Assignments

void King::Quaternion::SetAxisAngle(const float3& vector, const float& angleRadians)
{
    if (XMVector3Equal(vector, DirectX::XMVectorZero()))
    {
        v = vector;
        SetW(1.0f);
    }
    else
    {
        v = DirectX::XMQuaternionRotationAxis(vector, angleRadians);
        Validate();
    }
}

void King::Quaternion::Set(const float3 vFrom, const float3 vTo)
{
    float3 ave = vFrom + vTo;
    bool success = (float3::Magnitude(ave) > 1.192092896e-7f); // minimum float value
    if (success)
    {
        ave.MakeNormalize();
        *this = vFrom * ave;
        v = float3::Normal(float3::CrossProduct(vFrom, ave));
        SetW(-float3::DotProduct(vFrom, ave).GetX()); // not sure about the - sign
    }
    else
    {
        // angle is either 0 or 180 degrees
        if (vFrom == vTo)
            v = DirectX::XMQuaternionIdentity(); // 0 degrees
        else
        {
            v = float3::Normal(vFrom); // axis
            SetW(0.f); // 180 degree.
        }
    }
}
