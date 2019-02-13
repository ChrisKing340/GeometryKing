#include "MathSIMD.h"

using namespace King;
using namespace std;

/******************************************************************************
*	Streams
******************************************************************************/
std::ostream & King::operator<< (std::ostream & os, const King::UIntPoint2 &in) { return os << "{ " << "x: " << in.GetX() << " y: " << in.GetY() << " }"; }
std::ostream & King::operator<< (std::ostream & os, const King::IntPoint2 &in) { return os << "{ " << "x: " << in.GetX() << " y: " << in.GetY() << " }"; }
std::ostream & King::operator<< (std::ostream & os, const King::IntPoint3 &in) { return os << "{ " << "x: " << in.GetX() << " y: " << in.GetY() << " z: " << in.GetZ() << " }"; }
std::ostream & King::operator<< (std::ostream & os, const King::FloatPoint2 &in) { return os << "{ " << "x: " << in.f[0] << " y: " << in.f[1] << " }"; }
std::ostream & King::operator<< (std::ostream & os, const King::FloatPoint3 &in) { return os << "{ " << "x: " << in.f[0] << " y: " << in.f[1] << " z: " << in.f[2] << " }"; }
std::ostream & King::operator<< (std::ostream & os, const King::FloatPoint4 &in) { return os << "{ " << "x: " << in.f[0] << " y: " << in.f[1] << " z: " << in.f[2] << " w: " << in.f[3] << " }"; }
											    
std::istream & King::operator>> (std::istream & is, King::UIntPoint2 &in) { unsigned int ui[2]; is >> ui[0] >> ui[1]; in.Set(ui[0],ui[1]); return is; }
std::istream & King::operator>> (std::istream & is, IntPoint2 &in) { int i[2]; is >> i[0] >> i[1]; in.Set(i[0], i[1]); return is; }
std::istream & King::operator>> (std::istream & is, FloatPoint2 &in) { DirectX::XMFLOAT2 f; is >> f.x >> f.y; in.Set(f); return is; }
std::istream & King::operator>> (std::istream & is, FloatPoint3 &in) { DirectX::XMFLOAT3 f; is >> f.x >> f.y >> f.z; in.Set(f); return is; }
std::istream & King::operator>> (std::istream & is, FloatPoint4 & in) { DirectX::XMFLOAT4 f; is >> f.x >> f.y >> f.z >> f.w; in.Set(f); return is; }

/******************************************************************************
*	Math functions and methods
******************************************************************************/
inline King::IntPoint3::IntPoint3(const FloatPoint3 & in)
{
	auto f = in.Get_XMFLOAT3();
	i[0] = static_cast<int>(f.x);
	i[1] = static_cast<int>(f.y);
	i[2] = static_cast<int>(f.z);
}
inline King::FloatPoint3::FloatPoint3(FloatPoint4 vecIn)
{
	v = vecIn.v;
	DirectX::XMVectorSetW(v, 0.f);
}

DirectX::XMFLOAT3 King::Quaternion::GetRotationInEulerAngles()
{

	// Store the Euler angles in radians, credit to:
	// http://www.gamedev.net/topic/597324-quaternion-to-euler-angles-and-back-why-is-the-rotation-changing/

	XMFLOAT3 pitchYawRoll; // output

	float4 sqVec = DirectX::XMVectorMultiply(v, v);
	XMFLOAT4 sq = sqVec;
	
	XMFLOAT4 q;
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
	pitchYawRoll.y = -atan2(2.f * (q.y * q.w - q.x * q.z), sq.x - sq.y - sq.z + sq.w);
	pitchYawRoll.z = -atan2(2.f * (q.x * q.w - q.y * q.z), -sq.x + sq.y - sq.z + sq.w);

	return pitchYawRoll;

}


