#include "Quaternion.h"

using namespace Manifest_Math;

MFmat3 MFquaternion::GetRotation() const
{
	MFfloat x2 = x * x;
	MFfloat y2 = y * y;
	MFfloat z2 = z * z;
	MFfloat xy = x * y;
	MFfloat xz = x * z;
	MFfloat yz = y * z;
	MFfloat wx = w * x;
	MFfloat wy = w * y;
	MFfloat wz = w * z;	

	return MFmat3 //yup...
	{
		1.0f - 2.0f * (y2 + z2),2.0f * (xy + wz),2.0f * (xz - wy),
		2.0f * (xy - wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz + wx),
		2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (x2 + y2)
	};
}

void MFquaternion::SetRotation(const MFmat3& mA)
{
	const auto& m00 = mA(0, 0);
	const auto& m11 = mA(1, 1);
	const auto& m22 = mA(2, 2);
	const auto& sum = m00 + m11 + m22;

	if (sum >= 0.0f)//can be returned as is
	{
		w = std::sqrtf(sum + 1.0f) * 0.5f;
		const auto& f = 0.25f / w;
		x = (mA(2, 1) - mA(1, 2)) * f;
		y = (mA(0, 2) - mA(2, 0)) * f;
		z = (mA(1, 0) - mA(0, 1)) * f;
	}
	else if (m00 > m11 && m00 > m22)//x leading
	{
		x = std::sqrtf(m00 - m11 - m22 + 1.0f) * 0.5f;
		const auto& f = 0.25f / x;
		y = (mA(1, 0) + mA(0, 1)) * f;
		z = (mA(0, 2) + mA(2, 0)) * f;
		w = (mA(2, 1) - mA(1, 2)) * f;
	}
	else if (m11 > m22)//y leading
	{
		y = std::sqrtf(m11 - m00 - m22 + 1.0f) * 0.5f;
		const auto& f = 0.25f / x;
		x = (mA(1, 0) + mA(0, 1)) * f;
		z = (mA(2, 1) + mA(1, 2)) * f;
		w = (mA(0, 2) - mA(2, 0)) * f;
	}
	else
	{
		z = std::sqrtf(m22 - m00 - m11 + 1.0f) * 0.5f;
		MFfloat f = 0.25F / z;
		x = (mA(0, 2) + mA(2, 0)) * f;
		y = (mA(2, 1) + mA(1, 2)) * f;
		w = (mA(1, 0) - mA(0, 1)) * f;
	}
}

//rotates a vector v around quaternion q and q* where q*=conjuaget(q)
//v'=qvq*
//q can represent multiple rotations such that q = q2*q1 and q* will represent q2* * q1*
MFvec3 Manifest_Math::Rotation_Q(const MFvec3& v, const MFquaternion& q)
{
	const auto& b = q.Vector();
	const auto& b2 = b.x * b.x + b.y * b.y + b.z * b.z;

	return  (v * (q.w * q.w - b2) + b * (Dot(v, b) * 2.0f)
		+ Cross(b, v) * (q.w * 2.0f));
}

MFquaternion Manifest_Math::operator*(const MFquaternion& q1, const MFquaternion& q2)
{
	return MFquaternion
	{
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
		q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
	};
}

MFquaternion Manifest_Math::operator*(const MFquaternion& q, const MFfloat& s)
{
	return MFquaternion
	{
		q.x*s,q.y*s,q.z*s,q.w*s
	};
}

MFquaternion Manifest_Math::operator+(const MFquaternion& q1, const MFquaternion& q2)
{
	return MFquaternion
	{
		q1.x + q2.x , q1.y + q2.y , q1.z + q2.z , q1.w + q2.w
	};
}

//Slerp q1 between q2 with respect to t
MFquaternion Manifest_Math::Slerp(const MFquaternion& q1, MFquaternion q2, const MFfloat& t)
{
	auto&& cosOmega = q1.w * q2.w + Dot(q1.Vector(), q2.Vector());
	if (cosOmega < 0)
	{
		q2 = -q2;//conjugate of unit qutaternion is the inverse
		cosOmega = -cosOmega;
	}
	MFfloat k0, k1;
	if (cosOmega > 0.9999f)
	{//points are close, linearly interpolate
		k0 = 1.0f - t;
		k1 = t;
	}
	else
	{//slerp, sin^2 + cos^2 = 1
		auto&& sinOmega = std::sqrtf(1.0f - cosOmega * cosOmega);
		//calc omega
		auto&& invSinOmega = 1.0f / sinOmega;
		auto&& omega = atan2f(sinOmega, cosOmega);
		k0 = std::sinf((1.0f - t) * omega) * invSinOmega;
		k1 = std::sinf(t * omega) * invSinOmega;
	}
	//interpolate
	const auto& real = q1.w * k0 + q2.w * k1;
	const auto& imaginary = q1.Vector() * k0 + q2.Vector() * k1;

	return Normalize(MFquaternion{ imaginary,real });
}

MFquaternion Manifest_Math::QtoAxis(const MFquaternion& q)
{
	const auto& theta = acosf(q.w) * 2.0f;
	return MFquaternion{ q.Vector()*sinf(theta / 2.0f) , cosf(theta/2.0f) };
}