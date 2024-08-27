#pragma once
#include "Matrix3.h"


namespace Manifest_Math
{
	struct MFquaternion
	{
		MFquaternion() = default;
		MFquaternion(const MFfloat& _x, const MFfloat& _y, const MFfloat& _z, const MFfloat& _w) :
			x(_x), y(_y), z(_z), w(_w) {};
		MFquaternion(const MFvec3& v, const MFfloat& s) : x(v.x), y(v.y), z(v.z), w(s) {};
		MFquaternion& operator+=(const MFquaternion& inQ)
		{
			x += inQ.x;
			y += inQ.y;
			z += inQ.z;
			w += inQ.w;

			return *this;
		}
		MFquaternion& operator *=(const MFquaternion& q)
		{
			x = w * q.x + x * q.w + y * q.z - z * q.y;
			y = w * q.y - x * q.z + y * q.w + z * q.x;
			z = w * q.z + x * q.y - y * q.x + z * q.w;
			w = w * q.w - x * q.x - y * q.y - z * q.z;

			return *this;
		};
		MFquaternion& operator-()
		{
			x = -x;
			y = -y;
			z = -z;
			w = -w;

			return *this;
		}
		MFquaternion& operator /=(const MFfloat& s)
		{
			auto scale = 1.0f / s;

			x *= scale;
			y *= scale;
			z *= scale;
			w *= scale;

			return *this;
		}
		MFquaternion& operator-=(const MFquaternion& inQ)
		{
			x -= inQ.x;
			y -= inQ.y;
			z -= inQ.z;
			
			return *this;
		}

		const MFvec3& Vector() const
		{
			return reinterpret_cast<const MFvec3&>(x);
		};

		MFmat3 GetRotation() const;
		void SetRotation(const MFmat3& mA);

		//console shit
		friend std::ostream& operator<<(std::ostream& os, const MFquaternion& inQuat) {
			os << inQuat.w << "w " << inQuat.x << "i " << inQuat.y << "j " << inQuat.z << "k ";
			return os;
		};


		MFfloat x;
		MFfloat y;
		MFfloat z;
		MFfloat w;
	};

	//q1q2 = {q1Vxq2V + q1wq2V + q2wq1V },q1wq2w-q1V*q2V
	MFquaternion operator *(const MFquaternion& q1, const MFquaternion& q2);	
	MFquaternion operator *(const MFquaternion& q, const MFfloat& s);
	MFquaternion operator+(const MFquaternion& q1, const MFquaternion& q2);
	//returns pure quaternion	
	inline MFfloat Magnitude(const MFquaternion& q)	
	{
		return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	}
	inline MFquaternion Normalize(const MFquaternion& q)
	{
		const auto& s = 1.0f / Magnitude(q);
		return { q.x * s,q.y * s,q.z * s,q.w * s };		
	}
	inline MFvec3 QtoE(const MFquaternion& q)
	{
		const auto& singularityTest = q.x * q.y + q.z * q.w;
		if (singularityTest > 0.499)//north pole singularity		
			return MFvec3{ 2 * atan2f(q.x, q.w),Pi / 2.0f,0.0f };
		if (singularityTest < -0.499)//south pole singularity
			return MFvec3{ -2.0f * atan2f(q.x,q.w),-Pi / 2.0f,0.f };

		const auto&& x2 = q.x * q.x;
		const auto&& y2 = q.y * q.y;
		const auto&& z2 = q.z * q.z;
		return MFvec3
		{
			
			atan2f(2.0f * q.x * q.w - 2.0f * q.y * q.z, 1.0f - 2.0f * x2 - 2.0f * z2),//bank
			atan2f(2.0f * q.y * q.w - 2.0f * q.x * q.z, 1.0f - 2.0f * y2 - 2 * z2),//heading
			asinf(2.0f * singularityTest)//attitude			
		};
	}
	//rotates a vector v about a quaternion using the sandwich product of
	//qvq*, where q* = conjugate of q
	MFvec3 Rotation_Q(const MFvec3& v, const MFquaternion& q);
	//slerps q1 by q2 some angle t radians
	MFquaternion Slerp(const MFquaternion& q1, MFquaternion q2, const MFfloat& t);
	//converts quaternion to axis-angle form
	MFquaternion QtoAxis(const MFquaternion& q);
}