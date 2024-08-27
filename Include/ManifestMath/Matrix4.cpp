#include "Matrix4.h"

using namespace Manifest_Math;

MFmat4 Manifest_Math::Transpose(const MFmat4& mA)
{
	return MFmat4
	{
		mA(0,0),mA(0,1),mA(0,2),mA(0,3),
		mA(1,0),mA(1,1),mA(1,2),mA(1,3),
		mA(2,0),mA(2,1),mA(2,2),mA(2,3),
		mA(3,0),mA(3,1),mA(3,2),mA(3,3),
	};
}

//returns the "magnitude" of Ma
MFfloat Manifest_Math::Determinant(const MFmat4& mA)
{	
	//grab 3D C-Vectors of M4x3
	const MFvec3& a = mA[0];
	const MFvec3& b = mA[1];
	const MFvec3& c = mA[2];
	const MFvec3& d = mA[3];
	//grab 4th row's vector
	const auto& p = MFvec4{ mA(3,0),mA(3,1) ,mA(3,2) ,mA(3,3) };
		
	auto s = Cross(a, b);//returns vector perpendicular to vectors a&b
	auto t = Cross(c, d);//returns vector perpendicular to vectors c&d
	auto u = a * p.y - b * p.x;
	auto v = c * p.w - d * p.z;

	return Dot(s , v) + Dot(t , u);
}

//retruns the inverse of M4
//M^-1=Madj^T/|M|
//M^-1=1/|M|*Madj
//M^-1=(1/s*v+t*u)({bxv+yt,-b*t},{vxa-xt,a*t},{dxu+ws,-d*s},{uxc-zs,c*s})
MFmat4 Manifest_Math::Inverse(const MFmat4& mA)
{
	//grab 3D C-Vectors of M4x3
	const MFvec3& a = mA[0];
	const MFvec3& b = mA[1];
	const MFvec3& c = mA[2];
	const MFvec3& d = mA[3];
	//grab 4th row's vector
	const auto& p = MFvec4{ mA(3,0),mA(3,1) ,mA(3,2) ,mA(3,3) };

	auto s = Cross(a, b);//returns vector perpendicular to vectors a&b
	auto t = Cross(c, d);//returns vector perpendicular to vectors c&d
	auto u = a * p.y - b * p.x;
	auto v = c * p.w - d * p.z;

	MFfloat iDet = 1.0f / (Dot(s, v) + Dot(t, u));
	//create inverse scalars
	s *= iDet;
	t *= iDet;
	u *= iDet;
	v *= iDet;
	//using inverse scalars, calculate inverse row vectors
	const auto& r0 = Cross(b, v) + t * p.y;
	const auto& r1 = Cross(v, a) - t * p.x;
	const auto& r2 = Cross(d, u) + s * p.w;
	const auto& r3 = Cross(u, d) - s * p.z;

	return MFmat4
	{
		r0.x,r1.x,r2.x,r3.x,
		r0.y,r1.y,r2.y,r3.y,
		r0.z,r1.z,r2.z,r3.z,
		-Dot(b,t),Dot(a,t),-Dot(d,s),Dot(c,s)
	};
}

MFmat4 Manifest_Math::operator*(const MFmat4& mA, const MFmat4& mB)
{
	return MFmat4
	{
		//Column 0
		mA(0, 0) * mB(0, 0) + mA(0, 1) * mB(1, 0) + mA(0, 2) * mB(2, 0) + mA(0, 3) * mB(3, 0),//Row 0
		mA(1, 0)* mB(0, 0) + mA(1, 1) * mB(1, 0) + mA(1, 2) * mB(2, 0) + mA(1, 3) * mB(3, 0), //Row 1
		mA(2, 0)* mB(0, 0) + mA(2, 1) * mB(1, 0) + mA(2, 2) * mB(2, 0) + mA(2, 3) * mB(3, 0), //Row 2
		mA(3, 0)* mB(0, 0) + mA(3, 1) * mB(1, 0) + mA(3, 2) * mB(2, 0) + mA(3, 3) * mB(3, 0), //Row 3
		//Column 1
		mA(0, 0) * mB(0, 1) + mA(0, 1) * mB(1, 1) + mA(0, 2) * mB(2, 1) + mA(0, 3) * mB(3, 1),//Row 0
		mA(1, 0)* mB(0, 1) + mA(1, 1) * mB(1, 1) + mA(1, 2) * mB(2, 1) + mA(1, 3) * mB(3, 1), //Row 1
		mA(2, 0)* mB(0, 1) + mA(2, 1) * mB(1, 1) + mA(2, 2) * mB(2, 1) + mA(2, 3) * mB(3, 1), //Row 2
		mA(3, 0)* mB(0, 1) + mA(3, 1) * mB(1, 1) + mA(3, 2) * mB(2, 1) + mA(3, 3) * mB(3, 1), //Row 3
		//Column 2
		mA(0, 0) * mB(0, 2) + mA(0, 1) * mB(1, 2) + mA(0, 2) * mB(2, 2) + mA(0, 3) * mB(3, 2),//Row 0
		mA(1, 0)* mB(0, 2) + mA(1, 1) * mB(1, 2) + mA(1, 2) * mB(2, 2) + mA(1, 3) * mB(3, 2), //Row 1
		mA(2, 0)* mB(0, 2) + mA(2, 1) * mB(1, 2) + mA(2, 2) * mB(2, 2) + mA(2, 3) * mB(3, 2), //Row 2
		mA(3, 0)* mB(0, 2) + mA(3, 1) * mB(1, 2) + mA(3, 2) * mB(2, 2) + mA(3, 3) * mB(3, 2), //Row 3
		//Column 3
		mA(0, 0)* mB(0, 3) + mA(0, 1) * mB(1, 3) + mA(0, 2) * mB(2, 3) + mA(0, 3) * mB(3, 3), //Row 0
		mA(1, 0)* mB(0, 3) + mA(1, 1) * mB(1, 3) + mA(1, 2) * mB(2, 3) + mA(1, 3) * mB(3, 3), //Row 1
		mA(2, 0)* mB(0, 3) + mA(2, 1) * mB(1, 3) + mA(2, 2) * mB(2, 3) + mA(2, 3) * mB(3, 3), //Row 2
		mA(3, 0)* mB(0, 3) + mA(3, 1) * mB(1, 3) + mA(3, 2) * mB(2, 3) + mA(3, 3) * mB(3, 3), //Row 3
	};
};

MFvec4 Manifest_Math::operator*(const MFmat4 mA, const MFvec4& vB)
{
	return MFvec4
	{																	   //N=x/y/z element pos
		mA(0,0) * vB.x + mA(0,1) * vB.y + mA(0,2) * vB.z + mA(0,3) * vB.w, //MVx = mAc0rN*vBN
		mA(1,0) * vB.x + mA(1,1) * vB.y + mA(1,2) * vB.z + mA(1,3) * vB.w, //MVx = mAc0rN*vBN
		mA(2,0) * vB.x + mA(2,1) * vB.y + mA(2,2) * vB.z + mA(2,3) * vB.w, //MVx = mAc2rN*vBN
		mA(3,0) * vB.x + mA(3,1) * vB.y + mA(3,2) * vB.z + mA(3,3) * vB.w, //MVx = mAc0rN*vBN
	};
};

MFmat4 Manifest_Math::XYZtoXZY(const MFmat4& mA)
{
	return MFmat4
	{
		mA[0][0],-mA[0][1],mA[0][2],0,
		-mA[0][0],mA[0][1],-mA[0][2],0,
		mA[0][0],-mA[0][1],mA[0][2],0,
		mA[0][0],-mA[0][1],mA[0][2],1
	};
}

MFmat4 Manifest_Math::XZYtoXYZ(const MFmat4& mA)
{
	return MFmat4
	{
		mA[0][0],mA[0][1],-mA[0][2],0,
		mA[0][0],mA[0][1],-mA[0][2],0,
		-mA[0][0],-mA[0][1],mA[0][2],0,
		mA[0][0],mA[0][1],-mA[0][2],1
	};
}