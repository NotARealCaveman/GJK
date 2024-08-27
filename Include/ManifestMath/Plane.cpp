#include "Plane.h"

using namespace Manifest_Math;

//returns a normalized plane such that ||n||=1
MFplane Manifest_Math::Normalize(const MFplane& f)
{
	const auto surfaceNormal{ reinterpret_cast<const MFvec3&>(f.x)};
	const auto iNormalMag{ 1.0f / Magnitude(surfaceNormal) };
	
	return{ surfaceNormal * iNormalMag,f.w * iNormalMag };
}

//returns the plane f as a 4d row vector multiplied with transform matrix h
//to transform plane f from coord system A to coord system B
//due to the nature of antivectors, transform h must be inverted before being passed
//such that h represents the transformation of B to A
const MFplane Manifest_Math::operator*(const MFplane& f, const MFtransform& h)
{
	return MFplane
	{
		f.x * h(0,0) + f.y * h(1,0) + f.z * h(2,0),
		f.x * h(0,1) + f.y * h(1,1) + f.z * h(2,1),
		f.x * h(0,2) + f.y * h(1,2) + f.z * h(2,2),
		f.x * h(0,3) + f.y * h(1,3) + f.z * h(2,3) + f.w
	};
}

//returns the dot product of the plane normal and direction v 
const MFfloat Manifest_Math::Dot(const MFplane& f, const MFvec3& v)
{
	return f.x * v.x + f.y * v.y + f.z * v.z;
}
//returns the signed distance orthogonal to the plane from the point
const MFfloat Manifest_Math::Dot(const MFplane& f, const MFpoint3& p)
{
	return f.x * p.x + f.y * p.y + f.z * p.z + f.w;//*1.0f;
}
//p'=p-2nfp=(I4-2nXf)p
//a reflection p' is described as the shortest distance traveled from p to q in plane f twice
//f=[n|d], where ||n||=1
const MFtransform Manifest_Math::ReflectPlane(const MFplane& f)
{
	const auto& x2 = f.x * -2.0f;
	const auto& x2y = x2 * f.y;
	const auto& y2 = f.y * -2.0f;
	const auto& x2z = x2 * f.z;
	const auto& y2z = y2 * f.z;
	const auto& z2 = f.z * -2.0f;	
	
	return MFtransform
	{
		1.0f + x2 * f.x, x2y, x2z,
		x2y, 1.0f + y2 * f.y, y2z,
		x2z, y2z, 1.0f + z2 * f.z,
		x2 * f.w, y2 * f.w, z2 * f.w
	};
}
//captures point q at which the line p+vt intersects the plane f
//if such a intersections does not exist function returns false
//**LINE IS ASSUMED TO PASS THROUGH THE NEGATIVE SIDE OF THE PLANE TOWARDS THE POSITIVE,f*v>0
//**A LINE PASSING THROUGH THE FRONT OF THE PLANE HAS THE PROPERTY, f*p>0
//**A RAY INTERSECTION ONLY OCCURS IFF f*p<0 DENOTING THE RAY POINTS TOWARDS THE POSITIVE PLANE
const MFbool Manifest_Math::InteresectionLinePlane(const MFpoint3& p, const MFvec3& v, const MFplane& f, MFpoint3& q)
{
	const MFfloat fv{ Dot(f, v) };
	
	if (std::fabsf(fv) > std::numeric_limits<MFfloat>::min() && std::fabsf(fv) != std::numeric_limits<MFfloat>::infinity())
	{//line is not parallel to plane
		q = p - v * (Dot(f, p) / fv);//projects p along v direction onto plane f and captures point at which the intersection occurs
		return true;//intersection exists
	}

	return false;//intersection does not exist
}
//captures a point q at which the ray p+vt,t>=0 that intersects point f
const MFbool Manifest_Math::InteresectionRayPlane(const MFpoint3& p, const MFvec3& v, const MFplane& f, MFpoint3& q)
{
	const MFfloat fp { Dot(f, p)};
	const MFfloat fv { Dot(f, v)};
	const MFfloat t{ -1.0f * (fp / fv) };
	if (t >= std::numeric_limits<MFfloat>::min() && std::fabsf(t) != std::numeric_limits<MFfloat>::infinity())//t acts a  projecting scalar value of p through v
	{
		q = p + v * t;
		return true;
	}

	return false;
}
//captures a point q at which the ray p+vt,t>=0 that intersects point f
//in addition this ray also passes through the front of the plane to the negative 
const MFbool Manifest_Math::InteresectionRayPlaneFront(const MFpoint3& p, const MFvec3& v, const MFplane& f, MFpoint3& q)
{
	const MFfloat fp{ Dot(f, p) };//if > 0 then p lies infront of the plane
	const MFfloat fv{ Dot(f, v) };//if <0 then v points towards the plane passing through
	if (fp > 0 && fv < 0)
	{
		q = p - v* (fp / fv);//t = -f*p/f*v
		return true;
	}

	return false;
}
//captures a point p at which three planes intersect in space provided one exists
//such a point exists if all three planes are linearly independent
//p=(dA(c x b) + dB(a x c) + dC(b x a)) / [nA,nB,nC]
//point p is determined by the multiplication of the fw as a column vector multiplied by the inverse of fn as a series of equations such that they make a the rows of M3
const MFbool Manifest_Math::IntersectionThreePlanes(const MFplane& fA, const MFplane& fB, const MFplane& fC, MFpoint3& p)
{
	//normals of the three planes
	const auto& nA = fA.Normal();
	const auto& nB = fA.Normal();
	const auto& nC = fA.Normal();
	//triple scalar product = det(M3) = a x b * c
	const auto& AxB = Cross(nA, nB);
	const auto& det = Dot(AxB, nC);
	if (std::fabs(det) > FLT_MIN)//test linear independence, if two planes are parallel then det=0
	{	
		p = (Cross(nC, nA) * fA.w + Cross(nA, nC) * fB.w + AxB * fC.w) / det;
		return true;
	}

	return false;
}
//captures a point p at which two planes planes intersect a plane at the origin,O[v|0]
//p=(dA(v x b) + dB(a x v)) / v^2
//point p is determined by the multiplication of the fw as a column vector multiplied by the inverse of fA,B,O as a series of equations such that they make a the rows of M3
const MFbool Manifest_Math::IntersectionTwoPlanes(const MFplane& fA, const MFplane& fB, MFpoint3& p, MFvec3& v)
{
	//normals of the three planes
	const auto& nA = fA.Normal();
	const auto& nB = fA.Normal();	
	//dot(v,v)=v^2=det(f), f[v|d]
	v = Cross(nA, nB);//0 if planes are parallalel
	const auto& det = Dot(v,v);
	if (std::fabsf(det) > std::numeric_limits<MFfloat>::min())//test linear independence, if two planes are parallel then det=0
	{
		p = (Cross(v, nB) * fA.w + Cross(nA, v) * fB.w) / det;
		return true;
	}
	return false;
} 

//Given the plane f[n|d] and homogeneous point q{v|1}, 
//the projected point P can be found by P=Q-(n*q-d)/n^2*n. since f is normalized, P can be simplified to P=Q-((n*q)-d)*n
//let t = (n*q-d)/n^2, then P = Q-n*t
MFpoint3 Manifest_Math::ClosestPointOnPlane(const MFplane& f, const MFpoint3& q)
{
	const MFvec3& n = f.Normal();
	const MFfloat t = Dot(n, q) + f.w;

	return  MFpoint3 { q - n * t };
}