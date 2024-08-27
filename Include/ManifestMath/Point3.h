#pragma once
#include "Vector3.h"

//used when defining a point in 3d space
//4th component is assumed to be 1, not required to be stored in memory
namespace Manifest_Math
{
	struct MFpoint3 : public MFvec3
	{
		constexpr MFpoint3() = default;
		constexpr MFpoint3(const MFfloat& x, const MFfloat& y, const MFfloat& z) :MFvec3{ x,y,z } {};
		constexpr MFpoint3(const MFfloat& u) : MFvec3{ u } {};
		constexpr MFpoint3(const MFvec3& v) : MFvec3{ v } {};
	};		
	//returns the point on a line segment closet to an arbitary non collinear point
	MFpoint3 ClosestPointFromlineSegment(const MFpoint3& q, const MFpoint3& start, const MFpoint3& end);
	MFfloat PointDistanceFromLineSegmentSquared(const MFpoint3& q, const MFpoint3& start, const MFpoint3&);		
	MFpoint3 ComponentMultiply(const MFvec3& a, const MFpoint3& b);
	inline extern MFpoint3 ComponentMultiply(const MFpoint3& a, const MFvec3& b);
	// Computes closest points C1 and C2 of S1(s)=P1+s*(Q1-P1) and
	// S2(t)=P2+t*(Q2-P2), returning s and t. Function result is squared
	// distance between between S1(s) and S2(t)
	MFfloat ClosestPointsSegmentSegment(MFpoint3 segment00, MFpoint3 segment01, MFpoint3 segment10, MFpoint3 segment11, MFpoint3& closest0, MFpoint3& closest1, MFfloat& s, MFfloat& t);
}
