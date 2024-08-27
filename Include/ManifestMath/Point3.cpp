#include "Point3.h"

using namespace Manifest_Math;


//given the line segment AB and point Q,  point p can be found by
//P(t)=A+t(B-A) where t is the projecttion of c onto AB
//since a line segment is used and not inifite length, t must be clamped
MFpoint3 Manifest_Math::ClosestPointFromlineSegment(const MFpoint3& q, const MFpoint3& start, const MFpoint3& end)
{
	const MFpoint3 segment = end - start;	
	const MFfloat interval{ std::fminf(std::fmaxf(Dot(q - start, segment) / Dot(segment, segment), 0.0f), 1.0f) };

	return start + segment * interval;
}

MFfloat Manifest_Math::PointDistanceFromLineSegmentSquared(const MFpoint3& q, const MFpoint3& start, const MFpoint3& end)
{
	const MFpoint3 ab = end - start, ac = q - start, bc = q - end;
	const MFfloat e{ Dot(ac,ab) };
	//check if c projects outside line AB
	if (e <= 0.0f)//return distance of AC sqaured
		return Dot(ac,ac);
	const MFfloat f{ Dot(ab,ab) };
	if (e>= f)//return distance of AB sqaured
		return Dot(ab,ab);
	//return point c along it's interval on AC
	return Dot(ac, ac) - e * e / f;
}

//lifted from Real-Time collision Detection - returns the squared distance between S0(s) and S1(t)
MFfloat Manifest_Math::ClosestPointsSegmentSegment(MFpoint3 segment00, MFpoint3 segment01, MFpoint3 segment10, MFpoint3 segment11, MFpoint3& closest0, MFpoint3& closest1,	MFfloat& s, MFfloat& t)
{	
	constexpr MFfloat EPSILON{ 1e-5 };

	const MFvec3 direction0 = segment01 - segment00; //direction segment 0
	const MFvec3 direction1 = segment11 - segment10; // direction segment 1
	const MFpoint3 verticalDirection = segment00 - segment10;//inner "normal"
	const MFfloat direction0SqauredDistance = Dot(direction0, direction0);
	const MFfloat direction1SqauredDistance = Dot(direction1, direction1);
	//project direction 1 onto vertical 
	const MFfloat centerProjection1 = Dot(direction1, verticalDirection);
	// Check if either or both segments degenerate into points
	if (direction0SqauredDistance <= EPSILON && direction1SqauredDistance <= EPSILON)
	{
		// Both segments degenerate into points
		s = t = 0.0f;
		closest0 = segment00;
		closest1 = segment10;
			
		return Dot(closest0 - closest1, closest0 - closest1);
	}
	if (direction0SqauredDistance <= EPSILON)
	{
		// First segment degenerates into a point
		s = 0.0f;
		t = centerProjection1 / direction0SqauredDistance; // s = 0 => t = (b*s + f) / e = f / e
		t = std::clamp(t, 0.0f, 1.0f);
	}
	else
	{   //project direction 0 onto vertical 
		const MFfloat centerProjection0 = Dot(direction0, verticalDirection);
		if (direction1SqauredDistance <= EPSILON)
		{
			// Second segment degenerates into a point
			t = 0.0f;
			s = std::clamp(-centerProjection0 / direction0SqauredDistance, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
		}
		else 
		{
			// The general nondegenerate case starts here
			//project direction0 onto direction1
			const MFfloat b = Dot(direction0, direction1);
			const MFfloat denom = direction0SqauredDistance * direction1SqauredDistance - b * b; // Always nonnegative
			// If segments not parallel, compute closest point on L0 to L1 and
			// clamp to segment S0. Else pick arbitrary s (here 0)
			if (denom != 0.0f) 
				s = std::clamp((b * centerProjection1 - centerProjection0 * direction1SqauredDistance) / denom, 0.0f, 1.0f);
			else 
				s = 0.0f;
			// Compute point on L1 closest to S0(s) using
			// t = Dot((P0 + D0*s) - P1,D1) / Dot(D1,D1) = (b*s + f) / e
			t = (b * s + centerProjection1) / direction1SqauredDistance;
			// If t in [0,1] done. Else clamp t, recompute s for the new value
			// of t using s = Dot((P1 + D1*t) - P1,D0) / Dot(D0,D0)= (t*b - c) / a
			// and clamp s to [0, 1]
			if (t < 0.0f)
			{
				t = 0.0f;
				s = std::clamp(-centerProjection0 / direction0SqauredDistance, 0.0f, 1.0f);
			}
			else if (t > 1.0f)
			{
				t = 1.0f;
				s = std::clamp((b - centerProjection0) / direction0SqauredDistance, 0.0f, 1.0f);
			}
		}
	}

	closest0 = segment00 + direction0 * s;
	closest1 = segment10 + direction1 * t;

	return Dot(closest0 - closest1, closest0 - closest1);
}

MFpoint3 Manifest_Math::ComponentMultiply(const MFvec3& a, const MFpoint3& b)
{
	return MFpoint3{ a.x * b.x,a.y * b.y,a.z * b.z };
}
MFpoint3 Manifest_Math::ComponentMultiply(const MFpoint3& a, const MFvec3& b)
{
	return ComponentMultiply(b, a);
}