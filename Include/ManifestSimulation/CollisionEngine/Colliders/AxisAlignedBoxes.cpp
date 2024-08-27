#include "AxisAlignedBoxes.h"

using namespace Manifest_Simulation;

void Manifest_Simulation::AABBMinMax(const AxisBoundingBox& b, MFpoint3& min, MFpoint3& max)
{
	min = b.center - b.halfLength;
	max = b.center + b.halfLength;
}

AxisBoundingBox Manifest_Simulation::AABBFromMinMax(const MFpoint3& min, const MFpoint3& max)
{
	const MFpoint3 center{ (min + max) * 0.5f };
	const MFvec3 halfLength{ Max((max - min) * 0.5f,0.01) };

	return AxisBoundingBox{ center,halfLength };
}

AxisBoundingBox Manifest_Simulation::Union(const AxisBoundingBox& a, const AxisBoundingBox& b)
{
	MFpoint3 aMin, aMax, bMin, bMax;
	AABBMinMax(a, aMin, aMax);
	AABBMinMax(b, bMin, bMax);
	MFpoint3 min{ Min(aMin,bMin) };
	MFpoint3 max{ Max(aMax,bMax) };

	AxisBoundingBox result;
	result.center = (min + max) * 0.5f;
	result.halfLength = (max - min) * 0.5f;
	return result;
}

MFfloat Manifest_Simulation::SquaredDistanceFromPointToAABB(const AxisBoundingBox& bounds, const MFpoint3& point)
{
	MFpoint3 min, max;
	AABBMinMax(bounds, min, max);

	MFfloat result{ 0.0f };
	for (MFu8 axis{ 0 }; axis < 3; ++axis)
	{
		MFfloat gap{ point[axis] };
		if (gap < min[axis])
			result += (min[axis] - gap) * (min[axis] - gap);
		if (gap > max[axis])
			result += (max[axis] - gap) * (max[axis] - gap);
	}

	return result;
}

extern MFfloat Manifest_Simulation::SurfaceArea(const AxisBoundingBox& a)
{
	return 2 * (a.halfLength.x * a.halfLength.y + a.halfLength.x * a.halfLength.z + a.halfLength.y * a.halfLength.z);
}

extern MFfloat Manifest_Simulation::Volume(const AxisBoundingBox& a)
{
	return 8.0f * (a.halfLength.x * a.halfLength.y * a.halfLength.z);
}

MFbool Manifest_Simulation::IsEncapsulated(const AxisBoundingBox& captor, const AxisBoundingBox& captive)
{
	//Calculate the minimum and maximum extents
	MFpoint3 minCaptive = captive.center - captive.halfLength;
	MFpoint3 maxCaptive = captive.center + captive.halfLength;
	MFpoint3 minCaptor = captor.center - captor.halfLength;
	MFpoint3 maxCaptor = captor.center + captor.halfLength;
	// Check if all six boundaries of AABB b are within the boundaries of AABB a.
	return (minCaptive.x >= minCaptor.x) && (maxCaptive.x <= maxCaptor.x) && (minCaptive.y >= minCaptor.y) && (maxCaptive.y <= maxCaptor.y) && (minCaptive.z >= minCaptor.z) && (maxCaptive.z <= maxCaptor.z);
}
MFbool Manifest_Simulation::IsEncapsulated(const AxisBoundingBox& bounds, const MFpoint3& point)
{
	MFpoint3 minBounds = bounds.center - bounds.halfLength;
	MFpoint3 maxBounds = bounds.center + bounds.halfLength;
	const MFfloat epsilon{ EPSILON_3D({minBounds,maxBounds,point}) };
	minBounds -= {epsilon};
	maxBounds += {epsilon};

	return (point.x >= minBounds.x) && (point.x <= maxBounds.x) &&
		(point.y >= minBounds.y) && (point.y <= maxBounds.y) &&
		(point.z >= minBounds.z) && (point.z <= maxBounds.z);
}

//checks for all three axes of both boxes to be overlapping
MFbool Manifest_Simulation::Overlapping(const AxisBoundingBox& a, const AxisBoundingBox& b)
{
	std::vector<MFpoint3> points(4);
	AABBMinMax(a, points[0], points[1]);
	AABBMinMax(a, points[2], points[3]);
	const MFfloat epsilon{ EPSILON_3D(points) };

	for (MFu8 axis{ 0 }; axis < 3; ++axis)
		if (std::fabsf(a.center[axis] - b.center[axis]) > (a.halfLength[axis] + b.halfLength[axis] + epsilon))
			return false;

	return true;
}

//use SAT to rule out or detected overlapp between triangle and aabb
MFbool Manifest_Simulation::Overlapping(const MFtriangle& triangle, const AxisBoundingBox& bounds)
{
	//translate triangle making AABB 'origin'
	const MFpoint3 a{ triangle.vertices[0] - bounds.center };
	const MFpoint3 b{ triangle.vertices[1] - bounds.center };
	const MFpoint3 c{ triangle.vertices[2] - bounds.center };
	std::array<MFvec3, 3> triangleEdges{ b - a,c - b,a - c };
	std::array<MFvec3, 3> aabbAxes{ MFvec3{1,0,0},MFvec3{0,1,0},MFvec3{0,0,1 } };


	const auto& TestEdgeSeparation = [&](const MFvec3& alignedAxis, const MFvec3& edge)->MFbool
	{
		const MFvec3 separatingAxis{ Cross(alignedAxis,edge) };
		MFfloat p0{ Dot(a, separatingAxis) };
		MFfloat p1{ Dot(b, separatingAxis) };
		MFfloat p2{ Dot(c, separatingAxis) };
		MFfloat r{ bounds.halfLength.x * std::fabsf(Dot(aabbAxes[0], separatingAxis)) + bounds.halfLength.y * std::fabsf(Dot(aabbAxes[1], separatingAxis)) + bounds.halfLength.z * std::fabsf(Dot(aabbAxes[2], separatingAxis)) };

		return std::fmaxf(-std::fmaxf(std::fmaxf(p0, p1), p2), std::fminf(std::fminf(p0, p1), p2)) > r;
	};

	const auto& TestFaceSeparation = [&](const MFu32 axis)->MFbool
	{
		return std::fmaxf(std::fmaxf(a[axis], b[axis]), c[axis]) < -bounds.halfLength[axis] || std::fminf(std::fminf(a[axis], b[axis]), c[axis]) > bounds.halfLength[axis];
	};
	//test 9 edge cross products - combine loop with 3 aabb face normals
	for (MFu32 aabbAxis{ 0 }; aabbAxis < 3; ++aabbAxis)
	{
		for (MFu32 triangleEdge{ 0 }; triangleEdge < 3; ++triangleEdge)
			if (TestEdgeSeparation(aabbAxes[aabbAxis], triangleEdges[triangleEdge]))
			{
				//DLOG({ CONSOLE_BLUE }, "edge separation detected with axis:", aabbAxis, "and edge:", triangleEdge);
				return false;
			}
		//DLOG({ CONSOLE_BG_GREEN }, "no edges found to separate axis:", aabbAxis);

		if (TestFaceSeparation(aabbAxis))
		{
			//DLOG({ CONSOLE_BLUE }, "face separation detected with axis:", aabbAxis);
			return false;
		}
		//DLOG({ CONSOLE_GREEN }, "no face separation detected for axis:", aabbAxis);
	}
	//test triangle face normal for separation
	const MFvec3 surfaceNormal{ Cross(triangleEdges[0],triangleEdges[1]) };
	const MFplane surfacePlane{ surfaceNormal ,-Dot(surfaceNormal,a) };
		
	return Overlapping(Normalize(surfacePlane), bounds);
}

//test for intersection of the plane inside of the AABB
MFbool Manifest_Simulation::Overlapping(const MFplane& plane, const AxisBoundingBox& bounds)
{
	//compute projection interval of AABB onto L(t) = c + t * fn
	const MFvec3& n{ plane.Normal() };
	//Intersection occurs when distance of aabb center to plane falls within [-r,+r] interval
	const MFfloat interval{ bounds.halfLength.x * std::fabsf(n.x) + bounds.halfLength.y * std::fabsf(n.y) + bounds.halfLength.z * std::fabsf(n.z) };

	/*
	if(std::fabsf(Dot(plane, bounds.center)) > interval)
		DLOG({ CONSOLE_BG_RED }, "Triangle face detected as separating axis");
	else
		DLOG({ CONSOLE_BG_GREEN }, "Triangle face not detected as separating axis");
	*/

	return std::fabsf(Dot(plane, bounds.center)) > interval;
}

// Intersect ray R(t) = p + t*d against AABB a. When intersecting,
// return intersection distance tmin and point q of intersection
MFbool Manifest_Simulation::Intersects(const AxisBoundingBox& bounds, const MFpoint3& rayOrigin, const MFvec3& rayDirection, MFfloat& minimumIntersectionInterval, MFpoint3& intersectionPoint)
{	
	minimumIntersectionInterval = 0.0f;
	MFfloat maximumIntersectionInterval{ std::numeric_limits<MFfloat>::max() };
	const MFfloat EPSILON{ 1e-5 };

	const MFpoint3 min{ bounds.center - bounds.halfLength };
	const MFpoint3 max{ bounds.center + bounds.halfLength };

	for (MFu8 axis{ 0 }; axis < 3; ++axis)
	{
		if (std::fabsf(rayDirection[axis]) < EPSILON)
		{
			if (rayOrigin[axis] < min[axis] || rayOrigin[axis] > max[axis])
				return false;
		}
		else
		{
			MFfloat ood = 1.0f / rayDirection[axis];
			MFfloat t1 = (min[axis] - rayOrigin[axis]) * ood;
			MFfloat t2 = (max[axis] - rayOrigin[axis]) * ood;
			if (t1 > t2)
				std::swap(t1, t2);

			minimumIntersectionInterval = std::fmaxf(minimumIntersectionInterval, t1);
			maximumIntersectionInterval = std::fminf(maximumIntersectionInterval, t2);			

			if (minimumIntersectionInterval > maximumIntersectionInterval)
				return false;
		}		
	}

	intersectionPoint = rayOrigin + rayDirection * minimumIntersectionInterval;

	return true;

}