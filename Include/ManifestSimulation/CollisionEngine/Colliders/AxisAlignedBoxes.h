#pragma once

#include <ManifestMath/Point3.h>
#include <ManifestMath/Triangle.h>
#include <ManifestMath/Transform.h>
#include <ManifestMath/Quaternion.h>


using namespace Manifest_Math;

namespace Manifest_Simulation
{
	const MFpoint3 cubeVertices[8] =
	{
		{1.0,1.0,1.0},{1.0,1.0,-1.0},
		{-1.0,1.0,1.0},{-1.0,1.0,-1.0},
		{-1.0,-1.0,1.0},{-1.0,-1.0,-1.0},
		{1.0,-1.0,1.0},{1.0,-1.0,-1.0},
	};

	//contains all the information for creating and working AABBs - intersection tests done in BoundingVolume.h	
	//a box is the second volume - represented as a a center in local space and the distance from it to one edge - AABB	
	struct AxisBoundingBox
	{
		MFpoint3 center;
		MFvec3 halfLength;		
	};
	AxisBoundingBox AABBFromMinMax(const MFpoint3& min, const MFpoint3& max);
	AxisBoundingBox Union(const AxisBoundingBox& a, const AxisBoundingBox& b);	
	MFfloat SquaredDistanceFromPointToAABB(const AxisBoundingBox& bounds, const MFpoint3& point);
	inline MFfloat SurfaceArea(const AxisBoundingBox& a);
	inline MFfloat Volume(const AxisBoundingBox& a);	
	MFbool IsEncapsulated(const AxisBoundingBox& captor, const AxisBoundingBox& captive);	
	MFbool IsEncapsulated(const AxisBoundingBox& captor, const MFpoint3& captive);
	void AABBMinMax(const AxisBoundingBox& b, MFpoint3& min, MFpoint3& max);

	//AABB overlap tests - broadphase/midphase	
	MFbool Overlapping(const AxisBoundingBox& a0, const AxisBoundingBox& a1);	
	MFbool Overlapping(const MFtriangle& triangle, const AxisBoundingBox& bounds);
	MFbool Overlapping(const MFplane& plane, const AxisBoundingBox& bounds);	
	//Ray intersection test - picking/ray tracing
	MFbool Intersects(const AxisBoundingBox& bounds, const MFpoint3& rayOrigin, const MFvec3& rayDirection, MFfloat& intervalOnRay, MFpoint3& intersectionPoint);
}