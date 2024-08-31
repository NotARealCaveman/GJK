#pragma once
#include <algorithm>
#include <array>
#include <vector>

#include <ManifestMath/Barycentric.h>
#include <ManifestMath/Plane.h>
#include <ManifestMath/Simplex.h>

#include <ManifestSimulation/CollisionEngine/Collider.h>
#include "Support.h"

using namespace Manifest_Math;

namespace Manifest_Simulation
{
	//remove vertices from simplex that do not contribute, internally updates closest point
	const MFpoint3 CullSimplexVertices(Simplex_T<Support>& simplex);
	//calls barycentric functions based on simplex degree
	const MFpoint3 FindClosestPointOnSimplexFromOrigin(Simplex_T<Support>& simplex);
	//returns search directions based on simplex degree
	const MFvec3 SearchDirection(const Simplex_T<Support>& simplex);
	//returns containment status of origin based on simplex degree
	const MFbool ContainsOrigin(const Simplex_T<Support>& simplex);
	//checks if a given support point currently exists in the simplex
	const MFbool ContainsSupport(const Simplex_T<Support>& simplex, const Support& newPoint);
	//returns true if collision not detected, stores closest point
	template<typename Geometry>
	MFbool GJK(const Geometry& geometry, const ConvexHull& hull, Simplex_T<Support>& simplex, MFfloat& distance, MFvec3 direction = { 1,0,0 })
	{
		constexpr MFu32 ITERATION_LIMIT{ 10 };
		distance = std::numeric_limits<MFfloat>::infinity();
		for (MFu32 iteration{ 0 }; iteration < ITERATION_LIMIT; ++iteration)
		{ 
			Support support{ SupportPoint(geometry,hull,direction) };
			//check if point is a duplicate
			if (ContainsSupport(simplex, support))
				return true;// containment checked in prior iteration
			//ensure new support point is always front
			simplex.PushFront(support);
			const MFpoint3 closestPoint{ CullSimplexVertices(simplex) };
			//exit false if origin found to be enclosed
			if (ContainsOrigin(simplex))
				return false;
			//ensure progress towards origin
			const MFfloat distanceSqaured{ Dot(closestPoint,closestPoint) };
			if (distanceSqaured >= distance)
				return true;
			//update serach direction and distance for next iteration
			distance = distanceSqaured;
			direction = SearchDirection(simplex);
		} 
		//assume we couldn't find a collision or missed an exit condition
		return true;//report no collision
	}
}