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
	void CullSimplexVertices(Simplex_T<Support>& simplex, MFpoint3& closestPoint);
	//calls barycentric functions based on simplex degree
	const MFpoint3 FindClosestPointOnSimplexFromOrigin(Simplex_T<Support>& simplex);
	//returns search directions based on simplex degree
	const MFvec3 SearchDirection(const Simplex_T<Support>& simplex);
	//returns containment status of origin based on simplex degree
	const MFbool ContainsOrigin(const Simplex_T<Support>& simplex);
	//checks if a given support point currenlt exists in the simplex
	const MFbool ContainsSupport(const Simplex_T<Support>& simplex, const Support& newPoint);
	//returns true if collision not detected, stores closest point
	template<typename Geometry>
	MFbool GJK(const Geometry& geometry, const ConvexHull& hull, Simplex_T<Support>& simplex, MFpoint3& closestPoint, MFfloat& distance, MFvec3 direction = { 1,0,0 })
	{
	}		
}