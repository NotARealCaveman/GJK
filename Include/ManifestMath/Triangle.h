#pragma once
#include <array>
#include <vector>

#include "Point3.h"
#include "Plane.h"

namespace Manifest_Math
{
	const MFfloat EPSILON_3D(const std::vector<MFpoint3>& pointCloud);

	struct MFtriangle
	{
		std::array<MFpoint3, 3> vertices;		
	};
	MFpoint3 Centroid(const MFtriangle& triangle);
	MFvec3 CalculateSurfaceNormal(const MFtriangle& triangle);	
	MFvec3 CalculateSurfaceNormal(const MFvec3& v0, const MFvec3& v1, const MFvec3& v2);
	MFvec3 CalculateSurfaceBivector(const MFtriangle& triangle);
	MFvec3 CalculateSurfaceBivector(const MFvec3& v0, const MFvec3& v1, const MFvec3& v2);
	inline MFplane CalculateNormalizedSurfacePlane(const MFtriangle& triangle);
	inline MFplane CalculateNormalizedSurfacePlane(const MFvec3& v0, const MFvec3& v1, const MFvec3& v2);
	MFplane CalculateSurfacePlane(const MFtriangle& triangle);		
	MFplane CalculateSurfacePlane(const MFvec3& v0, const MFvec3& v1, const MFvec3& v2);
}
