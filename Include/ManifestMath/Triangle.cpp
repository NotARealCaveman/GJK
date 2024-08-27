#include "Triangle.h"

using namespace Manifest_Math;

const MFfloat Manifest_Math::EPSILON_3D(const std::vector<MFpoint3>& pointCloud)
{
	MFvec3 max{ -INFINITY };
	for (const auto& point : pointCloud)
	{
		max.x = std::max(max.x, std::fabsf(point.x));
		max.y = std::max(max.y, std::fabsf(point.y));
		max.z = std::max(max.z, std::fabsf(point.z));
	}

	return 3 * (max.x + max.y + max.z) * FLT_EPSILON;
}

//TRIANGLES
MFpoint3 Manifest_Math::Centroid(const MFtriangle& triangle)
{
	const MFfloat iDenom{ 1.0f / 3.0f };
	return MFpoint3
	{
	   (triangle.vertices[0].x + triangle.vertices[1].x + triangle.vertices[2].x) * iDenom,
	   (triangle.vertices[0].y + triangle.vertices[1].y + triangle.vertices[2].y) * iDenom,
	   (triangle.vertices[0].z + triangle.vertices[1].z + triangle.vertices[2].z) * iDenom
	};
}

MFvec3 Manifest_Math::CalculateSurfaceNormal(const MFtriangle& triangle)
{		
	return Normalize(CalculateSurfaceBivector(triangle));
}

MFvec3 Manifest_Math::CalculateSurfaceNormal(const MFvec3& v0, const MFvec3& v1, const MFvec3& v2)
{
	return Normalize(CalculateSurfaceBivector(v0,v1,v2));
}

MFvec3 Manifest_Math::CalculateSurfaceBivector(const MFtriangle& triangle)
{
	return Cross(triangle.vertices[1] - triangle.vertices[0], triangle.vertices[2] - triangle.vertices[0]);
}

MFvec3 Manifest_Math::CalculateSurfaceBivector(const MFvec3& v0, const MFvec3& v1, const MFvec3& v2)
{
	return Cross(v1 - v0, v2 - v0);
}

extern MFplane Manifest_Math::CalculateNormalizedSurfacePlane(const MFtriangle& triangle)
{
	return Normalize(CalculateSurfacePlane(triangle));
}

extern MFplane Manifest_Math::CalculateNormalizedSurfacePlane(const MFvec3& v0, const MFvec3& v1, const MFvec3& v2)
{
	return Normalize(CalculateSurfacePlane(v0, v1, v2));
}

MFplane Manifest_Math::CalculateSurfacePlane(const MFtriangle& triangle)
{
	auto bivector{ CalculateSurfaceBivector(triangle) };
	auto offset{ Dot(-bivector,triangle.vertices[0]) };
	return { bivector,offset };
}

MFplane Manifest_Math::CalculateSurfacePlane(const MFvec3& v0, const MFvec3& v1, const MFvec3& v2)
{
	auto bivector{ CalculateSurfaceBivector(v0,v1,v2)};
	auto offset{ -Dot(bivector,v0) };
	return { bivector,offset };
}