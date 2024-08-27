#pragma once
#include <ManifestSimulation/CollisionEngine/Collider.h>

namespace Manifest_Simulation
{		
	inline MFpoint3 FindFurthestPoint(const MFpoint3& point, const MFvec3& direction, MFu32& vertexIndex);
	const MFpoint3 FindFurthestPoint(const MFpoint3(& segment)[2], const MFvec3& direction, MFu32& vertexIndex);	
	const MFpoint3 FindFurthestPoint(const ConvexHull& hull, const MFvec3& direction, MFu32& vertexIndex);

	struct Support
	{
		MFpoint3 sA;
		MFpoint3 sB;
		MFpoint3 point;
		MFfloat weight;
		MFu32 vertexIndexA;
		MFu32 vertexIndexB;
	};

	template<typename Geometry>
	Support SupportPoint(const Geometry& geometry, const ConvexHull& hull, const MFvec3& direction)
	{
		Support result;		
		const MFvec3 localDirection{ Inverse(hull.worldSpace) * direction };		
		result.sA = FindFurthestPoint(geometry, localDirection, result.vertexIndexA);
		result.sB = FindFurthestPoint(hull, -localDirection, result.vertexIndexB);
		//DLOG({ CONSOLE_MAGENTA }, "sA:", result.sA, "sB:", result.sB, "-direction:",-direction);
		result.point = result.sA - result.sB;

		return result;
	}
};