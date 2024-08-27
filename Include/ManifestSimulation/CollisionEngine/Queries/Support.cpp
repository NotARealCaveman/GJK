#include "Support.h"



extern MFpoint3 Manifest_Simulation::FindFurthestPoint(const MFpoint3& point, const MFvec3& direction, MFu32& vertexIndex)
{
	vertexIndex = 0;
	return point;
}

const MFpoint3 Manifest_Simulation::FindFurthestPoint(const MFpoint3(&segment)[2], const MFvec3& direction, MFu32& vertexIndex)
{
	if (Dot(segment[0], direction) > Dot(segment[1], direction))
	{
		vertexIndex = 0;
		return segment[0];
	}
	vertexIndex = 1;

	return  segment[1];
}

const MFpoint3 Manifest_Simulation::FindFurthestPoint(const ConvexHull& hull, const MFvec3& direction, MFu32& vertexIndex)
{ 
	std::vector<HullVertex*> visited;

	MFpoint3 bestPoint{ 0 };
	MFfloat max = -INFINITY;
	MFfloat current;
	MFu32 currentIndex{ 0 };
	HullVertex const * vertex{ hull.mesh.vertices };
	do
	{
		const MFpoint3 localPoint{ vertex->vertex };
		if ((current = (Dot(localPoint, direction))) > max)
		{
			vertexIndex = currentIndex;			
			bestPoint = localPoint;
			max = current;
		}
		vertex = vertex->next;
		++currentIndex;
	} while (vertex != hull.mesh.vertices);

	return hull.worldSpace * bestPoint;
}