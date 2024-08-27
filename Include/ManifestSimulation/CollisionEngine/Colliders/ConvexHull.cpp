#include "ConvexHull.h"

using namespace Manifest_Simulation;

AxisBoundingBox Manifest_Simulation::Encapsulate(const ConvexHull& hull)
{
	MFpoint3 min{ INFINITY }, max{ -INFINITY };

	HullVertex const* const start{ hull.mesh.vertices };
	HullVertex const* vertex{ start };
	do
	{
		min = Min(min, vertex->vertex);
		max = Max(max, vertex->vertex);
	} while ((vertex = vertex->next) != start);

	const MFpoint3 center{ (min + max) * 0.5f };
	const MFvec3 halfLength{ (max - min) * 0.5f };

	return { hull.worldSpace * center, halfLength };
}
