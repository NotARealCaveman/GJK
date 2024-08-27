#pragma once
#include <vector>

#include <ManifestMath/HalfEdgeMesh.h>

#include "AxisAlignedBoxes.h"

using namespace Manifest_Math;

namespace Manifest_Simulation
{
	struct ConvexHull
	{		
		HalfEdgeMesh mesh;
		MFtransform worldSpace;					
	};

	AxisBoundingBox Encapsulate(const ConvexHull& hull);
}
