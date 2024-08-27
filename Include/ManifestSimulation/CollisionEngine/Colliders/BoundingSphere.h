#pragma once

#include "AxisAlignedBoxes.h"

using namespace Manifest_Math;

namespace Manifest_Simulation
{
	//contains all the information for creating and working spheres - intersection tests done in BoundingVolume.h
	//a sphere is the first bounding volume - represented as a center in local space and some radius	
	//IF SCALING AN OBJECT WITH A BOUNDING SPHERE THE SPHERE WILL EXPAND UNIFORMLY ALONG THE GREASTEST SCALAR AXIS
	struct BoundingSphere
	{		
		MFpoint3 center;
		MFfloat radius;					
	};

	AxisBoundingBox Encapsulate(const BoundingSphere& sphere);
}