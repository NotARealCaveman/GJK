#pragma once

#include "AxisAlignedBoxes.h"

using namespace Manifest_Math;

namespace Manifest_Simulation
{
	//a capsule is defined by two foci and 
	//IF SCALING AN OBJECT WITH A CAPSULE THE CAPSULEWILL EXPAND UNIFORMLY ALONG THE GREASTEST SCALAR AXIS
	struct Capsule
	{
		MFtransform worldSpace;
		MFpoint3 pointsLocal[2];//stored in local space. center of capsule is {0}
		MFpoint3 pointsWorld[2];//stored in world space. center of capsule is {T}
		MFfloat halfLength;//distance along internal normal from center to point
		MFfloat radius;				
	};
	AxisBoundingBox Encapsulate(const Capsule& capsule);
}