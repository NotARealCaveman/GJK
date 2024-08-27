#include "Capsule.h"

using namespace Manifest_Simulation;

AxisBoundingBox Manifest_Simulation::Encapsulate(const Capsule& capsule)
{
	const MFpoint3 min{ Min(capsule.pointsWorld[0],capsule.pointsWorld[1]) - capsule.radius};
	const MFpoint3 max{ Max(capsule.pointsWorld[0],capsule.pointsWorld[1]) + capsule.radius};
	const MFpoint3 center{ (min + max) * 0.5f };
	const MFvec3 halfLength{ vabsf(max - min) * 0.5f };

	return { center ,halfLength };
}	