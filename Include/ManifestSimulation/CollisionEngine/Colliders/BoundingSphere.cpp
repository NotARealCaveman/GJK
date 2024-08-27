#include "BoundingSphere.h"

using namespace Manifest_Simulation;

AxisBoundingBox Manifest_Simulation::Encapsulate(const BoundingSphere& sphere)
{
	return { sphere.center, sphere.radius };
}