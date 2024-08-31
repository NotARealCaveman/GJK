#pragma once
#include "vector"

#include "ManifestMath/QuickHull.h"
#include "Include/ManifestSimulation/CollisionEngine/Queries/GJK.h";

using namespace Manifest_Simulation;
using namespace Manifest_Math;

const std::vector<MFpoint3> pointCloud
{
		{-1, -1, -1},{-1, 1, -1},{1, 1, -1},{1, -1, -1},
		{-1, -1, 1},{1, -1, 1},{-1, 1, 1},{1, 1, 1}
};

int main()
{
	BoundingSphere boundingSphere{ .center = {0,3.0,0}, .radius = 1 };
	ConvexHull hull;
	hull.worldSpace = Identity();		
	hull.mesh = QuickHull(pointCloud);

	Simplex_T<Support> simplex;
	MFfloat distanceSquared;
	DLOG({ CONSOLE_BOLD }, "Beginning GJK Test with Sphere at:", boundingSphere.center, "and radius:", boundingSphere.radius, "and Hull at:", hull.worldSpace.GetTranslation());
	const MFbool gjkResult{ GJK(boundingSphere.center,hull,simplex, distanceSquared) };

	if (gjkResult)
	{
		if (distanceSquared <= boundingSphere.radius * boundingSphere.radius)
			DLOG({ CONSOLE_BG_GREEN,CONSOLE_BLACK, CONSOLE_BOLD, CONSOLE_BLINK }, "Shallow Contact Detected, distanceSquared:", distanceSquared);
		else
			DLOG({ CONSOLE_BG_RED,CONSOLE_BLACK, CONSOLE_BOLD, CONSOLE_BLINK }, "Separation Detected, distanceSquared:", distanceSquared);
	}
	else
		DLOG({ CONSOLE_BG_RED,CONSOLE_BLACK, CONSOLE_BOLD, CONSOLE_BLINK }, "Deep Contact Detected");
}