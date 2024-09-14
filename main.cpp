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
	BoundingSphere boundingSphere{ .center = {0,3.0,0}, .radius = 1.0 };
	ConvexHull hull;
	hull.worldSpace = Identity();		
	hull.mesh = QuickHull(pointCloud);
		
	MFbool collision{ false };
	constexpr MFfloat dt{ 1.0f / 20.0f };
	constexpr MFfloat dt2{ dt * dt };
	const MFvec3 gravity{ 0,-9.8,0.0 };
	const MFvec3 step{ gravity * dt2 };
	while (!collision)
	{		
		Simplex_T<Support> simplex;
		MFfloat distanceSquared;

		DLOG({ CONSOLE_BOLD }, "Beginning GJK Test with Sphere at:", boundingSphere.center, "and radius:", boundingSphere.radius, "and Hull at:", hull.worldSpace.GetTranslation());

		GJK(boundingSphere.center, hull, simplex, distanceSquared);

		if (distanceSquared <= boundingSphere.radius * boundingSphere.radius)
			collision = true;

		if (collision)
			DLOG({ CONSOLE_BG_GREEN,CONSOLE_BLACK, CONSOLE_BOLD, CONSOLE_BLINK }, "Shallow Contact Detected, distanceSquared:", distanceSquared);
		else
			DLOG({ CONSOLE_BG_RED,CONSOLE_BLACK, CONSOLE_BOLD, CONSOLE_BLINK }, "Separation Detected, distanceSquared:", distanceSquared);

		boundingSphere.center += step;
	}
};