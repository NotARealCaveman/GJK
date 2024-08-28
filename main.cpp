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
	BoundingSphere boundingSphere{ .center = {0,1.0,0}, .radius = 1 };
	ConvexHull hull;
	hull.worldSpace = Identity();		
	hull.mesh = QuickHull(pointCloud);

	Simplex_T<Support> simplex;
	MFpoint3 closestPoint;
	MFfloat distanceSquared;
	const MFbool objectsSeparated{ GJK(boundingSphere.center,hull,simplex, closestPoint,distanceSquared) };

	DLOG({ CONSOLE_BG_CYAN,CONSOLE_BLACK, CONSOLE_BOLD, CONSOLE_BLINK }, "objectsSeparated:", objectsSeparated, "closestPoint:", closestPoint, "distanceSquared:", distanceSquared);
}