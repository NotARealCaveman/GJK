#include "GJK.h"

using namespace Manifest_Simulation; 
 

//barycentric weights are set during closest point computation;
//all negative weights are set to 0 allowing simplex culling to copy valid points
//if a size change is detected the simplex was reduced
void Manifest_Simulation::CullSimplexVertices(Simplex_T<Support>& simplex, MFpoint3& closestPoint)
{		
	closestPoint = FindClosestPointOnSimplexFromOrigin(simplex);
	const MFfloat w0{ simplex[0].weight };
	const MFfloat w1{ simplex[1].weight };
	const MFfloat w2{ simplex[2].weight };
	const MFfloat w3{ simplex[3].weight };
	switch (simplex.Size())
	{ 		
		case 2:
		{
			if (w0 <= 0.0f)
			{
				simplex = { simplex[1] };
				return CullSimplexVertices(simplex, closestPoint);
			}
			else if (w1 <= 0.0f)
			{
				simplex = { simplex[0] };
				return CullSimplexVertices(simplex, closestPoint);
			}

			return;
		} 
		case 3:
		{
			if (w0 <= 0.0f)
			{
				simplex = { simplex[1],simplex[2]};
				return CullSimplexVertices(simplex, closestPoint);
			}
			else if (w1 <= 0.0f)
			{
				simplex = { simplex[0],simplex[2]};
				return CullSimplexVertices(simplex, closestPoint);
			}
			else if (w2 <= 0.0f)
			{
				simplex = { simplex[0],simplex[1] };
				return CullSimplexVertices(simplex, closestPoint);
			}

			return;
		}
		case 4:
		{
			//q lies within simplex - weights never computed after being set to 0(IEEE-754 guaranteed)
			if (w0 + w1 + w2 + w3 == 0.0f)
				return;

			if (w0 <= 0.0f)
			{
				simplex = { simplex[1],simplex[2],simplex[3]};
				return CullSimplexVertices(simplex, closestPoint);
			}
			else if (w1 <= 0.0f)
			{
				simplex = { simplex[0],simplex[2],simplex[3] };
				return CullSimplexVertices(simplex, closestPoint);
			}
			else if (w2 <= 0.0f)
			{
				simplex = { simplex[0],simplex[1],simplex[3] };
				return CullSimplexVertices(simplex, closestPoint);
			}
			else if (w3 <= 0.0f)
			{
				simplex = { simplex[0],simplex[1],simplex[2] };
				return CullSimplexVertices(simplex, closestPoint);
			}

			return;
		}
	}
}

const MFpoint3 Manifest_Simulation::FindClosestPointOnSimplexFromOrigin(Simplex_T<Support>& simplex)
{
	switch (simplex.Size())
	{
		case 1:
			return Barycentric({ 0.0f }, simplex[0].point, simplex[0].weight);
		case 2:
			return Barycentric({ 0.0f }, simplex[0].point, simplex[1].point, simplex[0].weight, simplex[1].weight);
		case 3:
			return Barycentric({ 0.0f }, simplex[0].point, simplex[1].point, simplex[2].point, simplex[0].weight, simplex[1].weight, simplex[2].weight);
		case 4:
			return Barycentric({ 0.0f }, simplex[0].point, simplex[1].point, simplex[2].point, simplex[3].point, simplex[0].weight, simplex[1].weight, simplex[2].weight, simplex[3].weight);
	}
	//shouldn't happen
	assert(0);
}

const MFvec3 Manifest_Simulation::SearchDirection(const Simplex_T<Support>& simplex)
{
	switch (simplex.Size())
	{
		case 1:
			return -simplex[0].point;
		case 2:
		{
			const MFvec3 AO{ -simplex[0].point };
			const MFvec3 AB{ simplex[1].point + AO };
			const MFvec3 ABxAO{ Cross(AB,AO) };
			const MFvec3 result{ Cross(ABxAO,AB) };

			return result;
		}
		case 3:
		{
			const MFplane surfacePlane{ CalculateNormalizedSurfacePlane(simplex[0].point,simplex[1].point,simplex[2].point) };
			//f[n|d]*q = n*q+d=n*0+d=d
			if (surfacePlane.w < 0.0f)
				return -surfacePlane.Normal();

			return surfacePlane.Normal();
		}
	}
	//shouldn't happen
	assert(0);
}

const MFbool Manifest_Simulation::ContainsOrigin(const Simplex_T<Support>& simplex)
{
	constexpr MFfloat TOLERANCE{ 1e-4 };

	switch (simplex.Size())
	{
		//check if origin is close enough to simplex
		case 1:
			return MagnitudeSquared(simplex[0].point) < TOLERANCE;
			//check if origin is within simplex "capsule"
		case 2:
			return MagnitudeSquared(ClosestPointFromlineSegment({ 0.0f }, simplex[0].point, simplex[1].point)) < TOLERANCE;
		//check if origin is close enough to simplex plane
		case 3://f[n|d]*q = n*q+d=n*0+d=d
			return std::powf(CalculateNormalizedSurfacePlane(simplex[0].point, simplex[1].point, simplex[2].point).w, 2) < TOLERANCE;
		//simplex 4 was unable to be culled - contains origin
		case 4:
			return true;
	}	
	//shouldn't happen
	assert(0);
}

const MFbool Manifest_Simulation::ContainsSupport(const Simplex_T<Support>& simplex, const Support& newPoint)
{	
	const auto checkDuplicate = [&](const Support& simplexPoint)->MFbool
		{
			return newPoint.vertexIndexA == simplexPoint.vertexIndexA && newPoint.vertexIndexB == simplexPoint.vertexIndexB;
		};

	return std::ranges::find_if(simplex, checkDuplicate) != simplex.end();
}