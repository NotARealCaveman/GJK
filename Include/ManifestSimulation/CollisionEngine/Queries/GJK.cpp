#include "GJK.h"

using namespace Manifest_Simulation; 
 

//barycentric weights are set during closest point computation;
//all negative weights are set to 0 allowing simplex culling to copy valid points
//if a size change is detected the simplex was reduced
void Manifest_Simulation::CullSimplexVertices(Simplex_T<Support>& simplex, MFpoint3& closestPoint)
{

}

const MFpoint3 Manifest_Simulation::FindClosestPointOnSimplexFromOrigin(Simplex_T<Support>& simplex)
{
}

const MFvec3 Manifest_Simulation::SearchDirection(const Simplex_T<Support>& simplex)
{
}

const MFbool Manifest_Simulation::ContainsOrigin(const Simplex_T<Support>& simplex)
{
}

const MFbool Manifest_Simulation::ContainsSupport(const Simplex_T<Support>& simplex, const Support& newPoint)
{
	
}