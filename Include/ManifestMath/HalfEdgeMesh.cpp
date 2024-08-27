#include "HalfEdgeMesh.h"

using namespace Manifest_Math;

MFvec3 Manifest_Math::GetEdgeDirection(const HullHalfEdge const* edge)
{
	return Normalize(GetEdgeOrigin(edge->twin) - GetEdgeOrigin(edge));
}

extern MFpoint3 Manifest_Math::GetEdgeOrigin(const HullHalfEdge const* edge)
{
	return edge->tail->vertex;
}

extern MFvec3  Manifest_Math::GetEdgeNormal(const HullHalfEdge const* edge)
{
	return Normalize(CalculateFacePlane(edge->face)).Normal();
}

extern MFvec3  Manifest_Math::GetTwinNormal(const HullHalfEdge const* edge)
{
	return Normalize(CalculateFacePlane(edge->twin->face)).Normal();
}


void Manifest_Math::TrackEdge(HullHalfEdge* edge)
{
#ifdef DEBUG
	++edge->tail->referenceCount;
	//DLOG(34, "adding edge: " << edge << " with vertex: " << edge->tail << "(" << edge->tail->vertex << ")  nReferences: " << edge->tail->referenceCount);
#endif // DEBUG	
}
void Manifest_Math::UntrackEdge(HullHalfEdge* edge)
{
#ifdef DEBUG
	--edge->tail->referenceCount;	
	//DLOG(34, "removing edge: " << edge << "(face: " << edge->face << ")  with vertex : " << edge->tail << " nReferences : " << edge->tail->referenceCount);
#endif // DEBUG	
}

void Manifest_Math::PrintFace(const HullFace* const face)
{
	DLOG({ CONSOLE_BG_YELLOW}, "printing vertices of face: ", face);
	auto edge{ face->edge };
	do
	{
		DLOG({ CONSOLE_WHITE}, edge->tail->vertex);
		edge = edge->next;
	} while (edge != face->edge);
}

std::vector<HullHalfEdge*> Manifest_Math::CollectFaceEdges(const HullFace* const face)
{
	//DLOG({ CONSOLE_WHITE }, "Gathering edges of face: " , face);
	std::vector<HullHalfEdge*> result{ face->edge };
	for (auto edge{ face->edge->next }; edge != face->edge; edge = edge->next)
		result.emplace_back(edge);

	return result;
}

std::vector<MFpoint3> Manifest_Math::CollectFaceVertices(const HullFace* const face)
{
	std::vector<MFpoint3> result;
	HullHalfEdge const * const start{ face->edge };
	HullHalfEdge const* edge{ start };
	do 
	{ 
		DLOG({ CONSOLE_BG_BLUE }, "edge:", edge,"vertex:",edge->tail);
		result.emplace_back(edge->tail->vertex); 
	} while ((edge = edge->next) != start);

	return result;
}

extern MFsize Manifest_Math::FaceVertexCount(const HullFace* const face)
{	
	return face->vertexCount;
} 

MFplane Manifest_Math::CalculateFacePlane(const HullFace* const face)
{
	MFplane result;	
	const MFsize planarVertexCount{ FaceVertexCount(face)};
	if (planarVertexCount > 3)
		result = NewellPlane(planarVertexCount, face);
	else
	{
		assert(planarVertexCount == 3);
		MFtriangle triangle
		{
		 face->edge->tail->vertex,
		 face->edge->next->tail->vertex,
		 face->edge->prev->tail->vertex
		};
		result = CalculateSurfacePlane(triangle);
	}

	return result;
}

MFplane Manifest_Math::CalculateScaledFacePlane(HullFace const* const face, const MFvec3 scale)
{
	const auto& ScaledNewellPlane = [&](const MFu32& planarEdgeCount, const HullFace* const planarFace)
		{
			MFvec3 normal{ 0 };
			HullHalfEdge const* planarEdge{ planarFace->edge };
			for (MFu32 edge{ 0 }; edge < planarEdgeCount; ++edge)
			{
				const MFpoint3& currentVertex{ ComponentMultiply(scale, planarEdge->tail->vertex) };
				const MFpoint3& nextVertex{ ComponentMultiply(scale, planarEdge->next->tail->vertex) };
				normal.x += (currentVertex.y - nextVertex.y) * (currentVertex.z + nextVertex.z);
				normal.y += (currentVertex.z - nextVertex.z) * (currentVertex.x + nextVertex.x);
				normal.z += (currentVertex.x - nextVertex.x) * (currentVertex.y + nextVertex.y);
				planarEdge = planarEdge->next;
			}			
			const MFfloat offset{ -Dot(normal,ComponentMultiply(scale,planarFace->edge->tail->vertex)) };

			return MFplane{ normal,offset };
		};

	MFplane result;
	const MFsize planarVertexCount{ FaceVertexCount(face) };
	if (planarVertexCount > 3)
		result = ScaledNewellPlane(planarVertexCount, face);
	else
	{
		assert(planarVertexCount == 3);
		MFtriangle triangle
		{
		 face->edge->tail->vertex,
		 face->edge->next->tail->vertex,
		 face->edge->prev->tail->vertex
		};
		result = CalculateSurfacePlane(triangle);
	}

	return result;
}

MFplane Manifest_Math::NewellPlane(const MFu32& planarEdgeCount, const HullFace* const planarFace)
{
	MFvec3 normal{ 0 };
	HullHalfEdge const* planarEdge{ planarFace->edge };
	for (MFu32 edge{ 0 }; edge < planarEdgeCount; ++edge)
	{		
		const MFpoint3& currentVertex{ planarEdge->tail->vertex };
		const MFpoint3& nextVertex{ planarEdge->next->tail->vertex };
		normal.x += (currentVertex.y - nextVertex.y) * (currentVertex.z + nextVertex.z);
		normal.y += (currentVertex.z - nextVertex.z) * (currentVertex.x + nextVertex.x);
		normal.z += (currentVertex.x - nextVertex.x) * (currentVertex.y + nextVertex.y);
		planarEdge = planarEdge->next;
	}	
	const MFfloat offset{ -Dot(normal,planarFace->edge->tail->vertex) };

	return MFplane{ normal,offset };
}

//uses Eurler's Formula and allocate double the amount required baseline
void Manifest_Math::AllocateBuffers(const MFsize& numberVertices, HullVertex*& vertexBuffer, HullHalfEdge*& edgeBuffer, HullFace*& faceBuffer)
{
	vertexBuffer = new HullVertex[numberVertices * 2];
	edgeBuffer = new HullHalfEdge[(3 * numberVertices - 6) * 2 * 2];//half edges,2*2
	faceBuffer = new HullFace[(2 * numberVertices - 4) * 2];
}

void Manifest_Math::DeallocateBuffers(HullVertex*& vertexBuffer, HullHalfEdge*& edgeBuffer, HullFace*& faceBuffer)
{
	//DLOG({ CONSOLE_ITALIC,CONSOLE_GREEN }, "Deallocating mesh buffers");
	if (vertexBuffer)
	{
		delete[] vertexBuffer;
		vertexBuffer = nullptr;
	}
	if (edgeBuffer)
	{
		delete[] edgeBuffer;
		edgeBuffer = nullptr;
	}
	if (faceBuffer)
	{
		delete[] faceBuffer;
		faceBuffer = nullptr;
	}
}

HullVertex* Manifest_Math::AllocateVertex(std::stack<HullVertex*>& vertexFreeList, MFsize& allocatedVertices, HullVertex*& vertexBuffer)
{
	HullVertex* result;
	if (vertexFreeList.size())
	{
		result = vertexFreeList.top();
		vertexFreeList.pop();
	}
	else
		result = &vertexBuffer[allocatedVertices++];

	return result;
}

HullHalfEdge* Manifest_Math::AllocateHalfEdge(std::stack<HullHalfEdge*>& edgeFreeList, MFsize& allocatedEdges, HullHalfEdge*& edgeBuffer)
{
	HullHalfEdge* result{ nullptr };
	//change this in a second to test if you can do a boolean set test
	if (edgeFreeList.size())
	{
		result = edgeFreeList.top();
		edgeFreeList.pop();
		//DLOG(35, "Allocating reuse edge: " << result);
	}
	else
	{
		result = &edgeBuffer[allocatedEdges++];
		//DLOG(36, "Allocating new edge: " << result);
	}
	//++edgeCount;	
	return new(result)HullHalfEdge;
}

HullFace* Manifest_Math::AllocateFace(std::stack<HullFace*>& faceFreeList, MFsize& allocatedFaces, HullFace*& faceBuffer)
{
	HullFace* result{ nullptr };
	if (faceFreeList.size())
	{
		result = faceFreeList.top();
		faceFreeList.pop();
		//DLOG(37, "Allocating reuse face: " << result);
	}
	else
	{
		result = &faceBuffer[allocatedFaces++];
		//DLOG(33, "Allocating new face: " << result);
	}

	return new(result)HullFace;
}
void Manifest_Math::DeallocateVertex(std::stack<HullVertex*>& vertexFreeList, MFsize& vertexCount, HullVertex*& hullVertices, HullVertex*& deallocatedVertex)
{
	//DLOG(44, "removing vertex: " << vertex << " " << vertex->vertex);
	//vertex should only be removed when no edges use it
	assert(deallocatedVertex->referenceCount == 0);
	//only needs to run if vertex was originally added to hull
	if (deallocatedVertex->prev)//will always have prev/next pointers if so
		RemoveHalfEdgeMeshFeature<HullVertex>(vertexCount, hullVertices, deallocatedVertex);
	vertexFreeList.push(deallocatedVertex);
}
void Manifest_Math::DeallocateHalfEdge(std::stack<HullHalfEdge*>& edgeFreeList, HullHalfEdge*& deallocatedEdge)
{
	//UntrackEdge(deallocatedEdge);
	deallocatedEdge->deallocated = true;
	edgeFreeList.push(deallocatedEdge);
}
void Manifest_Math::DeallocateFace(std::stack<HullFace*>& faceFreeList, MFsize& faceCount, HullFace*& hullFaces, HullFace*& deallocatedFace)
{
	//DLOG(34, "Deallocating face: " << face);
	RemoveHalfEdgeMeshFeature<HullFace>(faceCount, hullFaces, deallocatedFace);
	deallocatedFace->deallocated = true;
	faceFreeList.push(deallocatedFace);
}