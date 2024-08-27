#pragma once
#include <stack>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <list>
#include <type_traits>

#include "Plane.h"
#include "Triangle.h"

namespace Manifest_Math
{
	struct HullVertex
	{
		HullVertex* prev{ nullptr };
		HullVertex* next{ nullptr };

		MFpoint3 vertex{ 0 };
		MFu32 referenceCount{ 0 };
	};
	using ConflictList = std::vector<HullVertex>;

	struct HullFace;

	struct HullHalfEdge
	{
		HullHalfEdge* prev{ nullptr };
		HullHalfEdge* next{ nullptr };
		HullHalfEdge* twin{ nullptr };
		HullVertex* tail{ nullptr };
		HullFace* face;

		MFbool deallocated{ false };
	};
	//returns normalized direction from edge tail to head
	MFvec3 GetEdgeDirection(HullHalfEdge const* const edge);
	inline MFpoint3 GetEdgeOrigin(HullHalfEdge const* const edge);
	inline MFvec3 GetEdgeNormal(HullHalfEdge const* const edge);
	inline MFvec3 GetTwinNormal(HullHalfEdge const* const edge);
	//TTY tracking functions - debug mode	
	void TrackEdge(HullHalfEdge* edge);
	void UntrackEdge(HullHalfEdge* edge);

	struct HullFace
	{
		HullFace* prev{ nullptr };
		HullFace* next{ nullptr };
		HullHalfEdge* edge{ nullptr };		
		MFplane facePlane;
		MFu32 vertexCount{ 0 };//equal to face's edge count

		std::list<HullVertex*> conflictList;
		MFbool deallocated{ false };		
	};
	void PrintFace(const HullFace* const face);
	std::vector<HullHalfEdge*> CollectFaceEdges(const HullFace* const face);
	std::vector<MFpoint3> CollectFaceVertices(const HullFace* const face);
	inline MFsize FaceVertexCount(const HullFace* const face);	
	MFplane CalculateFacePlane(const HullFace* const face);
	MFplane CalculateScaledFacePlane(HullFace const* const face, const MFvec3 scale);
	MFplane NewellPlane(const MFu32& planarEdgeCount, const HullFace* const planarFace); 
	
	struct HalfEdgeMesh
	{			
			std::unordered_set<HullHalfEdge*> edges;
			HullVertex* vertices{ nullptr };			
			HullFace* faces{ nullptr };
			MFsize vertexCount;
			MFsize edgeCount;
			MFsize faceCount;
			MFfloat CONVEXITY_EPSILON{ FLT_EPSILON };							

			//allocation buffers
			HullVertex* vertexBuffer{ nullptr };
			HullHalfEdge* edgeBuffer{ nullptr };
			HullFace* faceBuffer{ nullptr };
			//represents total number of allocations, only increases
			//once allocated either on hull or in free list
			MFsize allocatedVertices{ 0 };
			MFsize allocatedEdges{ 0 };
			MFsize allocatedFaces{ 0 };
			//allocates off freelist first if possible
			std::stack<HullVertex*> vertexFreeList;
			std::stack<HullHalfEdge*> edgeFreeList;
			std::stack<HullFace*> faceFreeList;
	};
	//allocates inital data stores
	void AllocateBuffers(const MFsize& numberVertices, HullVertex*& vertexBuffer, HullHalfEdge*& edgeBuffer, HullFace*& faceBuffer);
	void DeallocateBuffers(HullVertex*& vertexBuffer, HullHalfEdge*& edgeBuffer, HullFace*& faceBuffer);
	//de/allocates hull structures as required		
	void DeallocateVertex(std::stack<HullVertex*>& vertexFreeList, MFsize& vertexCount, HullVertex*& hullVertices, HullVertex*& deallocatedVertex);
	void DeallocateHalfEdge(std::stack<HullHalfEdge*>& edgeFreeList, HullHalfEdge*& deallocatedEdge);
	void DeallocateFace(std::stack<HullFace*>& faceFreeList, MFsize& faceCount, HullFace*& hullFaces, HullFace*& deallocatedFace);
	HullVertex* AllocateVertex(std::stack<HullVertex*>& vertexFreeList, MFsize& allocatedVertices, HullVertex*& vertexBuffer);
	HullHalfEdge* AllocateHalfEdge(std::stack<HullHalfEdge*>& edgeFreeList, MFsize& allocatedEdges, HullHalfEdge*& edgeBuffer);
	HullFace* AllocateFace(std::stack<HullFace*>& faceFreeList, MFsize& allocatedFaces, HullFace*& faceBuffer);

	template<typename T>
	void AddHalfEdgeMeshFeature(MFsize& featureCount, T*& meshFeatures, T* additionalFeature)
	{		
		if (meshFeatures)
		{
			auto back{ meshFeatures->prev };
			back->next = additionalFeature;
			additionalFeature->prev = back;			
		}
		else		
			meshFeatures = additionalFeature;	
		
		additionalFeature->next = meshFeatures;
		meshFeatures->prev = additionalFeature;
		
		++featureCount;
	}
	template<typename T>
	void RemoveHalfEdgeMeshFeature(MFsize& featureCount, T*& meshFeatures, T* removedFeature)
	{				
		if (meshFeatures == removedFeature)
			meshFeatures = meshFeatures->next;
		
		removedFeature->prev->next = removedFeature->next;
		removedFeature->next->prev = removedFeature->prev;			

		--featureCount;
	}
}