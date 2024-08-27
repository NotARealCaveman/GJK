#include "Quickhull.h"

using namespace Manifest_Math;

void Manifest_Math::STUPID_EDGE_COLLECTION_HACK(HalfEdgeMesh& mesh)
{	
	auto startFaces{ mesh.faces };
	auto face{ startFaces };
	do
	{
		auto startEdges{ face->edge};
		auto edge{ startEdges };
		do
		{
			mesh.edges.insert(edge);
		} while ((edge = edge->next) != startEdges);
	} while ((face = face->next) != startFaces);
}

HalfEdgeMesh Manifest_Math::QuickHull(std::vector<MFpoint3> pointCloud)
{	
	HalfEdgeMesh result;

	//initiailze convex hull build buffers	
	AllocateBuffers(pointCloud.size(),result.vertexBuffer,result.edgeBuffer,result.faceBuffer);
	//set hull tolerance
	result.CONVEXITY_EPSILON = EPSILON_3D(pointCloud);
	//build initial tetrahedron
	MFvec4 CONSTRUCTION_NORMAL;//disgusting but im over it - make it work
	//creates and links initial data structures
	ConstructInitialHull(ConstructInitialSimplex(pointCloud, CONSTRUCTION_NORMAL), CONSTRUCTION_NORMAL, pointCloud,result);
	assert(AssertConvexity(result));
	//add conflict vertices to list until none remain
	HullVertex* conflictVertex{ nullptr };	
	//current face being operated on			
	HullFace* conflictFace{ nullptr };
	while (conflictVertex = NextConflictVertex(result, conflictFace))
		AddVertexToHull(result, conflictFace,conflictVertex);

	//PLEASE CHANGE THIS 
	STUPID_EDGE_COLLECTION_HACK(result);

	return result;
}

void Manifest_Math::AddPointToHull(HalfEdgeMesh& mesh,const MFpoint3& newPoint)
{
	//pray we don't run out of memory
	HullVertex* vertex{ AllocateVertex(mesh.vertexFreeList,mesh.allocatedVertices,mesh.vertexBuffer) };
	vertex->vertex = newPoint;
	//find face which maximizes the distance to the hull	
	HullFace* conflictFace{ nullptr };
	auto furthestDistance{ -INFINITY };	
	auto face{ mesh.faces };
	do
	{		
		const auto surfacePlane{ Normalize(CalculateFacePlane(face)) };
		const auto distanceToPlane{ Dot(surfacePlane,newPoint) };	
		if (distanceToPlane > furthestDistance)
		{
			furthestDistance = distanceToPlane;
			conflictFace = face;
		}
		face = face->next;
	} while (face != mesh.faces);
	//point is visible from hull, add it
	if (conflictFace)
	{ 	
		AddVertexToHull(mesh,conflictFace,vertex);		
	}		
}

HullVertex* Manifest_Math::NextConflictVertex(HalfEdgeMesh& mesh, HullFace*& conflictFace)
{
	HullVertex* result{ nullptr };
	conflictFace = nullptr;
	auto furthestDistance{ 0 };	
	auto face{ mesh.faces };
	do
	{				
		const auto surfacePlane{ Normalize(CalculateFacePlane(face)) };
		for (auto conflictVertex : face->conflictList)
		{			
			const auto distanceToPlane{ Dot(surfacePlane,conflictVertex->vertex) };
			//point is furthest
			if (distanceToPlane > furthestDistance)
			{
				furthestDistance = distanceToPlane;
				conflictFace = face;
				result = conflictVertex;
			}
		}
		face = face->next;
	} while (face != mesh.faces);
	//remove result from face
	if(result)			
		conflictFace->conflictList.remove(result);

	return result;
}

void Manifest_Math::AddVertexToHull(HalfEdgeMesh& mesh, HullFace* conflictFace, HullVertex* vertex)
{
	//add point into hull vertex list 
	AddHalfEdgeMeshFeature<HullVertex>(mesh.vertexCount, mesh.vertices, vertex);
	//build horizion
	std::list<HullHalfEdge*> horizon;
	std::vector<HullFace*> visibleFaces{ BuildHorizon(mesh,vertex,horizon,conflictFace) };
	std::vector<HullFace*> newFaces;
	//build and merge new faces, resolve orphaned conflict points
	BuildNewFaces(mesh,vertex, newFaces, horizon);
	MergeFaces(mesh,newFaces, conflictFace);
	std::_Erase_remove_if(newFaces, [&](HullFace* face) {return face->deallocated; });
	UpdateHullFaces(mesh,newFaces, visibleFaces, conflictFace);
	assert(AssertConvexity(mesh));
}

//using the conflict face and vertex builds the horizon using DFS
std::vector<HullFace*> Manifest_Math::BuildHorizon(HalfEdgeMesh& mesh, HullVertex* const vertex, std::list<HullHalfEdge*>& horizon, HullFace* conflictFace)
{		
	std::vector<HullFace*> result;
	std::unordered_set<HullFace*> visitedFaces;	
	HorizonDFS(conflictFace, vertex, visitedFaces, horizon, result,mesh.CONVEXITY_EPSILON);

	return result;
}
//if the face is new for the iteration, crosses over to the current face's edge's twin's face until the horizon is detected and collected on the way back up
void Manifest_Math::HorizonDFS(HullFace* face, const HullVertex* const vertex, std::unordered_set<HullFace*>& visitedFaces, std::list<HullHalfEdge*>& horizon, std::vector<HullFace*>& visibleFaces, const MFfloat epsilon)
{	
	const auto& IsFaceVisible = [&](const HullFace* const face, const HullVertex* const vertex, const MFfloat epsilon)->MFbool
	{
			MFplane surfacePlane{ Normalize(CalculateFacePlane(face)) };
		 
		return Dot(surfacePlane, vertex->vertex) > -epsilon;
	};
	
	if (visitedFaces.find(face) == visitedFaces.end())
	{
		visibleFaces.emplace_back(face);
		 
		visitedFaces.insert(face);
		std::unordered_set<HullHalfEdge*> visitedEdges;
		auto edge{ face->edge };
		while (visitedEdges.find(edge) == visitedEdges.end())
		{
			
			visitedEdges.insert(edge);
			auto next{ edge->twin->face };
			if (IsFaceVisible(next, vertex, epsilon))				
				HorizonDFS(next, vertex, visitedFaces, horizon, visibleFaces,epsilon);			
			else
			{ 
				horizon.emplace_back(edge);
			}							
			edge = edge->next;
		}				
	}
}

void Manifest_Math::BuildNewFaces(HalfEdgeMesh& mesh, HullVertex* const vertex, std::vector<HullFace*>& newFaces, std::list<HullHalfEdge*>& horizon)
{
	std::vector<HullHalfEdge*> newEdges;

	//pairs newly created edges with their twins
	const auto& FindTwin = [&](HullHalfEdge* edge)->HullHalfEdge*
	{
		const auto twin{ std::find_if(newEdges.begin(), newEdges.end(), [&](HullHalfEdge* potentialTwin)->MFbool
			{
				//edge begins at eye, find twin edge that terminates at the vertex and begins at the shared horizon vertex
				if (edge->tail == vertex)
					return potentialTwin->tail == edge->next->tail;
				//edge terminates at eye, find twin edge that terminates ate shared horizon vertex beginning from eye
				return potentialTwin->tail == vertex && potentialTwin->next->tail == edge->tail;
			}) };		
		if (twin == newEdges.end())
			return nullptr;
	 
		return *twin;
	};	

	for (HullHalfEdge* edge : horizon)
	{
		HullHalfEdge* e0{ AllocateHalfEdge(mesh.edgeFreeList,mesh.allocatedEdges,mesh.edgeBuffer) };
		HullHalfEdge* e1{ edge };
		HullHalfEdge* e2{ AllocateHalfEdge(mesh.edgeFreeList,mesh.allocatedEdges,mesh.edgeBuffer) };
		mesh.edgeCount += 2;
		HullFace* face{ AllocateFace(mesh.faceFreeList,mesh.allocatedFaces,mesh.faceBuffer) };
		//set inital edge/vertex count
		face->vertexCount = 3;
		//set pointers
		face->edge = e0;
		e0->face = e1->face = e2->face = face;
		//eye point
		e0->prev = e2;
		e0->next = e1;
		e0->tail = vertex;
		//store e2 tail before updating horizon edge
		e2->tail = edge->next->tail;
		//horizon edge, retains old tail and twin
		e1->prev = e0;
		e1->next = e2;
		//adjoining edge		
		e2->prev = e1;
		e2->next = e0;
		//add new edges and faces
		TrackEdge(e0); 
		TrackEdge(e2);
		newEdges.emplace_back(e0);
		newEdges.emplace_back(e2);
		newFaces.emplace_back(face);
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, face);
		for (const auto& newEdge : newEdges)		
			if(newEdge->twin = FindTwin(newEdge))
				newEdge->twin->twin = newEdge;
	}
}

MFbool Manifest_Math::CheckFaceConvexity(const HullFace* const centroidFace, const HullFace* const planarFace, const MFfloat& HULL_EPSILON)
{
	//calculate centroid	
	MFpoint3 centroid{ 0 };
	const auto centroidVertexCount{ FaceVertexCount(centroidFace) };
	auto faceEdge{ centroidFace->edge };
	for (auto edge{ 0 }; edge < centroidVertexCount; ++edge)
	{
		centroid += faceEdge->tail->vertex;
		faceEdge = faceEdge->next;
	}
	centroid /= centroidVertexCount;
	//calculate face plane		
	MFplane surfacePlane{ Normalize(CalculateFacePlane(planarFace)) };

	return Dot(surfacePlane, centroid) < -HULL_EPSILON;
};

//Detect concave/coplanar faces and merge as nessecary
void Manifest_Math::MergeFaces(HalfEdgeMesh& mesh, std::vector<HullFace*>& newFaces, HullFace* conflictFace)
{

	/*********face merging**********\
	*twin /|\	edge  ->	 /\		*
	*	 / | \		  ->	/  \	*
	*   / t|e \		  ->   /new \	*
	*	\  |  / 	  ->   \face/	*
	*	 \ | /		  ->	\  /	*
	*face \|/   face  ->	 \/		*
	\*******************************/
	const auto& MergeNonConvexFaces = [&](HullHalfEdge* edge)
	{ 
		HullFace* face{ edge->face };
		HullFace* twinFace{ edge->twin->face };
		//remove potentially dangling pointer			
		face->edge = edge->prev;
		//absorb old face edges into new face				
		for (auto twinEdge{ edge->twin->next }; twinEdge != edge->twin; twinEdge = twinEdge->next)
			twinEdge->face = face;
		//link edges of merged faces		
		edge->prev->next = edge->twin->next;		
		edge->next->prev = edge->twin->prev;		
		edge->twin->prev->next = edge->next;		
		edge->twin->next->prev = edge->prev;
		//deallocate face and edges
		UntrackEdge(edge);
		DeallocateHalfEdge(mesh.edgeFreeList,edge);
		UntrackEdge(edge->twin);
		DeallocateHalfEdge(mesh.edgeFreeList, edge->twin);
		mesh.edgeCount -= 2;
		//if deallocated face has any conflict points add them to the conflict face to be resolved with other orphaned points
		conflictFace->conflictList.insert(conflictFace->conflictList.end(), twinFace->conflictList.begin(), twinFace->conflictList.end());
		DeallocateFace(mesh.faceFreeList,mesh.faceCount,mesh.faces, twinFace);
		//add new edge/vertex to merge face
		++face->vertexCount;
	};

	//remove redundant vertex p, and incoming/outgoing edges
	//merge non shared edge into face
	/******topological error 1******\
	*		/-\		 ->		/ \		*
	*	f1 / - \ f3	 ->	   /   \ 	*
	*     /  -  \	 ->   /     \	*
	*    /i__-f13\	 ->  /-f123  \	*
	*    \  o|p  / 	 ->  \   -   / 	*
	*     \f2|  /	 ->   \  -  /	*
	* 	   \ | /	 ->    \ - /	*
	* 		\|/		 -> 	\-/		*
	\*******************************/
	const auto& TopologicalCorection1 = [&](HullHalfEdge* incoming, HullHalfEdge* outgoing)
	{ 
		//remove potentially dangling pointer
		auto face{ incoming->face };
		face->edge = incoming->prev;
		//find non shared edge of neighboring triangle		
		auto nonSharedEdge{ incoming->twin };
		while (nonSharedEdge == outgoing->twin || nonSharedEdge == incoming->twin)
			nonSharedEdge = nonSharedEdge->next;
		//merge non shared edge into face		
		incoming->prev->next = nonSharedEdge;		
		outgoing->next->prev = nonSharedEdge;		
		nonSharedEdge->prev = incoming->prev;		
		nonSharedEdge->next = outgoing->next;
		nonSharedEdge->face = face;		
		//remove redundant references
		UntrackEdge(incoming);
		DeallocateHalfEdge(mesh.edgeFreeList, incoming);
		UntrackEdge(outgoing);
		DeallocateHalfEdge(mesh.edgeFreeList, outgoing);
		UntrackEdge(incoming->twin);
		DeallocateHalfEdge(mesh.edgeFreeList, incoming->twin);
		UntrackEdge(outgoing->twin);
		DeallocateHalfEdge(mesh.edgeFreeList, outgoing->twin);
		mesh.edgeCount -= 4;
		DeallocateVertex(mesh.vertexFreeList, mesh.vertexCount, mesh.vertices, outgoing->tail);
		//if deallocated face has any conflict points add them to the conflict face to be resolved with other orphaned points
		conflictFace->conflictList.insert(conflictFace->conflictList.end(), incoming->twin->face->conflictList.begin(), incoming->twin->face->conflictList.end());
		DeallocateFace(mesh.faceFreeList, mesh.faceCount, mesh.faces, incoming->twin->face);
		//add new edge/vertex to merge face
		++face->vertexCount;
	};

	//f3 has more than 3 vetices, remove outgoing vertex and edge
	//extend incoming edge to old next of outgoing. removes potential concavities as well
	/******topological error 2******\
	*		/|\		 ->		/|\		*
	*	f1 / | \ f3	 ->	 f1/ | \ 	*
	*     /  |  \	 ->   /  |  \	*
	*    /---|o  \	 ->  /   |f3 \	*
	*    \f12|p  / 	 ->  \f12|   / 	*
	*  f2 \  |i /	 -> f2\  |i /	*
	* 	   \ | /	 ->    \ | /	*
	* 		\|/		 -> 	\|/		*
	\*******************************/
	const auto& TopologicalCorection2 = [&](HullHalfEdge* incoming, HullHalfEdge* outgoing)
	{ 
		auto face{ incoming->face };
		face->edge = incoming->prev;
		//stretch incoming to old outgoing next
		//outgoing twin over old incoming twin
		auto twinFace{ incoming->twin->face };
		twinFace->edge == incoming->twin->next;
		incoming->next = outgoing->next;
		outgoing->next->prev = incoming;
		outgoing->twin->next = incoming->twin->next;
		incoming->twin->next->prev = outgoing->twin;
		//remove redundant references		
		UntrackEdge(outgoing);
		DeallocateHalfEdge(mesh.edgeFreeList, outgoing);
		UntrackEdge(incoming->twin);
		DeallocateHalfEdge(mesh.edgeFreeList, incoming->twin);
		mesh.edgeCount -= 2;
		DeallocateVertex(mesh.vertexFreeList,mesh.vertexCount,mesh.vertices, outgoing->tail);//incoming->twin->tail
		incoming->twin = outgoing->twin;
		outgoing->twin->twin = incoming;
		//remove old edge/vertex counts from faces
		--face->vertexCount;
		--twinFace->vertexCount;
	};

	for (HullFace* face : newFaces)
	{ 
		if (face->deallocated)
			continue;
		std::unordered_set<HullHalfEdge*> visitedFaceEdges;
		auto edge{ face->edge };
		while (visitedFaceEdges.find(edge) == visitedFaceEdges.end())
		{
			visitedFaceEdges.insert(edge);
			if (edge->deallocated || edge->twin->face->deallocated)
			{
				edge = edge->next;				
				continue;
			} 
			auto isFaceConvex(CheckFaceConvexity(edge->face, edge->twin->face, mesh.CONVEXITY_EPSILON));
			auto isTwinConvex(CheckFaceConvexity(edge->twin->face, edge->face, mesh.CONVEXITY_EPSILON));
			if (isFaceConvex && isTwinConvex)
			{
				edge = edge->next;
				continue;
			}
			MergeNonConvexFaces(edge);
			//check if face merge introduce topological violation
			auto start{ face->edge };
			auto incoming{ start };
			do
			{
				const auto outgoing{ incoming->next };
				const auto& incomingTwinFace{ incoming->twin->face };
				const auto& outgoingTwinFace{ outgoing->twin->face };
				if (incomingTwinFace == outgoingTwinFace)
				{					
					const auto twinVertexCount{ FaceVertexCount(incomingTwinFace) };
					if (twinVertexCount >= 4)					
						TopologicalCorection2(incoming, outgoing);					
					else
						TopologicalCorection1(incoming, outgoing);
					start = incoming = face->edge;
				}
				incoming = incoming->next;
			} while (incoming != start);
			visitedFaceEdges.clear();
			edge = edge->next;
		}		
	}
}

void Manifest_Math::UpdateHullFaces(HalfEdgeMesh& mesh, std::vector<HullFace*>& newFaces, std::vector<HullFace*>& visibleFaces, HullFace* conflictFace)
{	
	//deallocate conflict faces
	for (auto& face : visibleFaces)
	{
		if (face != conflictFace)
			conflictFace->conflictList.insert(conflictFace->conflictList.end(), face->conflictList.begin(), face->conflictList.end());
		DeallocateFace(mesh.faceFreeList, mesh.faceCount, mesh.faces, face);
	}	
	//partition remaining conflicting points from conflict list
	ResolveOrphans(mesh,newFaces, conflictFace);
}

void Manifest_Math::ResolveOrphans(HalfEdgeMesh& mesh,std::vector<HullFace*>& newFaces, HullFace* conflictFace)
{
	for (auto point : conflictFace->conflictList)
	{
		HullFace* furthest{ nullptr };
		auto maxDistance{ 0 };
		for (auto face : newFaces)
		{
			//calculate face plane		
			MFplane surfacePlane{ Normalize(CalculateFacePlane(face)) };
			//get distance to surface plane from point
			auto planeDistance{ Dot(surfacePlane,point->vertex) };
			if (planeDistance > maxDistance)
			{
				maxDistance = planeDistance;
				furthest = face;
			}
		}
		if (!furthest)
		{
			DeallocateVertex(mesh.vertexFreeList, mesh.vertexCount, mesh.vertices, point);
			continue;
		}
		furthest->conflictList.emplace_back(point); 
	}
}

void Manifest_Math::ConstructInitialHull(const Simplex_T<MFpoint3>& simplex, const MFvec4& CONTRUCTION_NORMAL_IN, std::vector<MFpoint3>& pointCloud, HalfEdgeMesh& mesh)
{
	//remove simplex and excess points from pointcloud
	std::_Erase_remove_if(pointCloud, [&](const auto& p) {return std::ranges::find(simplex, p) != simplex.end(); });
	std::sort(pointCloud.begin(), pointCloud.end());	
	pointCloud.erase(std::unique(pointCloud.begin(), pointCloud.end()), pointCloud.end());	
	//set initial hull vertices
	auto v0{ AllocateVertex(mesh.vertexFreeList,mesh.allocatedVertices,mesh.vertexBuffer) };
	auto v1{ AllocateVertex(mesh.vertexFreeList,mesh.allocatedVertices,mesh.vertexBuffer) };
	auto v2{ AllocateVertex(mesh.vertexFreeList,mesh.allocatedVertices,mesh.vertexBuffer) };
	auto v3{ AllocateVertex(mesh.vertexFreeList,mesh.allocatedVertices,mesh.vertexBuffer) };
	//add current simplex points to hull
	v0->vertex = simplex[0];	
	v1->vertex = simplex[1];
	v2->vertex = simplex[2];
	v3->vertex = simplex[3];
	AddHalfEdgeMeshFeature<HullVertex>(mesh.vertexCount, mesh.vertices, v0);
	AddHalfEdgeMeshFeature<HullVertex>(mesh.vertexCount, mesh.vertices, v1);
	AddHalfEdgeMeshFeature<HullVertex>(mesh.vertexCount, mesh.vertices, v2);
	AddHalfEdgeMeshFeature<HullVertex>(mesh.vertexCount, mesh.vertices, v3);	 
	//set initial hull half edges 	
	const auto CreateHullFace =[&](HullVertex* v0, HullVertex* v1, HullVertex* v2)->HullFace*
	{
		HullFace* result{ AllocateFace(mesh.faceFreeList,mesh.allocatedFaces,mesh.faceBuffer) };
		HullHalfEdge* edges[3]{ AllocateHalfEdge(mesh.edgeFreeList,mesh.allocatedEdges,mesh.edgeBuffer) ,AllocateHalfEdge(mesh.edgeFreeList,mesh.allocatedEdges,mesh.edgeBuffer) ,AllocateHalfEdge(mesh.edgeFreeList,mesh.allocatedEdges,mesh.edgeBuffer) };
		mesh.edgeCount += 3;

		edges[0]->tail = v0;
		edges[1]->tail = v1;
		edges[2]->tail = v2;
		edges[0]->face = edges[1]->face = edges[2]->face = result;
		result->edge = edges[0];
		result->vertexCount = 3;
		//add edges to half edge mesh		
		edges[0]->prev = edges[2];
		edges[0]->next = edges[1];
		edges[1]->prev = edges[0];
		edges[1]->next = edges[2];
		edges[2]->prev = edges[1];
		edges[2]->next = edges[0];
		//can be removed - used in asserting vertex convexity
		TrackEdge(edges[1]);
		TrackEdge(edges[0]);
		TrackEdge(edges[2]); 

		return result;
	};	

	//pairs newly created edges with their twins
	const auto& FindTwin = [&](HullHalfEdge* edge)->HullHalfEdge*
	{				
		auto face{ mesh.faces };
		do
		{
			if (face == edge->face)
			{
				face = face->next;
				continue;
			}
			auto potentialTwin{ face->edge };
			do
			{
				if (edge->tail == potentialTwin->next->tail && edge->next->tail == potentialTwin->tail)
				{ 
					return potentialTwin;
				}

				potentialTwin = potentialTwin->next;
			} while (potentialTwin != face->edge);

			face = face->next;
		} while (face != mesh.faces);

		assert(("Failed to find twin", nullptr != nullptr));
	};
	
	//vertex orientation from john lloyd's java implementation
	if (Dot(simplex[3], reinterpret_cast<const MFvec3&>(CONTRUCTION_NORMAL_IN.x)) - CONTRUCTION_NORMAL_IN.w < 0)
	{
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, CreateHullFace(v0, v1, v2));
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, CreateHullFace(v3, v1, v0));
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, CreateHullFace(v3, v2, v1));
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, CreateHullFace(v3, v0, v2));
	}
	else
	{
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, CreateHullFace(v0, v2, v1));
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, CreateHullFace(v3, v0, v1));
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, CreateHullFace(v3, v1, v2));
		AddHalfEdgeMeshFeature<HullFace>(mesh.faceCount, mesh.faces, CreateHullFace(v3, v2, v0));
	}
	auto face{ mesh.faces};
	do
	{
		auto edge{ face->edge };
		for (auto edgeIndex{0}; edgeIndex < 3;++edgeIndex)
		{			
			if (edge->twin)
				continue;
			edge->twin = FindTwin(edge);
			edge->twin->twin = edge;
			edge = edge->next;
		}
		face = face->next;
	}while (face != mesh.faces);		
	
	//partition each point into face conflict lists
	for (const auto& point : pointCloud)
	{
		HullFace* furthestFace{ nullptr };
		auto furthestDistance{ 0 };
		auto face{ mesh.faces };
		do
		{
			const auto surfacePlane{ Normalize(CalculateFacePlane(face)) };
			const auto distancePointToPlane{ Dot(surfacePlane, point) };
			//point is visible from face && furthest				
			if (distancePointToPlane > furthestDistance)
			{
				furthestDistance = distancePointToPlane;
				furthestFace = face;
			}		
			face = face->next;
		} while (face != mesh.faces);

		if (!furthestFace)
			continue;//point inside current hull

		HullVertex* vertex{ AllocateVertex(mesh.vertexFreeList,mesh.allocatedVertices,mesh.vertexBuffer)};
		vertex->vertex = point;
		furthestFace->conflictList.emplace_back(vertex); 
	}
}

struct MFvec2
{
	MFfloat x;
	MFfloat y;
};

Simplex_T<MFpoint3> Manifest_Math::ConstructInitialSimplex(const std::vector<MFpoint3>& pointCloud, MFvec4& CONTRUCTION_NORMAL_OUT)
{
	//build initial hull as simplex
	Simplex_T<MFpoint3> result;

	//check each pair of points, record pair with greatest distance
	const auto cloudsize{ pointCloud.size() };	
	auto pairIndices{ new MFvec2[cloudsize * cloudsize] };
	auto pairDistances{ new MFfloat[cloudsize * cloudsize] };
	
	auto pair{ 0 };
	auto index{ 0 };
	auto currentDistance{ -INFINITY };
	for (auto p0{ 0 }; p0 < cloudsize; ++p0)
		for (auto p1{ 0 }; p1 < cloudsize; ++p1)
		{
			pairIndices[pair] = { (float)p0,(float)p1 };
			pairDistances[pair] = MagnitudeSquared(pointCloud[p1] - pointCloud[p0]);			
			if (pairDistances[pair] >= currentDistance)
			{
				index = pair;
				currentDistance = pairDistances[pair];
			}
			++pair;
		}
		
	auto p0{ pointCloud[pairIndices[index].x] };
	auto p1{ pointCloud[pairIndices[index].y] };
	result.PushBack(p0);
	result.PushBack(p1);
	delete[] pairDistances;
	delete[] pairIndices;	 

	//find point furthest from current simplex(line)
	//from john lloyd's java implementation
	MFvec3 u01{0};
	MFvec3 diff02{0};
	MFvec3 nrml{0};
	MFvec3 xprod{0};
	MFfloat maxSqr = 0;
	u01 = Normalize(result[1] - result[0]);	
	for(const auto& point : pointCloud)
	{
		diff02= point -  result[0];
		xprod = Cross(u01, diff02);
		MFfloat lenSqr = MagnitudeSquared(xprod);
		if (lenSqr > maxSqr &&
			point != result[0] &&  // paranoid
			point != result[1])
		{
			maxSqr = lenSqr;
			result[2] = point;
			nrml = xprod;
		}
	}
	// recompute nrml to make sure it is normal to u10 - otherwise could
	  // be errors in case vtx[2] is close to u10
	nrml=Normalize(nrml);
	MFvec3 res{};
	res = u01 * Dot(nrml, u01);// component of nrml along u01	
	nrml = Normalize(nrml - res);
	MFfloat maxDist = 0;
	MFfloat d0 = Dot(nrml, result[2]);
	for (const auto& point : pointCloud)
	{
		double dist = std::fabsf(Dot(point, nrml) - d0);		
		if (dist > maxDist &&
			point != result[0] &&  // paranoid
			point != result[1] &&
			point != result[2])
		{
			maxDist = dist;
			result[3] = point;
		}
	}
	reinterpret_cast<MFvec3&>(CONTRUCTION_NORMAL_OUT.x) = nrml;
	CONTRUCTION_NORMAL_OUT.w = d0;
	return result;
}

MFbool Manifest_Math::AssertConvexity(const HalfEdgeMesh& mesh)
{	
	//check each edge has one twin
	std::unordered_map<HullHalfEdge*, std::vector<HullHalfEdge*>> twinMap;
	//check each edge is shared between 2 faces
	std::unordered_map<HullHalfEdge*, std::unordered_set<HullFace*>> edgeMap;
	//check each vertex is shared between 3 at least face
	std::unordered_map<HullVertex*, std::unordered_set<HullFace*>> vertexMap;
	//check each face has at least 3 neighbors
	auto face{ mesh.faces };
	do
	{
		std::unordered_set<HullFace*> neighbors; 
		auto edge{ face->edge };
		do
		{			
			auto isFaceConvex(CheckFaceConvexity(edge->face, edge->twin->face, mesh.CONVEXITY_EPSILON));
			auto isTwinConvex(CheckFaceConvexity(edge->twin->face, edge->face, mesh.CONVEXITY_EPSILON));
			assert(isFaceConvex && isTwinConvex);
			twinMap[edge].emplace_back(edge->twin);
			edgeMap[edge].insert(face);
			edgeMap[edge->twin].insert(face);
			vertexMap[edge->tail].insert(face);
			neighbors.insert(edge->twin->face);
			edge = edge->next;			
		} while (edge != face->edge);
		assert(neighbors.size() >= 3);
		face = face->next;
	} while (face != mesh.faces);

	for (const auto& [edge, twin] : twinMap)
		assert(twin.size() == 1);
	for (const auto& [edge, faces] : edgeMap)
		assert(faces.size() == 2);
	for (const auto& [vertex, faces] : vertexMap)
		assert(faces.size() >= 3);	

	return true;
}