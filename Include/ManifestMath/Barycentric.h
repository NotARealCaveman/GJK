#pragma once
#include "Triangle.h"
#include <ManifestUtility/TypeAssist.h>

using namespace Manifest_Utility;

namespace Manifest_Math
{	
	//return closest point q to point a, stores barycentric value u
	MFpoint3 Barycentric(const MFpoint3& q, const MFpoint3& a,  MFfloat& u);
	//return closest point p from q to line AB, stores barycentric values u,v
	MFpoint3 Barycentric(const MFpoint3& q, const MFpoint3& a, const MFpoint3& b, MFfloat& u, MFfloat& v);
	//return closest point p from q to triangle ABC, stores barycentric values u,v,w
	MFpoint3 Barycentric(const MFpoint3& q, const MFpoint3& a, const MFpoint3& b, const MFpoint3& c, MFfloat& u, MFfloat& v, MFfloat& w);
	//stores barycentric values u,v,w,t from q to tetrahedron ABCD, 
	MFpoint3 Barycentric(const MFpoint3& q, const MFpoint3& a, const MFpoint3& b, const MFpoint3& c, const MFpoint3& d, MFfloat& u, MFfloat& v, MFfloat& w, MFfloat& t);
}