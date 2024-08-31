#include "Barycentric.h"

using namespace Manifest_Math;

MFpoint3 Manifest_Math::Barycentric(const MFpoint3& q, const MFpoint3& a, MFfloat& u)
{
	u = 1.0f;
	return a;
}

MFpoint3 Manifest_Math::Barycentric(const MFpoint3& q, const MFpoint3& a, const MFpoint3& b, MFfloat& u, MFfloat& v)
{
	//project q onto simplex
	const MFvec3 simplex{ b - a };
	const MFfloat iMag{ 1.0f / Magnitude(simplex) };
	const MFvec3 AQ{ q - a };
	const MFvec3 QB{ b - a };
	v = Dot(AQ, simplex * iMag) * iMag;
	u = 1 - v;
	assert(u + v > 0.99 && u + v < 1.1);
	//check outside voronoi regions
	if (u <= 0.0f)
	{
		u = 0.0f;
		return Barycentric(q, b, v);
	}
	else if (v <= 0.0f)
	{
		v = 0.0f;
		return Barycentric(q, a, u);
	}

	//calcualte inside region from barycentric weights
	return a * u + b * v;
}

MFpoint3 Manifest_Math::Barycentric(const MFpoint3& q, const MFpoint3& a, const MFpoint3& b, const MFpoint3& c, MFfloat& u, MFfloat& v, MFfloat& w)
{
	const auto CalcualteSignedTriangleArea = [](const MFpoint3& p, const MFpoint3& v0, const MFpoint3& v1, const MFvec3& normalABC)->MFfloat
		{
			const MFvec3 e0{ v0 - p };
			const MFvec3 e1{ v1 - p };

			return Dot(Cross(e0, e1), normalABC);
		};

	MFfloat uAB, vAB, uBC, vBC, uCA, vCA;

	Barycentric(q, a, b, uAB, vAB);
	Barycentric(q, b, c, uBC, vBC);
	Barycentric(q, c, a, uCA, vCA);

	u = v = w = 0;

	//test vertex regions first
	//closest to A
	if (uCA <= 0.0f && vAB <= 0.0f)
		return Barycentric(q, a, u);
	//closest to B
	if (uAB <= 0.0f && vBC <= 0.0f)
		return Barycentric(q, b, v);
	//closest to C
	if (uBC <= 0.0f && vCA <= 0.0f)
		return Barycentric(q, c, w);
	
	const MFvec3 n{ CalculateSurfaceNormal(a,b,c) };
	const MFfloat areaABC{ CalcualteSignedTriangleArea(a,b,c,n) };
	const MFfloat iAreaABC{ 1.0f / areaABC };
	const MFfloat areaQBC{ CalcualteSignedTriangleArea(q,b,c,n) };
	const MFfloat areaQCA{ CalcualteSignedTriangleArea(q,c,a,n) };
	const MFfloat areaQAB{ CalcualteSignedTriangleArea(q,a,b,n) };

	const MFfloat uABC{ areaQBC * iAreaABC };
	const MFfloat vABC{ areaQCA* iAreaABC };
	const MFfloat wABC{ areaQAB * iAreaABC };

	//test edge regions second
	//closest to AB
	if (uAB > 0.0f && vAB > 0.0f && wABC <= 0.0f)
	{
		u = 1 - vAB;
		v = vAB;
		w = 0.0f;

		return a * u + b * v;
	} 
	//closest to BC
	if (uBC > 0.0f && vBC > 0.0f && uABC <= 0.0f)
	{
		v = 1 - vBC;
		w = vBC;
		u = 0.0f;

		return b * v + c * w;
	}
	//closest to CA
	if (uCA > 0.0f && vCA > 0.0f && vABC <= 0.0f)
	{
		u = 1 - uCA;
		w = uCA;
		v = 0.0f;

		return c * w + a * u;
	}

	//test triangle region third	
	assert(uABC > 0.0f);
	assert(vABC > 0.0f);
	assert(wABC > 0.0f);
	assert(uABC + vABC + wABC > 0.99 && uABC + vABC + wABC < 1.1);

	u = 1 - wABC - vABC;
	v = vABC;
	w = wABC;

	return a * u + b * v + c * w;
}

MFpoint3 Manifest_Math::Barycentric(const MFpoint3& q, const MFpoint3& a, const MFpoint3& b, const MFpoint3& c, const MFpoint3& d, MFfloat& u, MFfloat& v, MFfloat& w, MFfloat& t)
{
	u = v = w = t = 0;
	//check if tetrahedron is degenerate - remove point d(oldest)
	if (std::fabsf(Dot(d - a, Cross(b - a, c - a))) <= 1e-5)
		return Barycentric(q, a, b, c, u, v, w);

	//test vertex regions first	
	const MFvec3 AB{ b - a };
	const MFvec3 AC{ c - a };
	const MFvec3 AD{ d - a };
	const MFvec3 AQ{ q - a };
	const MFfloat AQAB{ Dot(AQ,AB) };
	const MFfloat AQAC{ Dot(AQ,AC) };
	const MFfloat AQAD{ Dot(AQ,AD) };

	const MFbool qProjectsOutsideAB{ AQAB <= 0.0f };
	const MFbool qProjectsOutsideAC{ AQAC <= 0.0f };
	const MFbool qProjectsOutsideAD{ AQAD <= 0.0f };

	if (static_cast<MFbool>(qProjectsOutsideAB & qProjectsOutsideAC & qProjectsOutsideAD))
		return Barycentric(q, a, u);

	//closest to B
	const MFvec3 BA{ a - b };
	const MFvec3 BC{ c - b };
	const MFvec3 BD{ d - b };
	const MFvec3 BQ{ q - b };
	const MFfloat BQBA{ Dot(BQ,BA) };
	const MFfloat BQBC{ Dot(BQ,BC) };
	const MFfloat BQBD{ Dot(BQ,BD) };

	const MFbool qProjectsOutsideBA{ BQBA <= 0.0f };
	const MFbool qProjectsOutsideBC{ BQBC <= 0.0f };
	const MFbool qProjectsOutsideBD{ BQBD <= 0.0f };

	if (static_cast<MFbool>(qProjectsOutsideBA & qProjectsOutsideBC & qProjectsOutsideBD))
		return Barycentric(q, b, v);

	//closest to C
	const MFvec3 CA{ a - c };
	const MFvec3 CB{ b - c };
	const MFvec3 CD{ d - c };
	const MFvec3 CQ{ q - c };
	const MFfloat CQCA{ Dot(CQ,CA) };
	const MFfloat CQCB{ Dot(CQ,CB) };
	const MFfloat CQCD{ Dot(CQ,CD) };

	const MFbool qProjectsOutsideCA{ CQCA <= 0.0f };
	const MFbool qProjectsOutsideCB{ CQCB <= 0.0f };
	const MFbool qProjectsOutsideCD{ CQCD <= 0.0f };

	if (static_cast<MFbool>(qProjectsOutsideCA & qProjectsOutsideCB & qProjectsOutsideCD))
		return Barycentric(q, c, w);

	//closest to D
	const MFvec3 DA{ a - d };
	const MFvec3 DB{ b - d };
	const MFvec3 DC{ c - d };
	const MFvec3 DQ{ q - d };
	const MFfloat DQDA{ Dot(DQ,DA) };
	const MFfloat DQDB{ Dot(DQ,DB) };
	const MFfloat DQDC{ Dot(DQ,DC) };

	const MFbool qProjectsOutsideDA{ DQDA <= 0.0f };
	const MFbool qProjectsOutsideDB{ DQDB <= 0.0f };
	const MFbool qProjectsOutsideDC{ DQDC <= 0.0f };

	if (static_cast<MFbool>(qProjectsOutsideDA & qProjectsOutsideDB & qProjectsOutsideDC))
		return Barycentric(q, d, t);

	//test edge regions second
	//AB(BA)
	//surface normals
	const MFvec3 nABC{ Cross(AB,AC) };
	const MFvec3 nADB{ Cross(AD,AB) };
	//edge normals
	const MFvec3 nAB{ Cross(AB,nABC) };
	const MFvec3 nBA{ Cross(nADB,AB) };
	//projections with query point
	const MFbool qProjectsInsideAB{ AQAB >= 0.0f };
	const MFbool qProjectsInsideBA{ BQBA >= 0.0f };
	const MFfloat AQnAB{ Dot(AQ,nAB) };
	const MFfloat AQnBA{ Dot(AQ,nBA) };
	const MFbool qProjectsTowardsnAB{ AQnAB >= 0.0f };
	const MFbool qProjectsTowardsnBA{ AQnBA >= 0.0f };
	//evaluation
	if (static_cast<MFbool>(qProjectsInsideAB & qProjectsInsideBA & qProjectsTowardsnAB, qProjectsTowardsnBA))
		return Barycentric(q, a, b, u, v);

	//BC(CB)
	//surface normals
	const MFvec3 nDCB{ Cross(DC,DB) };
	//edge normals
	const MFvec3 nBC{ Cross(BC,nABC) };
	const MFvec3 nCB{ Cross(nDCB,BC) };
	//projections with query point
	const MFbool qProjectsInsideBC{ BQBC >= 0.0f };
	const MFbool qProjectsInsideCB{ CQCB >= 0.0f };
	const MFfloat BQnBC{ Dot(BQ,nBC) };
	const MFfloat BQnCB{ Dot(BQ,nCB) };
	const MFbool qProjectsTowardsnBC{ BQnBC >= 0.0f };
	const MFbool qProjectsTowardsnCB{ BQnCB >= 0.0f };
	//evaluation
	if (static_cast<MFbool>(qProjectsInsideBC & qProjectsInsideCB & qProjectsTowardsnBC & qProjectsTowardsnCB))
		return Barycentric(q, b, c, v, w);

	//CA(AC)
	//surface normals
	const MFvec3 nACD{ Cross(AC,AD) };
	//edge normals
	const MFvec3 nCA{ Cross(CA,nABC) };
	const MFvec3 nAC{ Cross(nACD,CA) };
	//projections with query points
	const MFbool qProjectsInsideCA{ CQCA >= 0.0f };
	const MFbool qProjectsInsideAC{ AQAC >= 0.0f };
	const MFfloat CQnCA{ Dot(CQ,nCA) };
	const MFfloat CQnAC{ Dot(CQ,nAC) };
	const MFbool qProjectsTowardsnCA{ CQnCA >= 0.0f };
	const MFbool qProjectsTowardsnAC{ CQnAC >= 0.0f };
	//evaluation
	if (static_cast<MFbool>(qProjectsInsideCA & qProjectsInsideAC & qProjectsTowardsnCA & qProjectsTowardsnAC))
		return Barycentric(q, c, a, w, u);

	//AD(DA)
	//edge normals
	const MFvec3 nAD{ Cross(AD,nADB) };
	const MFvec3 nDA{ Cross(nACD,AD) };
	//projections with query point
	const MFbool qProjectsInsideAD{ AQAD >= 0.0f };
	const MFbool qProjectsInsideDA{ DQDA >= 0.0f };
	const MFfloat AQnAD{ Dot(AQ,nAD) };
	const MFfloat AQnDA{ Dot(AQ,nDA) };
	const MFbool qProjectsTowardsnAD{ AQnAD >= 0.0f };
	const MFbool qProjectsTowardsnDA{ AQnDA >= 0.0f };
	//evaluation
	if (static_cast<MFbool>(qProjectsInsideAD & qProjectsInsideDA & qProjectsTowardsnAD & qProjectsTowardsnDA))
		return Barycentric(q, a, d, u, t);

	//DB(BD)
	//edge normals
	const MFvec3 nDB{ Cross(DB,nADB) };
	const MFvec3 nBD{ Cross(nDCB,DB) };
	//projections with query point
	const MFbool qProjectsInsideDB{ DQDB >= 0.0f };
	const MFbool qProjectsInsideBD{ BQBD >= 0.0f };
	const MFfloat DQnDB{ Dot(DQ,nDB) };
	const MFfloat DQnBD{ Dot(DQ,nBD) };
	const MFbool qProjectsTowardsnDB{ DQnDB >= 0.0f };
	const MFbool qProjectsTowardsnBD{ DQnBD >= 0.0f };
	//evaluation
	if (static_cast<MFbool>(qProjectsInsideDB & qProjectsInsideBD & qProjectsTowardsnDB & qProjectsTowardsnBD))
		return Barycentric(q, d, b, t, v);

	//CD(DC)
	//edge normals
	const MFvec3 nCD{ Cross(CD,nACD) };
	const MFvec3 nDC{ Cross(nDCB,CD) };
	//projections with query point
	const MFbool qProjectsInsideCD{ CQCD >= 0.0f };
	const MFbool qProjectsInsideDC{ DQDC >= 0.0f };
	const MFfloat CQnCD{ Dot(CQ,nCD) };
	const MFfloat CQnDC{ Dot(CQ,nDC) };
	const MFbool qProjectsTowardsnCD{ CQnCD >= 0.0f };
	const MFbool qProjectsTowardsnDC{ CQnDC >= 0.0f };
	//evaluation
	if (static_cast<MFbool>(qProjectsInsideCD & qProjectsInsideDC & qProjectsTowardsnCD & qProjectsTowardsnDC))
		return Barycentric(q, c, d, w, t);

	//test face regions third
	//ABC(D)
	const MFfloat signedVolumeABCD{ Dot(AD,nABC) };
	const MFfloat signedVolumeABCQ{ Dot(AQ,nABC) };
	const MFbool qIsSeparatedFromD{ (signedVolumeABCD * signedVolumeABCQ) < 0.0f };
	if (qIsSeparatedFromD)
		return Barycentric(q, a, b, c, u, v, w);
	//ADB(C)
	const MFfloat signedVolumeADBC{ Dot(AC,nADB) };
	const MFfloat signedVolumeADBQ{ Dot(AQ,nADB) };
	const MFbool qIsSeparatedFromC{ (signedVolumeADBC * signedVolumeADBQ) < 0.0f };
	if (qIsSeparatedFromC)
		return Barycentric(q, a, d, b, u, t, v);
	//ACD(B)
	const MFfloat signedVolumeACDB{ Dot(AB,nACD) };
	const MFfloat signedVolumeACDQ{ Dot(AQ,nACD) };
	const MFbool qIsSeparatedFromB{ (signedVolumeACDB * signedVolumeACDQ) < 0.0f };
	if (qIsSeparatedFromB)
		return Barycentric(q, a, c, d, u, w,t);
	//DCB(A)
	const MFfloat signedVolumeDCBA{ Dot(DA,nDCB) };
	const MFfloat signedVolumeDCBQ{ Dot(DQ,nDCB) };
	const MFbool qIsSeparatedFromA{ (signedVolumeDCBA * signedVolumeDCBQ) < 0.0f };
	if (qIsSeparatedFromA)
		return Barycentric(q, d, c, b, t, w, v);

	//q lies within simplex
	return q;
}