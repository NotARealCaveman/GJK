#include "Core.h"

using namespace Manifest_Math;

extern constexpr MFfloat Manifest_Math::CRadians(const MFfloat& degrees) { return (Pi / 180.0f) * degrees; };
extern constexpr MFfloat Manifest_Math::CDegrees(const MFfloat& radians) { return (180.0f / Pi) * radians; };
extern const MFfloat Manifest_Math::Radians(const MFfloat& degrees) { return (Pi / 180.0f) * degrees; };
extern const MFfloat Manifest_Math::Degrees(const MFfloat& radians) { return (180.0f / Pi) * radians; };
//calcluates Cotanget as 1.0f/tangent(t)
extern const MFfloat Manifest_Math::Cot(const MFfloat& t) { return 1.0f / tanf(t); };
extern const MFfloat& Manifest_Math::Min(const MFfloat& a, const MFfloat& b) { return a < b ? a : b; };
//returns a if greater than clamping value
extern const MFfloat Manifest_Math::ClampMin(const MFfloat& a, const MFfloat& clamp) { return a < clamp ? clamp : a; };
extern const MFfloat& Manifest_Math::Max(const MFfloat& a, const MFfloat& b) { return a > b ? a : b; };
//returns a if less than clamping vlaue
extern const MFfloat Manifest_Math::ClampMax(const MFfloat& a, const MFfloat& clamp) { return a > clamp ? clamp : a; };
//clamps value between desired range
extern const MFfloat Manifest_Math::ClampRange(const MFfloat& lower, const MFfloat& a, const MFfloat& upper)
{
	return Min(Max(lower, a), upper);
}
extern MFfloat Manifest_Math::Clamp_EPSILON(const MFfloat f, const MFfloat EPSILON, const MFfloat clampValue)
{
	return std::fabsf(f) < EPSILON ? clampValue : f;
}
//calculates an epsilon value between a relative and absolute tolerance
extern MFfloat Manifest_Math::Epsilon(const MFfloat relativeTolerance, const MFfloat relativeValue,const MFfloat absoluteTolerance)
{
	return relativeTolerance * relativeValue + absoluteTolerance;
}