#pragma once
#include <iostream>
#include <iomanip>

#include "Typenames.h"

namespace Manifest_Math
{

	constexpr MFfloat Pi = 3.1415927f;
	constexpr MFfloat Pi2 = Pi * 2.0f;
	constexpr MFfloat EpsilonMax = 1.0f - std::numeric_limits<MFfloat>::epsilon();

	inline constexpr MFfloat CRadians(const MFfloat& degrees);
	inline constexpr MFfloat CDegrees(const MFfloat& radians);
	inline const MFfloat Radians(const MFfloat& degrees);
	inline const MFfloat Degrees(const MFfloat& radians);
//calcluates Cotanget as 1.0f/tangent(t)
	inline const MFfloat Cot(const MFfloat& t);
	inline const MFfloat& Min(const MFfloat& a, const MFfloat& b);
//returns a if greater than clamping value
	inline const MFfloat ClampMin(const MFfloat& a, const MFfloat& clamp);	
	inline const MFfloat& Max(const MFfloat& a, const MFfloat& b);
//returns a if less than clamping vlaue
	inline const MFfloat ClampMax(const MFfloat& a, const MFfloat& clamp);
//clamps value between desired range
	inline const MFfloat ClampRange(const MFfloat& lower, const MFfloat& a, const MFfloat& upper);
	inline MFfloat Clamp_EPSILON(const MFfloat f, const MFfloat EPSILON, const MFfloat clampValue);
//calculates an epsilon value between a relative and absolute tolerance
	inline MFfloat Epsilon(const MFfloat relativeTolerance, const MFfloat relativeValue, const MFfloat absoluteTolerance);
}