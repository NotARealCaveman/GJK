#pragma once

#include <cstdint>
#include <utility>
#include <type_traits>

namespace Manifest_Utility
{	
	constexpr uint64_t pow2(const uint64_t base)
	{
		return uint64_t{ 1 } << base;
	}
	constexpr uint16_t ipow2(const uint64_t pow2Base)
	{
		return pow2Base>>1;
	}
	constexpr uint8_t root2(uint64_t pow2Base)
	{
		uint8_t result{ 0 };
		while (pow2Base = pow2Base >> 1)
			++result;
		return result;
	}	
	//RETURNS THE INTEGRAL VALUE OF AN OBJECT T
	template<typename T>
	inline constexpr typename std::underlying_type<T>::type UnderlyingType(const T& t)
	{
		return static_cast<typename std::underlying_type<T>::type>(t);
	};
	//DETERMINES RETURN TYPE BASED ON FUNCTION
	template<typename Function, typename... Params>
	using ReturnType = decltype(std::declval<Function>()(std::declval<Params>()...));
	//FORWARDS FUNCTION W/ PARAMS
	template<typename Function, typename... Params>
	decltype(auto) ForwardFunction(Function&& function, Params&&... params)
	{
		return std::invoke(std::forward<Function>(function), std::forward<Params>(params)...);
	};

	//RETURNS THE BITWISE & OF TWO INTEGRAL TYPES AS THE ORIGINAL OBJECT TYPE T
	template<typename T>
	inline constexpr T operator&(const T& t0, const T& t1)
	{
		return static_cast<T>(UnderlyingType(t0) & UnderlyingType(t1));
	}
	//RETURNS THE BITWISE | OF TWO INTEGRAL TYPES AS THE ORIGINAL OBJECT TYPE T
	template<typename T>
	inline constexpr T operator|(const T& t0, const T& t1)
	{
		return static_cast<T>(UnderlyingType(t0) | UnderlyingType(t1));
	}
	//RETURNS THE BITWISE |= OF TWO INTEGRAL TYPES AS THE ORIGINAL OBJECT TYPE T
	template<typename T>
	inline constexpr T operator|=(const T& t0, const T& t1)
	{
		return t0 | t1;
	}	
}