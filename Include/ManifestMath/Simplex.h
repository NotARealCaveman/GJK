#pragma once
#include <array>

#include <ManifestMath/Point3.h>


namespace Manifest_Math
{
	template<typename T>
	class Simplex_T
	{
	public:
		Simplex_T() = default;
		Simplex_T(std::initializer_list<T> _points)
		{
			*this = _points;
		}
		T& operator[](const MFint32& index) { return points[index]; };
		const T& operator[](const MFint32& index) const { return points[index]; };
		Simplex_T& operator=(std::initializer_list<T> _points)
		{
			size = _points.size();			
			std::transform(_points.begin(), _points.end(), points.begin(), [&](const T& point) {return point; });

			return *this;
		}				
		void PushFront(const T& point)
		{
			points = { point, points[0], points[1], points[2] };
			size = std::min(size + 1, 4);
		}	
		void PushBack(const T& point)
		{			
			if (size < 3)
			{
				points[size] = point;
				size = std::min(size + 1, 4);
			}
			else
			{
				points[0] = points[1];
				points[1] = points[2];
				points[2] = points[3];
				points[3] = point;	
			}			

		}
		inline const MFu8 Size() const { return size; }
		const auto begin() const { return points.begin(); };
		const auto end() const { return points.begin() + size; };
	private:
		std::array<T,4> points{ 0 };
		MFu8 size{ 0 };		
	}; 
}