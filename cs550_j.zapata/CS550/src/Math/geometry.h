// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the geometry namespace
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#pragma once
#include <array>
#include <glm/glm.hpp>

namespace Geometry
{
	struct Sphere;
	struct Ray;
	struct Segment;
	struct Plane;

	enum class classification_t { inside, outside, overlapping };

	float IntersectionRaySphere(const Ray& _r, const Sphere& _s);
	glm::vec3 ClosestPointPlane(const glm::vec3& pt, const Plane& p);
	classification_t ClassifyPlanePoint(const Plane& p, const glm::vec3& pt, float plane_thickness = 0.0f);
	float IntersectionSegmentPlane(const Segment& r, const Plane& p);
	Segment ClosestSegmentSegment(const Segment& _s1, const Segment& _s2);
	float DistancePointPlane(const Plane& _p, const glm::vec3& _pt);
	bool SphereVsSphere(const Sphere& _s1, const Sphere& _s2);

	struct Triangle
	{
		const glm::vec3& operator[] (size_t i) const
		{
			assert(i >= 0 && i < 3);
			return pts[i];
		}
		glm::vec3& operator[] (size_t i) 
		{
			assert(i >= 0 && i < 3);
			return pts[i];
		}
		std::array<glm::vec3, 3> pts;
	};

	struct Ray
	{
		glm::vec3 origin;
		glm::vec3 dir;
	};

	struct Point
	{
		glm::vec3 pos;
		float size;
	};

	struct Segment
	{
		glm::vec3 p0;
		glm::vec3 p1;
		float width;
	};

	struct Sphere
	{
		glm::vec3 center;
		float radius;
	};

	struct AABB
	{
		glm::vec3 min;
		glm::vec3 max;
	};

	struct Plane
	{
		glm::vec3 normal;
		glm::vec3 pt;
		float     dot_result;

		Plane() = default;

		Plane(glm::vec3 point, glm::vec3 n)
		{
			normal = glm::normalize(n);
			pt = point;
			dot_result = glm::dot(point, n);
		}
		glm::vec3 GetPoint() const { return normal * dot_result; }
	};


}