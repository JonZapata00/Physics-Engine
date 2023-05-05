// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the geometry namespace
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include "../Utilities/pch.hpp"
#include "math.h"

namespace Geometry
{
    /**
      * Computes the time of intersection between a ray and a sphere
      * @param _r - ray
      * @param _s - sphere
      * @return time of intersection (-1 if fail)
    */
    float IntersectionRaySphere(const Ray& _r, const Sphere& _s)
    {
        //take the vector joining the center of the sphere and the origin of the ray
        glm::vec3 sphereToRay = _r.origin - _s.center;
        //these are the variables used for the quadrtic formula
        float a = glm::dot(_r.dir, _r.dir);
        float b = 2.0f * glm::dot(_r.dir, sphereToRay);
        float c = glm::dot(sphereToRay, sphereToRay) - _s.radius * _s.radius;
        float discriminant = b * b - 4.0f * a * c;
        float denominator = 2.0f * a;
        //check first whether we have a negative number in the sqrt or if the denominator is 0,
        //in which case there is no solution and therefore no intersection
        if (discriminant < cEpsilon || (denominator <= cEpsilon && denominator >= -cEpsilon)) return -1.0f;

        //the quadratic formula gives us two solutions
        float t1 = (-b - sqrtf(discriminant)) / denominator;
        float t2 = (-b + sqrtf(discriminant)) / denominator;

        //if both are negative, there is no intersection
        if (t1 < cEpsilon && t2 < cEpsilon) return -1.0f;
        //we only take the positive
        else if (t1 < cEpsilon && t2 > cEpsilon) return t2;
        else if (t1 > cEpsilon && t2 < cEpsilon) return t1;
        //if both are positive, we return the minimum time.
        return t1;
    }

    glm::vec3 ClosestPointPlane(const glm::vec3& pt, const Plane& p)
    {
        glm::vec3 normal = glm::normalize(p.normal);
        //project the point and return
        glm::vec3 ptProjected = pt - glm::dot(pt - p.pt, normal) * normal;
        return ptProjected;
    }
    classification_t ClassifyPlanePoint(const Plane& p, const glm::vec3& pt, float plane_thickness)
    {
        //take the closest point
        glm::vec3 normal = glm::normalize(p.normal);
        glm::vec3 closestPoint = ClosestPointPlane(pt, p);
        //then check if the distance (squared) is less than the thickness (squared)
        //meaning no matter what it'll be outside the plane
        float distanceSq = glm::length2(closestPoint - pt);
        if (distanceSq <= (plane_thickness * plane_thickness)) return classification_t::overlapping;

        //otherwise, we check by substituting in the plane equation and interpreting results
        float dot = glm::dot(pt - p.pt, normal);
        if (dot < cEpsilon && dot > -cEpsilon) return classification_t::overlapping;
        else if (dot <= cEpsilon) return classification_t::inside;

        return classification_t::outside;
    }
    float IntersectionSegmentPlane(const Segment& r, const Plane& p)
    {
        //this is to ensure the normal is normalized
        glm::vec3 normal = glm::normalize(p.normal);

        //we take the dot product to check that ray and plane are not parallel
        //(normal and direction of ray are not perpendicular)
        float dot = glm::dot(normal, r.p1 - r.p0);

        if (dot > cEpsilon || dot < -cEpsilon)
        {
            //if they aren't, we compute the d of the plane's equation
            //to then calculate the time of intersection as seen in class
            float d = glm::dot(normal, p.pt);
            float t = (d - glm::dot(normal, r.p0)) / dot;
            //this is a way of checking that the time is clamped to [0, inf), since it is a ray
            if (t >= cEpsilon) return std::clamp(t, 0.0f, 1.0f);
        }
        else if (ClassifyPlanePoint(p, r.p0) == classification_t::overlapping && ClassifyPlanePoint(p, r.p1) == classification_t::overlapping)
            return 0.0f;

        return -1.0f;
    }
    Segment ClosestSegmentSegment(const Segment& _s1, const Segment& _s2)
    {
        //take direction vectors of both segments
        glm::vec3 dirS1 = _s1.p1 - _s1.p0;
        glm::vec3 dirS2 = _s2.p1 - _s2.p0;
        //take vector joining them
        glm::vec3 joiningSegments = _s1.p0 - _s2.p0;

        //these are the necessary dot products and operations to calculate the 
        //parameters that will give us the intersection points in each segment
        float aDot = glm::dot(dirS1, dirS1);
        float bDot = glm::dot(dirS1, dirS2);
        float cDot = glm::dot(dirS2, dirS2);
        float dDot = glm::dot(dirS1, joiningSegments);
        float eDot = glm::dot(dirS2, joiningSegments);
        float denominator = bDot * bDot - aDot * cDot;
        float s, t;
        //sanity check to see if the denominator is 0 (segments are parallel)
        if (denominator <= cEpsilon && denominator >= -cEpsilon) t = 0.0f;
        else
        {
            //computing t according to the equations, then clamping it
            t = (bDot * dDot - aDot * eDot) / denominator;
            t = std::clamp(t, 0.0f, 1.0f);
        }

        //checks to avoid dividing by 0
        if (aDot > cEpsilon || aDot < -cEpsilon)
        {
            //computing s, then clamping
            s = (-dDot + bDot * t) / aDot;
            s = std::clamp(s, 0.0f, 1.0f);
        }
        else s = 0.0f;

        if (cDot > cEpsilon || cDot < -cEpsilon)
        {
            //recomputing t and reclamping it
            t = (eDot + s * bDot) / cDot;
            t = std::clamp(t, 0.0f, 1.0f);
        }
        else t = 0.0f;

        //returning the segment joining both segments with our calculated parameters
        return { _s1.p0 + s * dirS1, _s2.p0 + t * dirS2 };
    }
    float DistancePointPlane(const Plane& _p, const glm::vec3& _pt)
    {
        auto planePt = _p.GetPoint();
        auto vec = _pt - _p.pt;
        return glm::dot(vec, _p.normal);
    }
    bool SphereVsSphere(const Sphere& _s1, const Sphere& _s2)
    {
        float sumRadiiSq = (_s1.radius + _s2.radius) * (_s1.radius + _s2.radius);
        float distSq = glm::length2(_s1.center - _s2.center);
        
        return distSq <= sumRadiiSq;
    }
}