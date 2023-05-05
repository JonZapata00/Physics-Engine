// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the DebugRenderer class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#pragma once
#include "../Utilities/pch.hpp"
#include "../Math/math.h"
#include "../Utilities/Singleton.h"
#include "RenderManager.h"
#include "Mesh.h"

class DebugRenderer
{
	MAKE_SINGLETON(DebugRenderer)
private:
	void CleanUp();

	struct Resources
	{
		std::shared_ptr<Mesh> point;
		std::shared_ptr<Mesh> segment;
		std::shared_ptr<Mesh> cube;
		std::shared_ptr<Mesh> sphere;
	};

	Resources mResources;

public:
	void Initialize();
	void DebugDrawPoint(const Geometry::Point& _pt, const glm::vec4& _color) const;
	void DebugDrawLine(const Geometry::Segment& _s, const glm::vec4& _color) const;
	void DebugDrawCube(const Geometry::AABB& _cube, const glm::vec4& _color) const;
	void DebugDrawSphere(const Geometry::Sphere& _sphere, const glm::vec4& _color) const;
	void DebugDrawMesh(const MeshData& _mesh, const glm::mat4 _m2w, const glm::vec4& _color, PolygonMode_t _polyMode, DrawMode_t _drawMode) const;

	~DebugRenderer();
	Resources GetResources() const { return mResources; }


};

#define DebugRenderMgr DebugRenderer::Instance()