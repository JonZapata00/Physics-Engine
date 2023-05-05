// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the DebugRenderer class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include "DebugRenderer.h"

/**
 * loads necessary resources for the debug renderer
*/
void DebugRenderer::Initialize()
{
	mResources.point = std::make_shared<Mesh>(std::vector<glm::vec3>{ {0.0f, 0.0f, 0.0f} }, std::vector<unsigned>{0});
	mResources.segment = std::make_shared<Mesh>(std::vector<glm::vec3>{ {0.0f, 0.0f, 0.0f}, { 1.0f, 1.0f, 1.0f } }, std::vector<unsigned>{0, 1});
	mResources.cube = std::make_shared<Mesh>("Resources/meshes/cube.obj");
	mResources.sphere = std::make_shared<Mesh>("Resources/meshes/sphere.obj");
}

/**
 * Debug draws a point
 * @param _pt - point
 * @param _color - color
*/
void DebugRenderer::DebugDrawPoint(const Geometry::Point& _pt, const glm::vec4& _color) const
{
	GfxManager.SetShader(Shader_Type::DEBUG);
	glPointSize(_pt.size);
	GfxManager.RenderMesh(*mResources.point, glm::translate(_pt.pos), _color, PolygonMode_t::PointCloud, DrawMode_t::Points);
	GfxManager.SetShader(Shader_Type::LIGHT);
}

/**
 * Debug draws a line
 * @param _s - line
 * @param _color - color
*/
void DebugRenderer::DebugDrawLine(const Geometry::Segment& _s, const glm::vec4& _color) const
{
	auto  T = glm::translate(_s.p0);
	auto  S = glm::scale(_s.p1 - _s.p0);
	auto  m2w = T * S;

	GfxManager.SetShader(Shader_Type::DEBUG);
	glLineWidth(_s.width);
	GfxManager.RenderMesh(*mResources.segment, m2w, _color, PolygonMode_t::Wireframe, DrawMode_t::Lines);
	GfxManager.SetShader(Shader_Type::LIGHT);
}

/**
 * Debug draws a cube
 * @param _cube - cube
 * @param _color - color
*/
void DebugRenderer::DebugDrawCube(const Geometry::AABB& _cube, const glm::vec4& _color) const
{
	auto  T = glm::translate((_cube.min + _cube.max) / 2.0f);
	auto  S = glm::scale(glm::abs(_cube.min - _cube.max));
	auto  m2w = T * S;
	GfxManager.SetShader(Shader_Type::DEBUG);
	GfxManager.RenderMesh(*mResources.cube, m2w, _color, PolygonMode_t::Wireframe);
	GfxManager.SetShader(Shader_Type::LIGHT);
}

/**
 * Debug draws a sphere
 * @param _sphere - sphere
 * @param _color - color
*/
void DebugRenderer::DebugDrawSphere(const Geometry::Sphere& _sphere, const glm::vec4& _color) const
{
	auto  T = glm::translate(_sphere.center);
	auto  S = glm::scale(glm::vec3(_sphere.radius));
	auto  m2w = T * S;
	GfxManager.SetShader(Shader_Type::DEBUG);
	GfxManager.RenderMesh(*mResources.sphere, m2w, _color, PolygonMode_t::Wireframe);
	GfxManager.SetShader(Shader_Type::LIGHT);
}

/**
 * Debug draws a given mesh
 * @param _mesh - _mesh data
 * @param _m2w - world transform
 * @param _color - color
 * @param _polyMode - polygon mode to render
 * @param _drawMode - drawing mode to render
*/
void DebugRenderer::DebugDrawMesh(const MeshData& _mesh, const glm::mat4 _m2w, const glm::vec4& _color, PolygonMode_t _polyMode, DrawMode_t _drawMode) const
{
	Mesh m;
	m.GetMeshData() = _mesh;
	m.CreateMesh();
	GfxManager.SetShader(Shader_Type::DEBUG);
	GfxManager.RenderMesh(m, _m2w, _color, _polyMode, _drawMode);
	m.DestroyMesh();
}

/**
* Destructor. Frees memory
*/
DebugRenderer::~DebugRenderer()
{
	CleanUp();
}

/**
* Frees the allocated memory for debug meshes
*/
void DebugRenderer::CleanUp()
{
	mResources.point.reset();
	mResources.segment.reset();
	mResources.cube.reset();
	mResources.sphere.reset();
}
