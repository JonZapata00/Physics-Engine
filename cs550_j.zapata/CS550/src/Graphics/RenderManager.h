// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the Render Manager class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#pragma once
#include "../Utilities/pch.hpp"
#include "GL/glew.h"
#include "../Utilities/Singleton.h"
#include "../Utilities/jsonWrapper.h"
#include "Shader.h"
#include "Window.h"
#include "Camera.h"
#include "Light.h"
#include "Material.h"

class Mesh;

struct CubeMap
{
	std::string dir{ "Resources/skybox" };
	GLuint cubemapFBO[6]{};
	GLuint tex{};
	GLuint vao{};
};

enum class PolygonMode_t { Solid, Wireframe, PointCloud };
enum class DrawMode_t {Triangles, Points, Lines};
enum class Shader_Type { LIGHT, DEBUG, CUBEMAP };

class RenderManager
{
	MAKE_SINGLETON(RenderManager)
public:
	void Initialize(int _width = 1280, int _height = 720);
	void SetShader(Shader_Type type);
	void Shutdown();
	void StartFrame();
	void EndFrame() const;

	void RenderAll();
	void RenderMesh(const Mesh& _mesh, const glm::mat4& m2w, const glm::vec4& _color, PolygonMode_t _mode, DrawMode_t _drawMode = DrawMode_t::Triangles);

	~RenderManager();

	const Camera& GetCamera() const { return camera; }
	Camera& GetCamera() { return camera; }
	const Window& GetWindow() const { return window; }

	const std::vector<Mesh*> GetMeshes() const { return mMeshes; }
	std::vector<Mesh*>& GetMeshes()  { return mMeshes; }

	void AddMesh();

	void SaveScene(json& _j) const;
	void LoadScene(json _j);
private:
	void RenderEditingBody();
	void InitializeOpenGL() const;
	void UploadLightingUniforms();
	void CreateCubeMap();
	void RenderCubeMap();

	std::vector<Mesh*> mMeshes{};

	Light* light = nullptr; //just one directional light
	Material* mat = nullptr; //one material for everyone
	CubeMap* cubeMap;

	std::unordered_map<Shader_Type, Shader*> shaders; 
	Shader_Type currentShader = Shader_Type::LIGHT;

	Window window;
	Camera camera;
};

#define GfxManager  RenderManager::Instance()