// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Render Manager class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include <filesystem>
#include "../Utilities/pch.hpp"
#include <glm/gtc/random.hpp>
#include "../ImGui/imgui.h"
#include "../Physics/PhysicsManager.h"
#include "../Utilities/ImGuiManager.h"
#include "../Utilities/Editor.h"
#define STB_IMAGE_IMPLEMENTATION
#include "../Utilities/stb_image.h"
#include "DebugRenderer.h"
#include "RenderManager.h"

/**
 * Creates window, Initializes OpenGL, creates shaders and initializes ImGui context
 * @param _width - window width
 * @param _height - window height
*/
void RenderManager::Initialize(int _width, int _height)
{
	window.GenerateWindow("CS550", { _width, _height });
	InitializeOpenGL();

	camera.Initialize();
	camera.SetProjection(60.0f, { _width, _height }, 0.001f, 1000.0f);

	shaders[Shader_Type::LIGHT] = new Shader("Resources/shaders/light.vert", "Resources/shaders/light.frag");
	shaders[Shader_Type::DEBUG] = new Shader("Resources/shaders/color.vert", "Resources/shaders/color.frag");
	shaders[Shader_Type::CUBEMAP] = new Shader("Resources/shaders/CubeMap.vert", "Resources/shaders/CubeMap.frag");
	shaders[currentShader]->Use();
	mat = new Material();
	light = new Light();
	UploadLightingUniforms();

	for (auto& m : mMeshes)
		m->CreateMesh();
	CreateCubeMap();

	ImGuiMgr.Initialize();
}

void RenderManager::SetShader(Shader_Type type)
{
	currentShader = type;
	shaders[currentShader]->Use();
}

/**
 * Clears the resources
*/
void RenderManager::Shutdown()
{
	for (auto& m : mMeshes)
	{
		m->DestroyMesh();
		delete m;
		m = nullptr;
	}
	mMeshes.clear();
}

/**
 * Begins a new frame
*/
void RenderManager::StartFrame()
{
	glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	SetShader(Shader_Type::LIGHT);
	ImGuiMgr.StartFrame();
}

/**
 * Ends the frame
*/
void RenderManager::EndFrame() const
{
	ImGuiMgr.EndFrame();
}

/**
 * Renders all the objects in the scene
*/
void RenderManager::RenderAll()
{
	camera.Update();
	shaders[Shader_Type::LIGHT]->SetUniform("World2Cam", camera.GetView());

	auto& objects = PhysicsMgr.GetBodies();
	for(auto i = 0u; i < objects.size(); ++i)
	{
		auto& rb = objects[i];
		auto m2w = rb.mTransform.GetModelToWorld();
		RenderMesh(*mMeshes[i], m2w, mMeshes[i]->GetColor(), PolygonMode_t::Solid);
	}

	//RenderEditingBody();
	RenderCubeMap();
	ImGuiMgr.Render();
	
	window.Swap();
}

/**
 * Renders a given mesh
 * @param _mesh - mesh to render
 * @param m2w - m2w matrix to render the mesh
 * @param _color - color
 * @param _mode - wireframe, pointcloud or solid
 * @param _drawMode - drawing mode to render
*/
void RenderManager::RenderMesh(const Mesh& _mesh, const glm::mat4& m2w, const glm::vec4& _color, PolygonMode_t _mode, DrawMode_t _drawMode)
{
	auto M2VP = camera.GetMtx() * m2w;

	glBindVertexArray(_mesh.GetVAO());
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _mesh.GetIBO());

	shaders[currentShader]->SetUniform("uniform_mvp", M2VP);
	shaders[currentShader]->SetUniform("uniform_color", _color);
	if(currentShader == Shader_Type::LIGHT)
		shaders[Shader_Type::LIGHT]->SetUniform("Model2World", m2w);

	switch (_mode)
	{
		case PolygonMode_t::PointCloud:	glPolygonMode(GL_FRONT_AND_BACK, GL_POINT); break;
		case PolygonMode_t::Wireframe:	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); break;
		case PolygonMode_t::Solid:		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); break;
	}
	switch (_drawMode)
	{
		case DrawMode_t::Lines:		glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(_mesh.GetMeshData().mVertexIndices.size()));
		case DrawMode_t::Points:	glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(_mesh.GetMeshData().mVertexIndices.size()));
		case DrawMode_t::Triangles: glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(_mesh.GetMeshData().mVertexIndices.size()));
	}
		
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}


/**
 * Frees allocated memory
*/
RenderManager::~RenderManager()
{
	Shutdown();
	for (auto& s : shaders)
	{
		delete s.second;
		s.second = nullptr;
	}
	delete mat;
	mat = nullptr;
	delete light;
	light = nullptr;
}

/**
 * Creates and adds a new mesh
*/
void RenderManager::AddMesh()
{
	std::vector<std::string> paths;
	for (auto& it : std::filesystem::directory_iterator("Resources/Meshes/"))
		paths.push_back(it.path().string());
	auto randPath = paths[rand() % paths.size()];

	mMeshes.push_back(new Mesh("Resources/Meshes/cube.obj"));
	int random = rand() % 3;
	glm::vec4 col = glm::vec4(1.0f);
	col[random] = glm::linearRand(0.0f, 1.0f);
	mMeshes[mMeshes.size() - 1]->SetColor(col);
}

/**
 * Saves all mesh information in json format
 * @param _j: the container (json) in which to store the info
*/
void RenderManager::SaveScene(json& _j) const
{
	int i = 0;

	_j["Camera"] << camera;

	for (auto& mesh : mMeshes)
	{
		//store the Mesh using its container index as ID
		json temp;
		std::string name = "Mesh_" + std::to_string(i);
		temp[name] << *mesh;
		//add it to the json vector
		_j["Meshes"].push_back(temp);
		++i;
	}
}

/**
 * Loads a given scene.
 * @param _j: the scene to load
*/
void RenderManager::LoadScene(json _j)
{
	//we first clear the current scene
	Shutdown();

	if (_j.find("Camera") != _j.end())
		_j["Camera"] >> camera;

	//find meshes
	if (_j.find("Meshes") != _j.end())
	{
		int i = 0;
		json& meshes = *_j.find("Meshes");
		//iterate through every stored mesh
		for (auto it = meshes.begin(); it != meshes.end(); ++it)
		{
			//take its name, following our convention
			std::string name = "Mesh_" + std::to_string(i);
			json& mesh = *it;
			//create our mesh and load its info
			Mesh* m = new Mesh();
			mesh[name] >> *m;
			m->CreateMesh();
			//add it to the container
			mMeshes.push_back(m);
			++i;
		}
	}
}

/**
 * Renders currently edting body
*/
void RenderManager::RenderEditingBody()
{
	auto editing = EDITOR.GetEditingBody();
	if (editing != -1)
	{
		auto m2w = PhysicsMgr.GetBodies()[editing].mTransform.GetModelToWorld();
		RenderMesh(*mMeshes[editing], m2w, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f), PolygonMode_t::Wireframe);
	}
}

/**
 * Initializes OpenGL context
*/
void RenderManager::InitializeOpenGL() const
{
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

/**
 * Uploads lighting uniforms
*/
void RenderManager::UploadLightingUniforms()
{
	//shaders[Shader_Type::LIGHT]->SetUniform("light.lightPosition", light->lightPosition);
	shaders[Shader_Type::LIGHT]->SetUniform("light.lightVector", light->lightVector);
	//shaders[Shader_Type::LIGHT]->SetUniform("light.LightColor", light->LightColor);
	shaders[Shader_Type::LIGHT]->SetUniform("light.ambientColor", light->ambientColor);
	shaders[Shader_Type::LIGHT]->SetUniform("light.diffuseColor", light->diffuseColor);
	shaders[Shader_Type::LIGHT]->SetUniform("light.specularColor", light->specularColor);
	//shaders[Shader_Type::LIGHT]->SetUniform("light.attenuation_1", light->attenuation_1);
	//shaders[Shader_Type::LIGHT]->SetUniform("light.attenuation_2", light->attenuation_2);
	//shaders[Shader_Type::LIGHT]->SetUniform("light.attenuation_3", light->attenuation_3);
	//shaders[Shader_Type::LIGHT]->SetUniform("light.innerCosCutOff", light->innerCosCutOff);
	//shaders[Shader_Type::LIGHT]->SetUniform("light.outerCosCutOff", light->outerCosCutOff);
	//shaders[Shader_Type::LIGHT]->SetUniform("light.fallOff", light->fallOff);
	shaders[Shader_Type::LIGHT]->SetUniform("material.ambient", mat->ambient);
	shaders[Shader_Type::LIGHT]->SetUniform("material.diffuse", mat->diffuse);
	shaders[Shader_Type::LIGHT]->SetUniform("material.specular", mat->specular);
	shaders[Shader_Type::LIGHT]->SetUniform("material.shininess", mat->shininess);
}

/**
 * Creates cube map
*/
void RenderManager::CreateCubeMap()
{
	float skyboxVertices[] = {
		// positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f
	};

	cubeMap = new CubeMap();
	unsigned int skyboxVAO, skyboxVBO;
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	cubeMap->vao = skyboxVAO;

	glGenTextures(1, &cubeMap->tex);
	glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMap->tex);
	std::vector<std::string> faces{ "right", "left", "top", "bottom", "front", "back" };
	//generate 1 image per face
	int width, height, comp;
	for (GLuint i = 0; i < faces.size(); ++i)
	{
		auto path = cubeMap->dir + "/" + faces[i] + ".jpg";
		unsigned char* data = stbi_load(path.c_str(), &width, &height, &comp, 0);
		if (data)
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_SRGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		else
			std::cout << "Cubemap texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}

	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	shaders[Shader_Type::CUBEMAP]->SetUniform("cubeMap", 0);
}

/**
 * Renders Cube Map
*/
void RenderManager::RenderCubeMap()
{
	shaders[Shader_Type::CUBEMAP]->Use();
	auto view = glm::mat4(glm::mat3(camera.GetView()));
	shaders[Shader_Type::CUBEMAP]->SetUniform("uniform_mvp", camera.GetProj() * view);
	glDisable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBindVertexArray(cubeMap->vao);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMap->tex);
	glDrawArrays(GL_TRIANGLES, 0, 36);
	glBindVertexArray(0);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glDepthFunc(GL_LESS);
	shaders[currentShader]->Use();
}

