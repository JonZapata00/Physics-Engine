// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the Mesh class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#pragma once
#include <vector>
#include <string>
#include <glm/glm.hpp>
#include <GL/glew.h>
#include <GL/GL.h>
#include "../Math/math.h"
#include "../Utilities/jsonWrapper.h"

struct MeshData
{
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<unsigned> mVertexIndices;
    std::vector<unsigned> mNormalIndices;
};

class Mesh
{
public:
    Mesh() {}
    explicit Mesh(const std::string& _objFile);
    explicit Mesh(const std::vector<glm::vec3>& positions);
    explicit Mesh(const std::vector<glm::vec3>& _positions, const std::vector<unsigned>& _indices);
    void CreateMesh();
    void CreateMesh(const std::string& _objFile);
    MeshData GetMeshData() const;
    MeshData& GetMeshData() { return mMeshData; }
    std::vector<Geometry::Triangle> GetTriangles() const;
    void DestroyMesh();
    GLuint GetVAO() const;
    void GenerateBuffers();
    void SetPath(const std::string& _path) { mMeshPath = _path; }
    std::string GetPath() const { return mMeshPath; }

    friend json& operator<<(nlohmann::json& j, const Mesh& val) { val.ToJson(j); return j; }
    friend void operator>>(Mesh& val, const nlohmann::json& j) { val.FromJson(j); }
    friend void operator>>(const nlohmann::json& j, Mesh& val) { val.FromJson(j); }

    void ToJson(nlohmann::json& j) const;
    void FromJson(const nlohmann::json& j);

    glm::vec4 GetColor() const { return color; }
    void SetColor(const glm::vec4& _color) { color = _color; }

private:
    void LoadObj(const char* filename);
    void ReadValuesFromFile(std::ifstream& file, const std::string& lineHeader, std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2>& UVs);
    bool FormatMatches(const std::string& line, unsigned* vertexIndex, unsigned* uvIndex, unsigned* normalIndex, int* matches);
    std::vector<glm::vec3> GenerateNormals();

    MeshData mMeshData;
    std::string mMeshPath = "Resources/meshes/cube.obj";
    glm::vec4 color { 1.0f };
    GLuint mVAO = 0;
    GLuint mVBOs[2]{ 0 };
};
