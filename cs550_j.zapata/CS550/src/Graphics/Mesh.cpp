// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Mesh class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include "../Utilities/pch.hpp"
#include "Mesh.h"
#include <GL/GLU.h>

/**
 * loads a mesh from an obj
 * @param filename - the obj file to read
*/
void Mesh::LoadObj(const char* filename)
{
    MeshData& data = mMeshData;
    std::vector<unsigned> vertexIndices, uvIndices, normalIndices;
    std::vector<glm::vec3> tempVertices;
    std::vector<glm::vec2> tempUvs;
    std::vector<glm::vec3> tempNormals;

    std::ifstream mFile(filename);

    std::string lineHeader;
    std::string attValue;
    if (!mFile.is_open())
    {
        std::cerr << "Impossible to open the file\n" << std::endl;
        return;
    }

    while (!mFile.eof())
    {
        mFile >> lineHeader;

        ReadValuesFromFile(mFile, lineHeader, tempVertices, tempNormals, tempUvs);
        //parsing the face
        if (lineHeader == "f")
        {
            std::string faceLine;
            std::getline(mFile, faceLine);

            //checking the format
            int matches[3] = { -1 };
            unsigned vertexIndex[3], uvIndex[3], normalIndex[3];
            if (!FormatMatches(faceLine, vertexIndex, uvIndex, normalIndex, matches)) return;

            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);

            //we are not always given normals
            if (matches[0] == 9 || matches[1] == 6)
            {
                normalIndices.push_back(normalIndex[0]);
                normalIndices.push_back(normalIndex[1]);
                normalIndices.push_back(normalIndex[2]);
            }

            //in some cases, we are not given UVs 
            if (matches[0] == 9)
            {
                uvIndices.push_back(uvIndex[0]);
                uvIndices.push_back(uvIndex[1]);
                uvIndices.push_back(uvIndex[2]);
            }
        }

    }
    for(auto i : vertexIndices)
        data.mVertexIndices.push_back(i - 1);

    data.positions = tempVertices;
   
    //if(normalIndices.empty()) GenerateNormals();
    //else
    //{
    //    for (auto i : normalIndices)
    //        data.mNormalIndices.push_back(i - 1);
    //    data.normals = tempNormals;
    //}
    //we then process the data we stored to actually build the shape
   //unsigned vtxIndicesSize = static_cast<unsigned>(vertexIndices.size());
   //for (unsigned i = 0; i < vtxIndicesSize; ++i)
   //{
   //    glm::vec3 vertex = tempVertices[vertexIndices[i] - 1];
   //    data.positions.push_back(vertex);
   //}

    //if there are normals
    //if (!normalIndices.empty())
    //{
    //    unsigned normalIndicesSize = static_cast<unsigned>(normalIndices.size());
    //    for (unsigned i = 0; i < normalIndicesSize; ++i)
    //    {
    //        glm::vec3 normal = tempNormals[normalIndices[i] - 1];
    //        data.normals.push_back(normal);
    //    }
    //}
    //
    ////if there are uvs
    //if (!uvIndices.empty())
    //{
    //    unsigned uvsIndicesSize = static_cast<unsigned>(uvIndices.size());
    //    for (unsigned i = 0; i < uvsIndicesSize; ++i)
    //    {
    //        glm::vec2 uv = tempUvs[uvIndices[i] - 1];
    //        data.uvs.push_back(uv);
    //    }
    //}
}


/**
 * Reads position, normal and uvs data from a given file
 * @param file - the file to read from
 * @param lineHeader - the header of the line we're reading
 * @param positions - vector to store postions
 * @param normals - vector to store normals
 * @param uvs - vector to store uvs
*/
void Mesh::ReadValuesFromFile(std::ifstream& file, const std::string& lineHeader, std::vector<glm::vec3>& positions, std::vector<glm::vec3>& normals, std::vector<glm::vec2>& UVs)
{
    std::string attValue;

    //vertex position
    if (lineHeader == "v")
    {
        glm::vec3 vertex;
        file >> attValue;
        vertex.x = static_cast<float>(std::atof(attValue.c_str()));
        file >> attValue;
        vertex.y = static_cast<float>(std::atof(attValue.c_str()));
        file >> attValue;
        vertex.z = static_cast<float>(std::atof(attValue.c_str()));
        positions.push_back(vertex);
    }
    //texture coordinates
    else  if (lineHeader == "vt")
    {
        glm::vec2 UV;
        file >> attValue;
        UV.x = static_cast<float>(std::atof(attValue.c_str()));
        file >> attValue;
        UV.y = static_cast<float>(std::atof(attValue.c_str()));
        UVs.push_back(UV);
    }
    //vertex normals
    else if (lineHeader == "vn")
    {
        glm::vec3 normal;
        file >> attValue;
        normal.x = static_cast<float>(std::atof(attValue.c_str()));
        file >> attValue;
        normal.y = static_cast<float>(std::atof(attValue.c_str()));
        file >> attValue;
        normal.z = static_cast<float>(std::atof(attValue.c_str()));
        normals.push_back(normal);
    }
}

/**
 * checks whether the face can be read by checking the format
 * @param line - line to read from the file (a face)
 * @param vertexIndex - array to store vertex indices
 * @param uvIndex - array to store uv indices (if any)
 * @param normalIndex - array to store normal indices (if any)
 * @param matches - number to get which format is this obj using
 * @return true if we are given positions, positions and normals or positions, normals and uvs.
*/
bool Mesh::FormatMatches(const std::string& line, unsigned* vertexIndex, unsigned* uvIndex, unsigned* normalIndex, int* matches)
{
    matches[0] = sscanf_s(line.c_str(), "%u/%u/%u %u/%u/%u %u/%u/%u\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2]);
    matches[1] = sscanf_s(line.c_str(), "%u//%u %u//%u %u//%u\n", &vertexIndex[0], &normalIndex[0], &vertexIndex[1], &normalIndex[1], &vertexIndex[2], &normalIndex[2]);
    matches[2] = sscanf_s(line.c_str(), "%u %u %u\n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2]);

    //this conditions are added according to which type of objs we are reading, since they are handed in different formats.
    if (matches[0] != 9 && matches[1] != 6 && matches[2] != 3 && line != "")
    {
        printf("File can't be read: ( Try exporting with other options\n");
        return false;
    }
    return true;
}

/**
 * Generates normales given vertex data
*/
std::vector<glm::vec3> Mesh::GenerateNormals()
{
    std::vector<glm::vec3> renderingPos;
    for (size_t i = 0; i < mMeshData.mVertexIndices.size(); i += 3)
    {
        auto v1 = mMeshData.positions[mMeshData.mVertexIndices[i + 0]];
        auto v2 = mMeshData.positions[mMeshData.mVertexIndices[i + 1]];
        auto v3 = mMeshData.positions[mMeshData.mVertexIndices[i + 2]];
        auto n = glm::normalize(glm::cross(v2 - v1, v3 - v1));

        mMeshData.normals.push_back(n);
        mMeshData.normals.push_back(n);
        mMeshData.normals.push_back(n);

        renderingPos.push_back(v1);
        renderingPos.push_back(v2);
        renderingPos.push_back(v3);
    }
    return renderingPos;
}

/**
 * stores the obj file to load
 * @param file - the obj file to store and load
*/
Mesh::Mesh(const std::string& _objFile)
{
    mMeshPath = _objFile;
    CreateMesh();
}

/**
 * Generates a mesh given some positions
 * @param _positions
*/
Mesh::Mesh(const std::vector<glm::vec3>& positions)
{
	mMeshData.positions = positions;
	GenerateBuffers();
}

/**
 * Generates a mesh given some positions and indices
 * @param _positions
 * @param _indices
*/
Mesh::Mesh(const std::vector<glm::vec3>& _positions, const std::vector<unsigned>& _indices)
{
    mMeshData.positions = _positions;
    mMeshData.mVertexIndices = _indices;
    GenerateBuffers();
}

/**
 * Creates a Mesh with the given file
*/
void Mesh::CreateMesh()
{
	//load our corresponding obj
	LoadObj(mMeshPath.c_str());

	//generate and bind array object
	GenerateBuffers();
}

/**
 * Creates a mesh given a file
*/
void Mesh::CreateMesh(const std::string& _objFile)
{
    mMeshPath = _objFile;
    CreateMesh();
}

std::vector<Geometry::Triangle> Mesh::GetTriangles() const
{
    std::vector<Geometry::Triangle> triangles;

    for (uint32_t i = 0; i < mMeshData.mVertexIndices.size(); i += 3) 
    {
        triangles.push_back(
            {
                mMeshData.positions[mMeshData.mVertexIndices[i + 0]],
                mMeshData.positions[mMeshData.mVertexIndices[i + 1]],
                mMeshData.positions[mMeshData.mVertexIndices[i + 2]],
            });
    }
    return triangles;
}

/**
 * Frees the buffers we allocated
*/
void Mesh::DestroyMesh()
{
	glDeleteBuffers(2, mVBOs);
	glDeleteVertexArrays(1, &mVAO);
	mMeshData.positions.clear();
    mMeshData.mVertexIndices.clear();
	mMeshData.normals.clear();
	//mMeshData.uvs.clear();
}

/**
 * Generates buffers for the Mesh
*/
void Mesh::GenerateBuffers()
{
    std::vector<glm::vec3> renderPos = mMeshData.positions;
    if(mMeshData.mVertexIndices.size() >= 3u) renderPos = GenerateNormals();
	//create and bind buffers
	glGenVertexArrays(1, &mVAO);
	glGenBuffers(2, mVBOs);
	glBindVertexArray(mVAO);
	glBindBuffer(GL_ARRAY_BUFFER, mVBOs[0]);

	// Give our vertices to OpenGL.
	glBufferData(GL_ARRAY_BUFFER, renderPos.size() * sizeof(renderPos[0]), &renderPos[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mIBO);
    //glBufferData(GL_ELEMENT_ARRAY_BUFFER, mMeshData.mVertexIndices.size() * sizeof(unsigned int), &mMeshData.mVertexIndices[0], GL_STATIC_DRAW);

	//in case we have normals
	if (!mMeshData.normals.empty())
	{
		glBindBuffer(GL_ARRAY_BUFFER, mVBOs[1]);
		glBufferData(GL_ARRAY_BUFFER, mMeshData.normals.size() * sizeof(mMeshData.normals[0]), &mMeshData.normals[0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    }

	//in case we have uvs
	//if (!mMeshData.uvs.empty())
	//{
	//	glBindBuffer(GL_ARRAY_BUFFER, mVBOs[2]);
	//	glBufferData(GL_ARRAY_BUFFER, mMeshData.uvs.size() * sizeof(mMeshData.uvs[0]), &mMeshData.uvs[0], GL_STATIC_DRAW);
	//	glEnableVertexAttribArray(2);
	//	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
	//}

	//unbind
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void Mesh::ToJson(nlohmann::json& j) const
{
    j["Mesh path"] << mMeshPath;
    j["Color"] << color;
}

void Mesh::FromJson(const nlohmann::json& j)
{
    j["Mesh path"] >> mMeshPath;
    j["Color"] >> color;
}


/**
 * gets tge Mesh data of this Mesh
 * @return the Mesh data of the Mesh
*/
MeshData Mesh::GetMeshData() const { return mMeshData; }

/**
 * gets the VAO of this Mesh
 * @return the VAO data of the Mesh
*/
GLuint Mesh::GetVAO() const { return mVAO; }

