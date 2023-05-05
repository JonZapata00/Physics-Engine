// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the HalfEdge class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#pragma once
#include "../Graphics/Mesh.h"
#include "../Graphics/DebugRenderer.h"
#include "../Math/math.h"


struct Edge;
struct Face
{
    void MergeFaces(Face* _f1, Face* _f2);
    void MergeEdges();
    void Shutdown();
    std::vector<Edge*> edges;
    std::vector<glm::vec3*> vertices;
    glm::vec3 normal;
};

struct Edge
{
    glm::vec3* vertex;
    Face* face;
    Edge* next;
    Edge* prev;
    Edge* twin;
};

struct HalfEdge
{
    HalfEdge();
	void Initialize(MeshData& _data = DebugRenderMgr.GetResources().cube->GetMeshData());
	void Update();
	void Shutdown();
    void AddFaceProcedurally(glm::vec3* _p1, glm::vec3* _p2, glm::vec3* _p3);

    static glm::vec3 Support(const glm::vec3& _dir, const std::vector<glm::vec3>& _points);
    
    //head of the list
    std::vector<Face*> mFaces;
    std::vector<glm::vec3> mVertices;

private:

    void AddFace(glm::vec3* _p1, glm::vec3* _p2, glm::vec3* _p3);
    void ConvertToEdgecentric(MeshData& _data);
    void GenerateFaces(MeshData& _data);
    void GetTwinEdges();
    void Sanitize();
    Face* MergeFaces(Face* _f1, Face* _f2) const;

    
};