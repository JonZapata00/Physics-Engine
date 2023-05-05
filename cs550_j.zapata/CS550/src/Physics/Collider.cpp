// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the HalfEdge class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include "../Utilities/pch.hpp"
#include "../Graphics/DebugRenderer.h"
#include "Collider.h"

/**
 * Constructor. Generates its own mesh.
*/
HalfEdge::HalfEdge()
{
}

/**
 * Initializes mesh and converts it to edgecentric
*/
void HalfEdge::Initialize(MeshData& _data)
{
    ConvertToEdgecentric(_data);
}

/**
 * Update
*/
void HalfEdge::Update()
{
}

/**
 * Clears all memory allocated
*/
void HalfEdge::Shutdown()
{
    for (auto& face : mFaces)
    {
        face->Shutdown();
        delete face;
        face = nullptr;
    }
    mFaces.clear();
    mVertices.clear();
}

/**
 * Procedurally adds a new face
 * @param _p1
 * @param _p2
 * @param _p3
*/
void HalfEdge::AddFaceProcedurally(glm::vec3* _p1, glm::vec3* _p2, glm::vec3* _p3)
{
    AddFace(_p1, _p2, _p3);
    GetTwinEdges();
    Sanitize();
}

/**
 * Support function to retrieve the furthest point in an object at a given direction
 * @param _dir: direction along which to project
 * @param _points: points to project along dir
 * @return furthest point
*/
glm::vec3 HalfEdge::Support(const glm::vec3& _dir, const std::vector<glm::vec3>& _points)
{
    //we simply keep the maximum dot product
    glm::vec3 supp{};
    float maxDist = -std::numeric_limits<float>::max();
    for (auto q : _points)
    {
        auto proj = glm::dot(q, _dir);
        if (proj > maxDist)
        {
            maxDist = proj;
            supp = q;
        }
    }

    return supp;
}


/**
 * Adds a new face
 * @param _p1
 * @param _p2
 * @param _p3
*/
void HalfEdge::AddFace(glm::vec3* _p1, glm::vec3* _p2, glm::vec3* _p3)
{
    mFaces.push_back(new Face());
    auto& face = mFaces[mFaces.size() - 1];

    //creaete edges
    Edge* e1 = new Edge();
    Edge* e2 = new Edge();
    Edge* e3 = new Edge();

    //set pointers as they correspond
    e1->next = e2;
    e1->prev = e3;
    e1->vertex = _p1;

    e2->next = e3;
    e2->prev = e1;
    e2->vertex = _p2;

    e3->next = e1;
    e3->prev = e2;
    e3->vertex = _p3;

    //add to the list
    face->vertices.push_back(_p1); face->vertices.push_back(_p2); face->vertices.push_back(_p3);
    face->edges.push_back(e1); face->edges.push_back(e2); face->edges.push_back(e3);

    //compute the normal
    face->normal = glm::normalize(glm::cross(*_p2 - *_p1, *_p3 - *_p1));
    //set face ptr
    e1->face = e2->face = e3->face = face;
}

/**
 * Converts from triangle soup to edgecentric (using
 * half edge data structure)
*/
void HalfEdge::ConvertToEdgecentric(MeshData& _data)
{
    GenerateFaces(_data);
    GetTwinEdges();
    Sanitize();
}

/**
 * Generates faces (which will be, for now, triangles)
 * given position data
*/
void HalfEdge::GenerateFaces(MeshData& _data)
{
    for (auto i = 0u, j = 0u; i < _data.mVertexIndices.size(); i += 3, ++j)
    {
        //take points that will create the triangle
        auto &p1 = _data.positions[_data.mVertexIndices[i + 0]];
        auto &p2 = _data.positions[_data.mVertexIndices[i + 1]];
        auto &p3 = _data.positions[_data.mVertexIndices[i + 2]];
        AddFace(&p1, &p2, &p3);       
    }
    mVertices = _data.positions;
}

/**
 * Computes twin edges by iterating through each of the faces
*/
void HalfEdge::GetTwinEdges()
{

    //go through every face
    for (auto& f1 = mFaces.begin(); f1 != mFaces.end(); ++f1)
    {
        //check against every other face
        for (auto& f2 = std::next(f1); f2 != mFaces.end(); ++f2)
        {
            //check pairs of edges
            for (auto& e1 : (*f1)->edges)
            {
                for (auto& e2 : (*f2)->edges)
                {
                    //if edge 1's destination matches edge 2's origin and vice versa,
                    //then they're twins
                    if (e1->next->vertex == e2->vertex && e2->next->vertex == e1->vertex)
                    {
                        e1->twin = e2;
                        e2->twin = e1;
                    }
                }
            }
        }
    }
}

/**
 * Sanitizes the collider shape by merging faces
*/
void HalfEdge::Sanitize()
{
    //this vector will hold the new mesh with merged coplanar faces
    std::vector<Face*> temp = mFaces;
    //go through each face
    for (unsigned i = 0u; i < temp.size(); ++i)
    {
        //compare with the rest
        for (unsigned j = 0u; j < temp.size(); ++j)
        {
            //sanity check
            if (i == j) continue;
            auto& f1 = temp[i];
            auto& f2 = temp[j];
    
            //we already merged these
            if (!f1 || !f2) continue;
    
            //if normals coincide, they are coplanar
            //if (glm::epsilonEqual<glm::vec3>(f1->normal, f2->normal, glm::vec3(cEpsilon)))
            if(f1->normal == f2->normal)
            {
                //try to merge the faces (maybe they are not connected)
                Face* result = MergeFaces(f1, f2);
                if (result)
                {
                    //if success, remove previous faces
                    f1->edges.clear(); f1->vertices.clear();
                    f2->edges.clear(); f2->vertices.clear();
                    delete f1;
                    delete f2;
                    f1 = f2 = nullptr;
    
                    //add new face
                    temp.push_back(result);
                }
            }
        }
    }
    
    //we will restore the new values in our array
    mFaces.clear();
    for (auto i = 0u; i < temp.size(); ++i)
    {
        //if null, it means it was deleted and therefore merged, so we skip it
        if (temp[i])
            mFaces.push_back(temp[i]);
    }
}

/**
 * Merges a face given two other faces
 * @param _f1 - one of the faces to merge
 * @param _f2 - the other face to merge
*/
Face* HalfEdge::MergeFaces(Face* _f1, Face* _f2) const
{
    //we iterate through both faces' edges
    for (auto& e1 : _f1->edges)
    {
        for (auto& e2 : _f2->edges)
        {
            //if they are each other's twins, it means that
            //these faces share these edges
            if (e1->twin == e2 && e2->twin == e1)
            {
                //for now, we reset the ptrs and free the memory
                e1->prev->next = e1->next->prev = nullptr;
                e2->prev->next = e2->next->prev = nullptr;
                delete e1;
                delete e2;
                e1 = e2 = nullptr;

                //and now we will create the new face
                Face* merged = new Face();
                merged->MergeFaces(_f1, _f2);
                return merged;
            }
        }
    }
    return nullptr;
}

/**
 * Generates a new face from two other faces
 * @param _f1 - one of the faces to merge
 * @param _f2 - the other face to merge
*/
void Face::MergeFaces(Face* _f1, Face* _f2)
{
    //copy valid edges (if null it means it was merged)
    for (auto& e1 : _f1->edges)
    {
        if (e1)
        {
            e1->face = this;
            edges.push_back(e1);
        }
    }
    for (auto& e2 : _f2->edges)
    {
        if (e2)
        {
            e2->face = this;
            edges.push_back(e2);
        }
     }

    //set next and prev ptrs again
    MergeEdges();

    auto start = edges[0];
    auto current = start;
    do
    {
        vertices.push_back(current->vertex);
        current = current->next;
    } while (current != start);

    //copy normal
    normal = _f1->normal;
}

/**
 * Recomputes prev and next values of edges, so that
 * linked list is maintained even after merging
*/
void Face::MergeEdges()
{
    for (auto i = 0; i < edges.size(); ++i)
    {
        auto nextIdx = i + 1;
        if (nextIdx >= edges.size()) nextIdx = 0;
        auto prevIdx = i - 1;
        if (prevIdx < 0) prevIdx = (unsigned)edges.size() - 1;
        edges[i]->next = edges[nextIdx];
        edges[i]->prev = edges[prevIdx];
    }
}

/**
 * Frees the memory
*/
void Face::Shutdown()
{
    for (auto& e : edges)
    {
        delete e;
        e = nullptr;
    }
    edges.clear();
    vertices.clear();
}

