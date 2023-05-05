// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the Physics Manager class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#pragma once
#include <vector>
#include "../Utilities/Singleton.h"
#include "../Graphics/Shader.h"
#include "../Graphics/Window.h"
#include "../Graphics/Camera.h"
#include "../Math/geometry.h"
#include "Collider.h"
#include "Rigidbody.h"

struct ContacPoint
{
	glm::vec3 pt;
	glm::vec3 RAcrossN;
	glm::vec3 RBcrossN;
	glm::vec3 RAcrossU;
	glm::vec3 RBcrossU;
	glm::vec3 RAcrossV;
	glm::vec3 RBcrossV;
	float contactLambda{ 0.0f };
	float restitutionBias{ 0.0f };
	float uFrictionLambda{ 0.0f };
	float vFrictionLambda{ 0.0f };
	float effMassContact{ 0.0f };
	float effMassU{ 0.0f };
	float effMassV{ 0.0f };
	float penetration{ 0.0f };
};

struct ContactManifold
{
	void GenerateManifold(float _sepEdge, float _sepPfromQ, float _sepQfromP,
						 Face* _faceP, Face* _faceQ, Edge* _e1, Edge* _e2, 
						 const glm::mat4& _m2wP, const glm::mat4& _m2wQ, 
						 const Geometry::Plane& _planeP, const Geometry::Plane& _planeQ,
						 const HalfEdge* _P, const HalfEdge* _Q);
	void GetIncindentFace(const HalfEdge* _incidentPolyTope, const glm::mat4& _m2wReference, const glm::mat4& _m2wIncident);
	void ClipFaces(const glm::mat4& _m2wReference, const glm::mat4& _m2wIncident, const Geometry::Plane& _referencePlane);
	void ComputeConstants(const Rigidbody* _rbA, const Rigidbody* _rbB);
private:
	void SutherlandHodgeman(const std::vector<glm::vec3>& _pointsToClip, const Geometry::Plane& _p, std::vector<glm::vec3>& _negative) const;
public:

	std::vector<ContacPoint> contactPoints{};
	glm::vec3 contactNormal{};
	glm::vec3 uFriction{}; 
	glm::vec3 vFriction{};
	Face* incidentFace{}; //debug
	Face* referenceFace{}; //debug
	Edge* eP{}; //debug
	Edge* eQ{}; //debug
	Geometry::Segment s; //debug
	unsigned short bodyA{};
	unsigned short bodyB{};
	bool isEdgeCollision{ false }; //debug
	bool isPReference{ true }; //debug. if false, P is incident and Q reference
};

class PhysicsManager
{
	MAKE_SINGLETON(PhysicsManager)
public:
	void Initialize();
	void Update();
	void Shutdown();

	~PhysicsManager();

	const std::vector<Rigidbody>& GetBodies() const { return mBodies; }
	std::vector<Rigidbody>& GetBodies() { return mBodies; }
	std::vector<std::vector<Rigidbody>>& GetPrevFrames() { return mPreviousFramesRigidBody; }
	void SetUpdateAll(bool _update) { mbUpdatePhysics = _update; }
	void SaveScene(json& _j) const;
	void LoadScene(json _j);
	void CreateRigidBody();

	float& GetBeta() { return beta; }
	float& GetDt() { return dt; }
	float& GetRestitution() { return restitutionCoeff; }
	int& GetContactIterations() { return mContactSolverIterations; }
	int& GetPrevFrameIdx() { return currentFrame; }
	bool& GetUpdatePhysics() { return mbUpdatePhysics; }
	bool& GetFrameByFrame() { return mbFrameByFrame; }
	bool& GetFixedDt() { return mbFixedDt; }

	bool mbShowManifold = false;
private:
	void CollideBodies();
	bool SeparatingAxisTheorem(const HalfEdge* _P, const HalfEdge* _Q, const glm::mat4& _m2wP, const glm::mat4& _m2wQ, ContactManifold* _c) const;
	void SolveConstraints(std::vector<ContactManifold>& _contacts);
	void SolvePenetration(std::vector<ContactManifold>& _contacts);
	void SolveFriction(std::vector<ContactManifold>& _contacts);
	float MaxPenetration(const std::vector<glm::vec3>& _worldPointsP, const std::vector<glm::vec3>& _worldPointsQ, const glm::vec3& _dir) const;
	bool ArcIntersection(const glm::vec3& _a, const glm::vec3& _b, const glm::vec3& _c, const glm::vec3& _d, const glm::mat4& _m2wP, const glm::mat4& _m2wQ) const;
	float DistanceEdgeEdge(const Edge& _a, const Edge& _b, const glm::mat4& _m2wP, const glm::mat4& _m2wQ, const glm::vec3& _centroid, glm::vec3& _normal) const;
	void UpdateRigidbodies();

	void DebugDrawContactManifold(const ContactManifold& _c, const glm::mat4& _m2wP, const glm::mat4& _m2wQ) const;

	std::vector<Rigidbody> mBodies;
	std::vector<std::vector<Rigidbody>> mPreviousFramesRigidBody;

	float beta = 0.2f;
	float dt = 1.0f / 60.0f;
	float restitutionCoeff = 0.3f;
	unsigned maxFrames = 200;
	int currentFrame = 0;
	unsigned currentStartingIndex = 0;
	int mContactSolverIterations = 15;
	int mFrictionSolverIterations = 1;
	bool mbUpdatePhysics = true;
	bool mbFrameByFrame = false;
	bool mbApplyFriction = true;
	bool mbFixedDt = true;
};

#define PhysicsMgr PhysicsManager::Instance()