// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the Rigidbody class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#pragma once
#include "../Utilities/jsonWrapper.h"
#include "Collider.h"

struct Rigidbody
{
	Rigidbody();
	~Rigidbody();
	Rigidbody& operator= (const Rigidbody& _rhs);
	void Initialize();
	void Update();
	void Shutdown();
	void Integrate(float dt);
	void ApplyForceAndTorque(const glm::vec3& _force, const glm::vec3& _point);
	void ApplyForce(const glm::vec3& _force);
	void ComputeInertiaTensor(const std::vector<Geometry::Triangle>& _triangles);
	void ClearForces();

	void SetLinearVelocity(const glm::vec3& _vel);
	void SetLinearMomentum(const glm::vec3& _mom);
	void SetAngularVelocity(const glm::vec3& _angVel);
	void SetAngularMomentum(const glm::vec3& _angMom);

	friend json& operator<<(nlohmann::json& j, const Rigidbody& val) { val.ToJson(j); return j; }
	friend void operator>>(const nlohmann::json& j, Rigidbody& val) { val.FromJson(j); }

	void ToJson(nlohmann::json& j) const;
	void FromJson(const nlohmann::json& j);

	glm::mat3 ComputeOmegaMatrix() const;

	Transform3D mTransform;

	//inertia tensors
	glm::mat3 mInertiaTensorWorld{};
	glm::mat3 mInertiaTensorWorldInverse{};
	glm::mat3 mInertiaTensorLocal = glm::identity<glm::mat3>();
	glm::mat3 mInertiaTensorLocalInverse{};

	//velocity quantities
	glm::vec3 mVelocity{};
	glm::vec3 mLinearMomentum{};
	glm::vec3 mAngularVelocity{};
	glm::vec3 mAngularMomentum{};

	//forces
	glm::vec3 mForce{};
	glm::vec3 mTorque{};

	glm::vec3 CM{};

	HalfEdge* mCollider = nullptr;
	
	//mass
	float mMass = 1.0f;;
	float mInvMass{};
	float mFrictionCoeff = 0.2f;
	
	unsigned short mID{};

	bool mbApplyForces = true;
	bool mbLockPosition = false;
	bool mbApplyGravity = true;
};