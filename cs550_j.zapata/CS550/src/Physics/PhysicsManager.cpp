// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Physics Manager class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#include "../Utilities/pch.hpp"
#include "../Utilities/FrameRateController.h"
#include "../Math/math.h"
#include "../Graphics/RenderManager.h"
#include "../Graphics/DebugRenderer.h"
#include "PhysicsManager.h"

/**
 * Converts points to a given space
 * @param _m2w: transformation matrix
 * @param _localPoints: points to transform
 * @return points in world space
*/
std::vector<glm::vec3> TransformPoints(const glm::mat4& _m2w, const std::vector<glm::vec3>& _localPoints)
{
	std::vector<glm::vec3> world;
	for (auto p : _localPoints)
	{
		auto pWorld = glm::vec3(_m2w * glm::vec4(p, 1.0f));
		world.push_back(pWorld);
	}
	return world;
}

/**
 * Converts points to a given space
 * @param _m2w: transformation matrix
 * @param _localPoints: points to transform
 * @return points in world space
*/
std::vector<glm::vec3> TransformPoints(const glm::mat4& _m2w, const std::vector<glm::vec3*>& _localPoints)
{
	std::vector<glm::vec3> world;
	for (auto p : _localPoints)
	{
		auto pWorld = glm::vec3(_m2w * glm::vec4(*p, 1.0f));
		world.push_back(pWorld);
	}
	return world;
}

/**
 * Initializes all bodies in the scene
*/
void PhysicsManager::Initialize()
{
	mPreviousFramesRigidBody.resize(maxFrames);

	for (auto& rb : mBodies)
		rb.Initialize();
}

/**
 * Updates all bodies in the scene
*/
void PhysicsManager::Update()
{
	CollideBodies();
	if(mbUpdatePhysics)
		UpdateRigidbodies();
	if (mbFrameByFrame)
	{
		mPreviousFramesRigidBody.push_back(mBodies);
		mbUpdatePhysics = false;
		mbFrameByFrame = false;
		currentFrame = (int)mPreviousFramesRigidBody.size() - 1;
	}
}

/**
 * Bit of a hack, clears mesh resources of each rb
*/
void PhysicsManager::Shutdown()
{
	for (auto& rb : mBodies)
		rb.Shutdown();
	mBodies.clear();

	mPreviousFramesRigidBody.clear();
}

/**
 * Destructor
*/
PhysicsManager::~PhysicsManager()
{
	Shutdown();
}

/**
 * Saves Scene to a json
 * @param _j - The json container to store the saved info
*/
void PhysicsManager::SaveScene(json& _j) const
{
	int i = 0;
	_j["Number of bodies"] << (int)mBodies.size();
	//_j["Number of Colliders"] << (int)mColliders.size();

	for (auto& body : mBodies)
	{
		json temp;
		std::string name = "Rigidbody_" + std::to_string(i);
		temp[name] << body;
		_j["Rigidbodies"].push_back(temp);
		++i;
	}
}

/**
 * Loads Scene from a json
 * @param _j - The json container to load the saved info from
*/
void PhysicsManager::LoadScene(json _j)
{
	Shutdown();

	if (_j.find("Number of bodies") != _j.end())
	{
		int size = 0;
		_j["Number of bodies"] >> size;
		mBodies.resize(size);
	}

	if (_j.find("Rigidbodies") != _j.end())
	{
		int i = 0;
		json& bodies = *_j.find("Rigidbodies");
		for (auto it = bodies.begin(); it != bodies.end(); ++it)
		{
			std::string name = "Rigidbody_" + std::to_string(i);
			json& body = *it;
			body[name] >> mBodies[i];;
			++i;
		}
	}

	Initialize();
}

/**
 * Creates a new Rigidbody
*/
void PhysicsManager::CreateRigidBody()
{
	mBodies.push_back(Rigidbody());
	mBodies[mBodies.size() - 1].mID = (unsigned short)mBodies.size() - 1;
	mBodies[mBodies.size() - 1].Initialize();
}

/**
 * Checks collision detection between all bodies
*/
void PhysicsManager::CollideBodies()
{
	std::vector<ContactManifold> contactsThisFrame;
	for (auto rbA = mBodies.begin(); rbA != mBodies.end(); ++rbA)
	{
		for (auto rbB = std::next(rbA); rbB != mBodies.end(); ++rbB)
		{
			if (rbA->mMass < cEpsilon && rbB->mMass < cEpsilon) continue;
			ContactManifold c;
			auto m2wP = rbA->mTransform.GetModelToWorld();
			auto m2wQ = rbB->mTransform.GetModelToWorld();
			if(SeparatingAxisTheorem(rbA->mCollider, rbB->mCollider, m2wP, m2wQ, &c))
			{
				c.ComputeConstants((&*rbA), &(*rbB));
				contactsThisFrame.push_back(c);
				if(mbShowManifold)
					DebugDrawContactManifold(c, m2wP, m2wQ);
			}
			
		}
	}
	if(mbUpdatePhysics)
		SolveConstraints(contactsThisFrame);
}

/**
 * Performs the separating axis theorem between two objects
 * @param _P: object P
 * @param _Q: object Q
 * @param _m2wP: model 2 world matrix for object P
 * @param _m2wQ: model 2 world matrix for objecg Q
 * @param _c: contact manifold to be filled
 * @return true if objects intersect, false otherwise
*/
bool PhysicsManager::SeparatingAxisTheorem(const HalfEdge* _P, const HalfEdge* _Q, const glm::mat4& _m2wP, const glm::mat4& _m2wQ, ContactManifold* _c) const
{
	//get points in world space
	std::vector<glm::vec3> PWorldPoints = TransformPoints(_m2wP, _P->mVertices);
	std::vector<glm::vec3> QWorldPoints = TransformPoints(_m2wQ, _Q->mVertices);

	//initialize separations
	float sepQfromP = -std::numeric_limits<float>::max();
	float sepPfromQ = sepQfromP;
	float sepEdge = -std::numeric_limits<float>::max();

	//planes and edge vector
	Geometry::Plane planeP, planeQ{};

	Edge* eP{}, * eQ{};

	Face* faceP{}, * faceQ{};

	for (auto& f : _P->mFaces)
	{
		auto normalWorld = glm::vec3(_m2wP * glm::vec4(f->normal, 0.0f));
		auto pWorld = glm::vec3(_m2wP * glm::vec4(*f->vertices[0], 1.0f));
		auto plane = Geometry::Plane(pWorld, normalWorld);

		auto supp = HalfEdge::Support(-normalWorld, QWorldPoints);

		float sep = Geometry::DistancePointPlane(plane, supp);

		if (sep > 0.0f)
			return false;

		//Get minimum penetration along P's faces (aka the less negative)
		if (sep > sepPfromQ)
		{
			sepPfromQ = sep;
			faceP = f;
			planeP = plane;
		}
	}

	int faceIDXQ = 0;
	for (auto& f : _Q->mFaces)
	{
		auto normalWorld = glm::vec3(_m2wQ * glm::vec4(f->normal, 0.0f));
		auto pWorld = glm::vec3(_m2wQ * glm::vec4(*f->vertices[0], 1.0f));
		auto plane = Geometry::Plane(pWorld, normalWorld);

		auto supp = HalfEdge::Support(-normalWorld, PWorldPoints);

		float sep = Geometry::DistancePointPlane(plane, supp);

		if (sep > 0.0f)
			return false;

		//Get the minimum penetration along Q's faces
		if (sep > sepQfromP)
		{
			sepQfromP = sep;
			planeQ = plane;
			faceQ = f;
		}
	}

	//for each pair of edges
	for (auto& f1 : _P->mFaces)
	{
		for (auto& e1 : f1->edges)
		{
			//check against every other face
			for (auto& f2 : _Q->mFaces)
			{
				//check pairs of edges
				for (auto& e2 : f2->edges)
				{
					//Do the edges intersect (using a Gauss map)?
					if (ArcIntersection(f1->normal, e1->twin->face->normal, -f2->normal, -e2->twin->face->normal, _m2wP, _m2wQ))
					{
						//if so, we compute the distance to determine whether they form a minkowski face
						glm::vec3 v{};
						float bias = 0.01f;
						float sep = DistanceEdgeEdge(*e1, *e2, _m2wP, _m2wQ, glm::vec3(_m2wP[3]), v);
						if (sep > 0.0f) return false;
						sep -= bias;
						if (sep > sepEdge)
						{
							sepEdge = sep;
							eP = e1;
							eQ = e2;
						}
					}
				}
			}
		}
	}

	_c->GenerateManifold(sepEdge, sepPfromQ, sepQfromP, faceP, faceQ, eP, eQ, _m2wP, _m2wQ, planeP, planeQ, _P, _Q);

	return true;
}

/**
 * Solves every constraint using the Gauss Seidel iterative algorithm
 * @param _contacts: contacts stored this frame
*/
void PhysicsManager::SolveConstraints(std::vector<ContactManifold>& _contacts)
{
	SolvePenetration(_contacts);
	SolveFriction(_contacts);
}

/**
 * Solves penetration of all contacts stored this frame using
 * the Gauss Seidel iterative algorithm
 * @param _contacts: contacts stored this frame 
*/
void PhysicsManager::SolvePenetration(std::vector<ContactManifold>& _contacts)
{
	//Gauss Seidel iterative solver
	for (auto i = 0; i < mContactSolverIterations; ++i)
	{
		float firstJv = 0.0f;
		for (auto& c : _contacts)
		{
			auto& rbA = mBodies[c.bodyA];
			auto& rbB = mBodies[c.bodyB];

			auto invMA = rbA.mInvMass;
			auto& vA = rbA.mVelocity;
			auto& pA = rbA.mLinearMomentum;
			auto& wA = rbA.mAngularVelocity;
			auto& lA = rbA.mAngularMomentum;
			auto InvIWorldA = rbA.mInertiaTensorWorldInverse;

			auto invMB = rbB.mInvMass;
			auto& vB = rbB.mVelocity;
			auto& pB = rbB.mLinearMomentum;
			auto& wB = rbB.mAngularVelocity;
			auto& lB = rbB.mAngularMomentum;
			auto InvIWorldB = rbB.mInertiaTensorWorldInverse;

			for (auto j = 0u; j < c.contactPoints.size(); ++j)
			{
				auto& ctp = c.contactPoints[j];
				//bias = penetration * 1 / dt * factor
				float bias = ctp.penetration * 1.0f / dt * beta;
				//take restitution coefficient into account
				bias += ctp.restitutionBias;
				auto p = ctp.pt;

				auto crossPtNormalA = ctp.RAcrossN;
				auto crossPtNormalB = ctp.RBcrossN;

				//Compute Jacobian times velocity vector, with
				// J = [-n^t (-rA x n)^t n^t (rB x n)^t
				// v = [vA wA vB wB]
				float Jv = glm::dot(-c.contactNormal, vA) + glm::dot(-crossPtNormalA, wA) + glm::dot(c.contactNormal, vB) + glm::dot(crossPtNormalB, wB);
				//contactLambda = (Jv + bias) / JM^(-1)J^t
				//WARNING!!! Depending on the direction of the collision, might need to flip the contactLambda!!
				float contactLambda = Jv + bias;
				contactLambda /= ctp.effMassContact;

				//take old contactLambda and accumulate the new one
				float prevLambda = ctp.contactLambda;
				ctp.contactLambda += contactLambda;
				ctp.contactLambda = glm::max(0.0f, ctp.contactLambda);
				//compute delta contactLambda, as this will be the impulse to apply
				float deltaLambda = ctp.contactLambda - prevLambda;

				//Apply impulses as:
				// pAnew = pAprev + j * n;
				// lAnew = lAprev + j * (rA x n)
				rbA.SetLinearMomentum(pA + deltaLambda * c.contactNormal);
				rbA.SetAngularMomentum(lA + deltaLambda * crossPtNormalA);

				// pBnew = pBprev - j * n;
				// lBnew = lBprev - j * (rB x n)
				rbB.SetLinearMomentum(pB - deltaLambda * c.contactNormal);
				rbB.SetAngularMomentum(lB - deltaLambda * crossPtNormalB);
			}
		}
	}
}

/**
 * Solves friction constraint of all contacts stored this frame using
 * the Gauss Seidel iterative algorithm
 * @param _contacts: contacts stored this frame
*/
void PhysicsManager::SolveFriction(std::vector<ContactManifold>& _contacts)
{
	//FRICTION SOLVER
	//Gauss Seidel iterative solver
	for (auto i = 0; i < mFrictionSolverIterations; ++i)
	{
		for (auto& c : _contacts)
		{
			auto& rbA = mBodies[c.bodyA];
			auto& rbB = mBodies[c.bodyB];

			auto invMA = rbA.mInvMass;
			auto& vA = rbA.mVelocity;
			auto& pA = rbA.mLinearMomentum;
			auto& wA = rbA.mAngularVelocity;
			auto& lA = rbA.mAngularMomentum;
			auto InvIWorldA = rbA.mInertiaTensorWorldInverse;

			auto invMB = rbB.mInvMass;
			auto& vB = rbB.mVelocity;
			auto& pB = rbB.mLinearMomentum;
			auto& wB = rbB.mAngularVelocity;
			auto& lB = rbB.mAngularMomentum;
			auto InvIWorldB = rbB.mInertiaTensorWorldInverse;

			for (auto j = 0u; j < c.contactPoints.size(); ++j)
			{
				auto& ctp = c.contactPoints[j];
				float maxLambda = glm::min(rbA.mFrictionCoeff, rbB.mFrictionCoeff) * ctp.contactLambda;
				//bias = penetration * 1 / dt * factor
				//float bias = ctp.penetration * 1.0f / dt * beta;
				auto p = ctp.pt;

				auto crossPtUA = ctp.RAcrossU;
				auto crossPtUB = ctp.RBcrossU;
				auto crossPtVA = ctp.RAcrossV;
				auto crossPtVB = ctp.RBcrossV;

				//float Jv = glm::dot(-c.contactNormal, vA) + glm::dot(-crossPtNormalA, wA) + glm::dot(c.contactNormal, vB) + glm::dot(crossPtNormalB, wB);
				float Cfric1 = glm::dot(-c.uFriction, vA) + glm::dot(-crossPtUA, wA) + glm::dot(c.uFriction, vB) + glm::dot(crossPtUB, wB);

				//WARNING!!! Depending on the direction of the collision, might need to flip the contactLambda!!
				float uFrictionLambda = Cfric1 / ctp.effMassU;

				//take old contactLambda and accumulate the new one
				float prevULambda = ctp.uFrictionLambda;
				ctp.uFrictionLambda += uFrictionLambda;
				ctp.uFrictionLambda = glm::clamp(ctp.uFrictionLambda, -maxLambda, maxLambda);
				//compute delta contactLambda, as this will be the impulse to apply
				float deltaLambdaU = ctp.uFrictionLambda - prevULambda;

				//Apply impulses as:
				// pAnew = pAprev + j * n;
				// lAnew = lAprev + j * (rA x n)
				rbA.SetLinearMomentum(pA + deltaLambdaU * c.uFriction);
				rbA.SetAngularMomentum(lA + deltaLambdaU * crossPtUA);

				// pBnew = pBprev - j * n;
				// lBnew = lBprev - j * (rB x n)
				rbB.SetLinearMomentum(pB - deltaLambdaU * c.uFriction);
				rbB.SetAngularMomentum(lB - deltaLambdaU * crossPtUB);


				//FRICTION IN V
				float Cfric2 = glm::dot(-c.vFriction, vA) + glm::dot(-crossPtVA, wA) + glm::dot(c.vFriction, vB) + glm::dot(crossPtVB, wB);
				float vFrictionLambda = Cfric2 / ctp.effMassV;

				//take old contactLambda and accumulate the new one
				float prevVLambda = ctp.vFrictionLambda;
				ctp.vFrictionLambda += vFrictionLambda;
				ctp.vFrictionLambda = glm::clamp(ctp.vFrictionLambda, -maxLambda, maxLambda);
				//compute delta contactLambda, as this will be the impulse to apply
				float deltaLambdaV = ctp.vFrictionLambda - prevVLambda;

				//Apply impulses as:
				// pAnew = pAprev + j * n;
				// lAnew = lAprev + j * (rA x n)
				rbA.SetLinearMomentum(pA + deltaLambdaV * c.vFriction);
				rbA.SetAngularMomentum(lA + deltaLambdaV * crossPtVA);

				// pBnew = pBprev - j * n;
				// lBnew = lBprev - j * (rB x n)
				rbB.SetLinearMomentum(pB - deltaLambdaV * c.vFriction);
				rbB.SetAngularMomentum(lB - deltaLambdaV * crossPtVB);
			}
		}
	}
}

/**
 * Computes the minimum projection (= max penetration) at a given direction
 * @param _worldPointsP: points in world space of object P
 * @param _worldPointsQ: points in world space of object Q
 * @param _dir: direction along which to project
 * @return minimum projection (= max penetration)
*/
float PhysicsManager::MaxPenetration(const std::vector<glm::vec3>& _worldPointsP, const std::vector<glm::vec3>& _worldPointsQ, const glm::vec3& _dir) const
{
	//BE CAREFUL! COMPUTATIONS MUST BE DONE IN THE SAME SPACE
	float minProj = std::numeric_limits<float>::max(); 	

	for (auto p : _worldPointsP)
	{
		for (auto q : _worldPointsQ)
		{
			auto proj = glm::dot(q - p, _dir);
			if (proj < minProj)
				minProj = proj; 
		}

	}
	return minProj;
}

/**
 * Tests whether two edges form a minkowski face using Gauss maps
 * @param _a: normal 1 of edge 1
 * @param _b: normal 2 of edge 1
 * @param _c: normal 1 of edge 2
 * @param _d: normal 2 of edge 2
 * @param _m2wP: model 2 world matrix for edge 1
 * @param _m2wQ: model 2 world matrix for edge 2
 * @return true if edges form a minkowski face, false otherwise.
*/
bool PhysicsManager::ArcIntersection(const glm::vec3& _a, const glm::vec3& _b, const glm::vec3& _c, const glm::vec3& _d, const glm::mat4& _m2wP, const glm::mat4& _m2wQ) const
{
	//convert normals to world space (which will act as vertices 
	//in the Gauss maps)
	auto a = glm::vec3(_m2wP * glm::vec4(_a, 0.0f));
	auto b = glm::vec3(_m2wP * glm::vec4(_b, 0.0f));
	auto c = glm::vec3(_m2wQ * glm::vec4(_c, 0.0f));
	auto d = glm::vec3(_m2wQ * glm::vec4(_d, 0.0f));

	//compute products and perform checks as it corresponds
	auto ba = glm::cross(b, a);
	float cba = glm::dot(c, ba);
	float dba = glm::dot(d, ba);
	if (cba * dba >= 0.0f) return false;

	auto dc = glm::cross(d, c);
	float adc = glm::dot(a, dc);
	float bdc = glm::dot(b, dc);
	if (adc * bdc >= 0.0f) return false;

	auto cb = glm::cross(c, b);
	float acb = glm::dot(a, cb);
	float dcb = glm::dot(d, cb);
	if (acb * dcb <= 0.0f) return false;

	return true;
}


/**
 * Computes the distance between edges by generating a plane going through one of them.
 * @param _a: one of the edges
 * @param _b: the other edge.
 * @param _m2wP: model 2 world matrix for edge 1
 * @param _m2wQ: model 2 world matrix for edge 2
 * @param _centroid: centroid of polygon of edge 1
 * @param _normal: normal of collision
 * @return distance between both edges.
*/
float PhysicsManager::DistanceEdgeEdge(const Edge& _a, const Edge& _b, const glm::mat4& _m2wP, const glm::mat4& _m2wQ, const glm::vec3& _centroid, glm::vec3& _normal) const
{
	auto a = glm::vec3(_m2wP * glm::vec4(*_a.vertex, 1.0f));
	auto b = glm::vec3(_m2wP * glm::vec4(*_a.next->vertex, 1.0f));

	auto c = glm::vec3(_m2wQ * glm::vec4(*_b.vertex, 1.0f));
	auto d = glm::vec3(_m2wQ * glm::vec4(*_b.next->vertex, 1.0f));
	_normal = glm::cross(b - a, d - c);

	auto dot = glm::dot(_normal, b - _centroid);
	if (dot < 0.0f)	_normal = -_normal;

	float dist = glm::dot(_normal, c - b) / glm::length(_normal);

	//DebugRenderMgr.DebugDrawLine({ a, b, dist }, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
	//DebugRenderMgr.DebugDrawLine({ c, d, dist }, glm::vec4(0.0f, 1.0f, 1.0f, 1.0f));
	//DebugRenderMgr.DebugDrawLine({ a, a + _normal, dist }, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

	return dist;
}


/**
 * Updates all rigidbodies in the scene
*/
void PhysicsManager::UpdateRigidbodies()
{
	for (auto& rb : mBodies)
		rb.Update();
}

/**
 *
 Debug Draws contact manifold
 * @param _c: the contact manifold information
 * @param _m2wP: model 2 wolrd matrix of polygon A (contact manifold info is in local)
 * @param _m2wQ: model 2 wolrd matrix of polygon B (contact manifold info is in local)
*/
void PhysicsManager::DebugDrawContactManifold(const ContactManifold& _c, const glm::mat4& _m2wP, const glm::mat4& _m2wQ) const
{
	if (!_c.isEdgeCollision)
	{
		//start with the reference face (RED)
		auto start = _c.referenceFace->edges[0];
		auto current = start;
		do
		{
			glm::mat4 mat;
			if (_c.isPReference) mat = _m2wP;
			else				 mat = _m2wQ;
			auto p1 = glm::vec3(mat * glm::vec4(*current->vertex, 1.0f));
			auto p2 = glm::vec3(mat * glm::vec4(*current->next->vertex, 1.0f));
			DebugRenderMgr.DebugDrawLine({ p1, p2 }, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
			current = current->next;
		} while (current != start);

		//incident face
		start = _c.incidentFace->edges[0];
		current = start;
		do
		{
			glm::mat4 mat;
			if (!_c.isPReference) mat = _m2wP;
			else				 mat = _m2wQ;
			auto p1 = glm::vec3(mat * glm::vec4(*current->vertex, 1.0f));
			auto p2 = glm::vec3(mat * glm::vec4(*current->next->vertex, 1.0f));
			DebugRenderMgr.DebugDrawLine({ p1, p2 }, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
			current = current->next;
		} while (current != start);
	}
	else
	{
		if (_c.eP && _c.eQ)
		{
			auto a = glm::vec3(_m2wP * glm::vec4(*_c.eP->vertex, 1.0f));
			auto b = glm::vec3(_m2wP * glm::vec4(*_c.eP->next->vertex, 1.0f));

			auto c = glm::vec3(_m2wQ * glm::vec4(*_c.eQ->vertex, 1.0f));
			auto d = glm::vec3(_m2wQ * glm::vec4(*_c.eQ->next->vertex, 1.0f));

			DebugRenderMgr.DebugDrawLine({ a, b, glm::length(b - a) }, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
			DebugRenderMgr.DebugDrawLine({ c, d, glm::length(c - d) }, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
			DebugRenderMgr.DebugDrawLine({ _c.s.p0, _c.s.p1, glm::length(_c.s.p1 - _c.s.p0) }, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
		}
	}

	//contact points
	for (auto p : _c.contactPoints)
	{
		DebugRenderMgr.DebugDrawPoint({ p.pt, 6.0f }, glm::vec4(1.0f));
		DebugRenderMgr.DebugDrawLine({ p.pt, p.pt - _c.contactNormal * p.penetration , 1.0f}, glm::vec4(1.0f));
	}

}

/**
 *
 * Generates the contact manifold for a given collision
 * @param _sepEdge: computed separation for an edge-edeg collision
 * @param _sepPfromQ: computed separation for face-face collision, from P to Q
 * @param _sepQfromP: computed separation for face-face collision, from Q to P
 * @param _faceP: face candidate to be reference.
 * @param _faceQ: face candidate to be reference.
 * @param _eP, _eQ: possible edges yielding edge-edge collision
 * @param _m2wP: model 2 wolrd matrix of polygon A (contact manifold info is in local)
 * @param _m2wQ: model 2 wolrd matrix of polygon B (contact manifold info is in local)
 * @param _planeP: candidate to be reference plane
 * @param _planeQ: candidate to be reference plane
 * @param _P: collider P
 * @param _Q: collider Q
*/
void ContactManifold::GenerateManifold(float _sepEdge, float _sepPfromQ, float _sepQfromP,
									   Face* _faceP, Face* _faceQ, Edge* _eP, Edge* _eQ,
									   const glm::mat4& _m2wP, const glm::mat4& _m2wQ,
									   const Geometry::Plane& _planeP, const Geometry::Plane& _planeQ,
									   const HalfEdge* _P, const HalfEdge* _Q)
{

	//take the MINIMUM PENETRATION (aka the BIGGEST separation value)
	if (_sepEdge < _sepPfromQ || _sepEdge < _sepQfromP)
	{
		Geometry::Plane refPlane;

		//P is reference, yields minimum translation vector
		if (_sepPfromQ > _sepQfromP)
		{
			refPlane = _planeP;
			contactNormal = refPlane.normal;
			referenceFace = _faceP;
			isPReference = true;
			//penetration = sepPfromQ;
			GetIncindentFace(_Q, _m2wP, _m2wQ);
			ClipFaces(_m2wP, _m2wQ, refPlane);
		}
		else
		{
			refPlane = _planeQ;
			contactNormal = refPlane.normal;
			referenceFace = _faceQ;
			isPReference = false;
			//penetration = sepQfromP;
			GetIncindentFace(_P, _m2wQ, _m2wP);
			ClipFaces(_m2wQ, _m2wP, refPlane);
		}
	}
	else
	{
		isEdgeCollision = true;
		isPReference = false;
		//penetration = sepEdge;
		if (_eP && _eQ)
		{
			auto a = glm::vec3(_m2wP * glm::vec4(*_eP->vertex, 1.0f));
			auto b = glm::vec3(_m2wP * glm::vec4(*_eP->next->vertex, 1.0f));

			auto c = glm::vec3(_m2wQ * glm::vec4(*_eQ->vertex, 1.0f));
			auto d = glm::vec3(_m2wQ * glm::vec4(*_eQ->next->vertex, 1.0f));

			auto closest = Geometry::ClosestSegmentSegment({ a, b }, { c, d });
			auto translationVec = closest.p1 - closest.p0;
			contactNormal = glm::normalize(translationVec);
			eP = _eP;
			eQ = _eQ;
			ContacPoint cpt;
			cpt.pt = (closest.p0 + closest.p1) / 2.0f;
			cpt.penetration = glm::length(translationVec) / 2.0f;
			contactPoints.push_back(cpt);
			s = closest;
		}
	}

}

/**
 * Computes which is the incident face
 * @param _incidentPolyTope: the polytope to retrieve the face from
 * @param _m2wReference: model 2 world matrix of reference face's object
 * @param _m2wIncident:model 2 world matrix of incident face's object
*/
void ContactManifold::GetIncindentFace(const HalfEdge* _incidentPolyTope, const glm::mat4& _m2wReference, const glm::mat4& _m2wIncident)
{
	//we're going to look for the most parallel face
	float minDot = std::numeric_limits<float>::max();
	//compute normal in wolrd
	auto nWorldRef = glm::vec3(_m2wReference * glm::vec4(referenceFace->normal, 0.0f));

	for (auto f : _incidentPolyTope->mFaces)
	{
		auto nWorldIncident = glm::vec3(_m2wIncident * glm::vec4(f->normal, 0.0f));
		float dot = glm::dot(nWorldIncident, nWorldRef);

		if (dot < minDot)
		{
			minDot = dot;
			incidentFace = f;
		}
	}
}

/**
 * Clips incident face against reference face's neighbors
 * @param _m2wReference: model 2 world matrix of reference face's object
 * @param _m2wIncident:model 2 world matrix of incident face's object
 * @param _referencePlane: reference face's plane
*/
void ContactManifold::ClipFaces(const glm::mat4& _m2wReference, const glm::mat4& _m2wIncident, const Geometry::Plane& _referencePlane)
{
	//get all face's points in world
	std::vector<glm::vec3> points =	TransformPoints(_m2wIncident, incidentFace->vertices);

	for (auto e : referenceFace->edges)
	{
		std::vector<glm::vec3> negative;
		
		//get twin face and compute its plane in world space
		auto twinFace = e->twin->face;
		auto worldNormal = glm::vec3(_m2wReference * glm::vec4(twinFace->normal, 0.0f));
		auto worldPointPlane = glm::vec3(_m2wReference * glm::vec4(*twinFace->vertices[0], 1.0f));
		Geometry::Plane p(worldPointPlane, worldNormal);

		//perform SutherlandHodgeman algorithm on top of the previous result. We start off with all
		//points in the incident face, but each time we keep only those who lie inside each plane.
		//positive ones are ignored
		SutherlandHodgeman(points, p, negative);
		points = negative;
	}

	//discard points that are outside the reference plane
	for (auto p : points)
	{
		auto classification = Geometry::ClassifyPlanePoint(_referencePlane, p, 0.0f);
		if (classification != Geometry::classification_t::outside)
		{
			ContacPoint contactPt;
			contactPt.pt = p;
			contactPt.penetration = glm::length(p - Geometry::ClosestPointPlane(p, _referencePlane));
			contactPoints.push_back(contactPt);
		}
	}
}

/**
 * Precomputes constant values for collision solving
 * @param _rbA - rigidbody A to take info from
 * @param _rbB - rigidbody B to take info from
*/
void ContactManifold::ComputeConstants(const Rigidbody* _rbA, const Rigidbody* _rbB)
{
	//we need to flip the normal in this case
	if (!isEdgeCollision && isPReference)
		contactNormal *= -1.0f;

	if (glm::all(glm::epsilonEqual(glm::abs(contactNormal), glm::vec3(0.0f, 1.0f, 0.0f), cEpsilon)))
		 uFriction = glm::normalize(glm::cross(contactNormal, glm::vec3(0.0f, 0.0f, 1.0f)));
	else uFriction = glm::normalize(glm::cross(contactNormal, glm::vec3(0.0f, 1.0f, 0.0f)));
	
	vFriction = glm::normalize(glm::cross(uFriction, contactNormal));

	for (auto& cpt : contactPoints)
	{
		//CONTACT CONSTRAINT CONSTANTS
		cpt.RAcrossN = glm::cross(cpt.pt - _rbA->mTransform.mPosition, contactNormal);
		cpt.RBcrossN = glm::cross(cpt.pt - _rbB->mTransform.mPosition, contactNormal);
		float effectiveMassContact = _rbA->mInvMass + _rbB->mInvMass;
		effectiveMassContact += glm::dot(cpt.RAcrossN, _rbA->mInertiaTensorWorldInverse * cpt.RAcrossN);
		effectiveMassContact += glm::dot(cpt.RBcrossN, _rbB->mInertiaTensorWorldInverse * cpt.RBcrossN);
		cpt.effMassContact = effectiveMassContact;

		//FRICITON CONSTRAINT CONSTANTS
		//U direction
		cpt.RAcrossU = glm::cross(cpt.pt - _rbA->mTransform.mPosition, uFriction);
		cpt.RBcrossU = glm::cross(cpt.pt - _rbB->mTransform.mPosition, uFriction);
		float effectiveMassU = _rbA->mInvMass + _rbB->mInvMass;
		effectiveMassU += glm::dot(cpt.RAcrossU, _rbA->mInertiaTensorWorldInverse * cpt.RAcrossU);
		effectiveMassU += glm::dot(cpt.RBcrossU, _rbB->mInertiaTensorWorldInverse * cpt.RBcrossU);
		cpt.effMassU = effectiveMassU;

		//v direction
		cpt.RAcrossV = glm::cross(cpt.pt - _rbA->mTransform.mPosition, vFriction);
		cpt.RBcrossV = glm::cross(cpt.pt - _rbB->mTransform.mPosition, vFriction);
		float effectiveMassV = _rbA->mInvMass + _rbB->mInvMass;
		effectiveMassV += glm::dot(cpt.RAcrossV, _rbA->mInertiaTensorWorldInverse * cpt.RAcrossV);
		effectiveMassV += glm::dot(cpt.RBcrossV, _rbB->mInertiaTensorWorldInverse * cpt.RBcrossV);
		cpt.effMassV = effectiveMassV;

		//Restitution coefficient (bias)
		float Jv = glm::dot(-contactNormal, _rbA->mVelocity) + glm::dot(-cpt.RAcrossN, _rbA->mAngularVelocity);
		Jv += glm::dot(contactNormal, _rbB->mVelocity) + glm::dot(cpt.RBcrossN, _rbB->mAngularVelocity);
		cpt.restitutionBias += Jv * PhysicsMgr.GetRestitution();
	}
	bodyA = _rbA->mID;
	bodyB = _rbB->mID;
}

/**
 * Performs the Sutherland Hodgeman clipping algorithm
 * @param _pointsToClip: points in polygon to clip
 * @param _p: plane to clip against
 * @param _referencePlane: reference face's plane
 * @param _positive: points lying outside the plane
 * @param _negative: points lying inside the plane
*/
void ContactManifold::SutherlandHodgeman(const std::vector<glm::vec3>& _pointsToClip, const Geometry::Plane& _p, std::vector<glm::vec3>& _negative) const
{
	auto count = (unsigned)_pointsToClip.size();
	using namespace Geometry;
	for (auto i = 0u; i < count; ++i)
	{
		//take current and previous vertex (if we just started looping, the prev will be the last vertex from the polygon)
		auto current = _pointsToClip[i];
		glm::vec3 prev;
		if (i == 0) prev = _pointsToClip[count - 1];
		else        prev = _pointsToClip[i - 1];

		Geometry::Segment seg = { prev, current };

		//take intersection time and corresponding classification
		float t = IntersectionSegmentPlane(seg, _p);
		auto intersection = prev + t * (current - prev);
		
		auto prevClassification = ClassifyPlanePoint(_p, prev, 0.0f);
		auto currentClassification = ClassifyPlanePoint(_p, current, 0.0f);

		//add points accordingly
		if (prevClassification == classification_t::outside &&
			currentClassification == classification_t::inside)
		{
			_negative.push_back(intersection);
			_negative.push_back(current);
		}
		else if (prevClassification == classification_t::inside &&
			currentClassification == classification_t::inside)
			_negative.push_back(current);
		else if (prevClassification == classification_t::inside &&
			currentClassification == classification_t::outside)
			_negative.push_back(intersection);
		else if (prevClassification == classification_t::overlapping && currentClassification == classification_t::overlapping)
		{
			_negative.push_back(prev);
			_negative.push_back(current);
		}
		else if (prevClassification == classification_t::overlapping)
			_negative.push_back(current);
	}
}

