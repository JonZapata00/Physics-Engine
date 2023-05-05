// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Rigidbody class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include "../Math/math.h"
#include "Rigidbody.h"
#include "PhysicsManager.h"
#include "../Graphics/RenderManager.h"

static inline void Subexpressions(float w0, float w1, float w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2)
{
	auto temp0 = w0 + w1; auto temp1 = w0 * w0; auto temp2 = temp1 + w1 * temp0;
	f1 = temp0 + w2;
	f2 = temp2 + w2 * f1;
	f3 = w0 * temp1 + w1 * temp2 + w2 * f2;

	g0 = f2 + w0 * (f1 + w0);
	g1 = f2 + w1 * (f1 + w1);
	g2 = f2 + w2 * (f1 + w2);
}


Rigidbody::Rigidbody()
{
	//mMesh = new Mesh();
	mCollider = new HalfEdge();
}

/**
 * Destructor for rigidboy
*/
Rigidbody::~Rigidbody()
{
}

/**
 * Operator= for debug
 * @param _rhs
 * @return - this
*/
Rigidbody& Rigidbody::operator=(const Rigidbody& _rhs)
{
	if (this == &_rhs) return *this;

	mTransform			= _rhs.mTransform;
	mInertiaTensorWorld = _rhs.mInertiaTensorWorld;
	mInertiaTensorLocal = _rhs.mInertiaTensorLocal;
	mVelocity			= _rhs.mVelocity;
	mLinearMomentum		= _rhs.mLinearMomentum;
	mAngularMomentum	= _rhs.mAngularMomentum;
	mAngularVelocity	= _rhs.mAngularVelocity;
	mMass				= _rhs.mMass;
	mInvMass			= _rhs.mInvMass;

	*mCollider			= *_rhs.mCollider;

	return *this;
}

/**
 * Initializes Rigidbody
*/
void Rigidbody::Initialize()
{
	mCollider->Initialize(GfxManager.GetMeshes()[mID]->GetMeshData());
	ComputeInertiaTensor(GfxManager.GetMeshes()[mID]->GetTriangles());

	mTransform.mRotationMatrix = glm::toMat3(mTransform.mRotation);
	mInertiaTensorLocalInverse = glm::inverse(mInertiaTensorLocal);
	mInvMass = 1.0f / mMass;

	if (mMass < cEpsilon)
	{
		mInvMass = 0.0f;
		mInertiaTensorLocalInverse = glm::mat3();
	}

}

/**
 * Performs integration to produce rigidbody motion
*/
void Rigidbody::Update()
{
	if (mbApplyGravity) ApplyForce(mMass * glm::vec3( 0.0f, -9.81f, 0.0f));

	Integrate(PhysicsMgr.GetDt());
}

/**
 * Clears memory
*/
void Rigidbody::Shutdown()
{
	delete mCollider;
	mCollider = nullptr;
}

/**
 * Performs Euler integration
 * @param dt
*/
void Rigidbody::Integrate(float dt)
{
	if (!mbApplyForces) return;

	auto& P			= mLinearMomentum;
	auto& L			= mAngularMomentum;
	auto& v			= mVelocity;
	auto& w			= mAngularVelocity;
	auto& pos		= mTransform.mPosition;
	auto& ILocal	= mInertiaTensorLocal;
	auto& ILocalInv = mInertiaTensorLocalInverse;
	auto& IWorld	= mInertiaTensorWorld;
	auto& IWorldInv = mInertiaTensorWorldInverse;
	auto& R			= mTransform.mRotation;
	auto& RMat		= mTransform.mRotationMatrix;

	//Compute possible alterations in momenta (they will change if 
	//external forces are applied). 
	//we integrate based on the fact that dp/dt = F = ma and dL/dt = T = Ialpha
	P += mForce * dt;
	L += mTorque * dt;

	//compute velocity based on the linear momentum (p = mv)
	v = P * mInvMass;

	auto rotationTranspose = glm::transpose(RMat);
	//compute the inertia tensor in world space (R * Ilocal * R^t)
	IWorld = RMat * ILocal * rotationTranspose;
	IWorldInv = RMat * ILocalInv * rotationTranspose;

	//compute new angular velocity as I^-1 * L
	w = IWorldInv * L;
	//finally, new rotation as dq/dt = 0.5f * angularVel * q * dt (NOTE: Quaternion multiplication is NOT commutative)
	R += 0.5f * glm::quat(0.0f, w[0], w[1], w[2]) * R * dt;
	
	R = glm::normalize(R);
	RMat = glm::toMat3(R);
	
	//finally, new rotation matrix as dR/dt = w * R, note * denotes a different product (with the skew matrix of omega)
	//mTransform.mRotationMatrix += ComputeOmegaMatrix() * mTransform.mRotationMatrix * dt;

	//update position
	if(!mbLockPosition)
		pos += v * dt;

	//clear the forces
	ClearForces();
}

/**
 * Applies a force at a given force
 * @param _force - the force that will be applied
 * @param _point - the point at which it is applied
*/
void Rigidbody::ApplyForceAndTorque(const glm::vec3& _force, const glm::vec3& _point)
{
	if (!mbApplyForces) return;
	mForce += _force;
	mTorque += glm::cross(_point - mTransform.mPosition, _force);
}

void Rigidbody::ApplyForce(const glm::vec3& _force)
{
	if (!mbApplyForces) return;
	mForce += _force;
}

void Rigidbody::ComputeInertiaTensor(const std::vector<Geometry::Triangle>& _triangles)
{
	//these coefficients are used to compute the result of the surface integrals for each of the polynomials:
	//x, x^2, x^3, y^2, y^3, z^2, z^3, x^2*y, y^2*z, z^2*x, which yield:
	// 1, x, x^2, y, y^2, z, z^2, x*y, y*z, z*x. 
	//NOTE: Keep in mind that we are solving the INNER part of the integral. There is also an external part
	//that affects these coefficients!.
	static const float coefficients[] = { 1.0f / 6.0f, 1.0f / 24.0f, 1.0f / 24.0f , 1.0f / 24.0f, 1.0f / 60.0f,
										  1.0f / 60.0f, 1.0f / 60.0f, 1.0f / 120.0f, 1.0f / 120.0f, 1.0f / 120.0f };

	float integrals[10] = { 0.0f };

	for (auto i = 0u; i < _triangles.size(); ++i)
	{
		float f1x, f2x, f3x, g0x, g1x, g2x;
		float f1y, f2y, f3y, g0y, g1y, g2y;
		float f1z, f2z, f3z, g0z, g1z, g2z;

		auto tr = _triangles[i];
		auto normal = glm::cross(tr[1] - tr[0], tr[2] - tr[0]);
		Subexpressions(tr[0].x, tr[1].x, tr[2].x, f1x, f2x, f3x, g0x, g1x, g2x);
		Subexpressions(tr[0].y, tr[1].y, tr[2].y, f1y, f2y, f3y, g0y, g1y, g2y);
		Subexpressions(tr[0].z, tr[1].z, tr[2].z, f1z, f2z, f3z, g0z, g1z, g2z);

		integrals[0] += normal.x * f1x;
		integrals[1] += normal.x * f2x;
		integrals[2] += normal.y * f2y;
		integrals[3] += normal.z * f2z;
		integrals[4] += normal.x * f3x;
		integrals[5] += normal.y * f3y;
		integrals[6] += normal.z * f3z;
		integrals[7] += normal.x * (tr[0].y * g0x + tr[1].y * g1x + tr[2].y * g2x);
		integrals[8] += normal.y * (tr[0].z * g0y + tr[1].z * g1y + tr[2].z * g2y);
		integrals[9] += normal.z * (tr[0].x * g0z + tr[1].x * g1z + tr[2].x * g2z);
	}

	for (int i = 0; i < 10; ++i)
		integrals[i] *= coefficients[i];

	//polynomial p = 1 is the mass!!
	float mass = integrals[0];
	CM.x = integrals[1] * 1.0f / mass;
	CM.y = integrals[2] * 1.0f / mass;
	CM.z = integrals[3] * 1.0f / mass;

	glm::mat3 inertia{};

	inertia[0][0] = integrals[5] + integrals[6];
	inertia[0][0] -= mass * (CM.y * CM.y + CM.z * CM.z);
	inertia[1][1] = integrals[4] + integrals[6];
	inertia[1][1] -= mass * (CM.x * CM.x + CM.z * CM.z);
	inertia[2][2] = integrals[4] + integrals[5];
	inertia[2][2] -= mass * (CM.y * CM.y + CM.x * CM.x);

	inertia[0][1] = -(integrals[7] - mass * CM.x * CM.y);
	inertia[1][2] = -(integrals[8] - mass * CM.z * CM.y);
	inertia[0][2] = -(integrals[9] - mass * CM.x * CM.z);

	mInertiaTensorLocal = inertia;
	mInertiaTensorLocalInverse = glm::inverse(mInertiaTensorLocal);
	ClearForces();
}

void Rigidbody::ClearForces()
{
	mForce = glm::vec3(0.0f);
	mTorque = glm::vec3(0.0f);
}

void Rigidbody::SetLinearVelocity(const glm::vec3& _vel)
{
	mVelocity = _vel;
	mLinearMomentum = mMass * mVelocity;
}

void Rigidbody::SetLinearMomentum(const glm::vec3& _mom)
{
	mLinearMomentum = _mom;
	mVelocity = mLinearMomentum * mInvMass;
}

void Rigidbody::SetAngularVelocity(const glm::vec3& _angVel)
{
	mAngularVelocity = _angVel;
	mAngularMomentum = mInertiaTensorWorld * mAngularVelocity;
}

void Rigidbody::SetAngularMomentum(const glm::vec3& _angMom)
{
	mAngularMomentum = _angMom;
	mAngularVelocity = mInertiaTensorWorldInverse * mAngularMomentum;
}

/**
 * RigidBody serialization, saves information
 * @param j - json value to store the information
*/
void Rigidbody::ToJson(nlohmann::json& j) const
{
	j["Position"] << mTransform.mPosition;
	j["Rotation"] << mTransform.mRotation;
	j["Scale"] << mTransform.mScale;
	j["Inertia Tensor (local)"] << mInertiaTensorLocal;
	j["Inertia Tensor Inverse (local)"] << mInertiaTensorLocalInverse;
	j["Inertia Tensor (world)"] << mInertiaTensorWorld;
	j["Inertia Tensor Inverse (world)"] << mInertiaTensorWorldInverse;
	j["Linear Velocity"] << mVelocity;
	j["Linear Momentum"] << mLinearMomentum;
	j["Angular Velocity"] << mAngularVelocity;
	j["Angular Momentum"] << mAngularMomentum;
	j["Accumulated Force"] << mForce;
	j["Accumulated Torque"] << mTorque;
	j["Mass"] << mMass;
	j["Friction Coefficient"] << mFrictionCoeff;
	j["ID"] << mID;
	j["Apply physics"] << mbApplyForces;
	j["Lock position"] << mbLockPosition;
	j["Apply Gravity"] << mbApplyGravity;
}

/**
 * RigidBody serialization, reads information
 * @param j - json value to read the information from
*/
void Rigidbody::FromJson(const nlohmann::json& j)
{
	j["Position"] >> mTransform.mPosition;
	j["Rotation"] >> mTransform.mRotation;
	j["Scale"] >> mTransform.mScale;
	j["Inertia Tensor (local)"] >> mInertiaTensorLocal;
	j["Inertia Tensor Inverse (local)"] >> mInertiaTensorLocalInverse;
	j["Inertia Tensor (world)"] >> mInertiaTensorWorld;
	j["Inertia Tensor Inverse (world)"] >> mInertiaTensorWorldInverse;
	j["Linear Velocity"] >> mVelocity;
	j["Linear Momentum"] >> mLinearMomentum;
	j["Angular Velocity"] >> mAngularVelocity;
	j["Angular Momentum"] >> mAngularMomentum;
	j["Accumulated Force"] >> mForce;
	j["Accumulated Torque"] >> mTorque;
	j["Mass"] >> mMass;
	j["Friction Coefficient"] >> mFrictionCoeff;
	j["ID"] >> mID;
	j["Apply physics"] >> mbApplyForces;
	j["Lock position"] >> mbLockPosition;
	j["Apply Gravity"] >> mbApplyGravity;
}

/**
 *	Computes skew matrix using the angular velocity
*/
glm::mat3 Rigidbody::ComputeOmegaMatrix() const
{
	glm::mat3 result{};
	result[0][1] = -mAngularVelocity[2];
	result[0][2] =  mAngularVelocity[1];

	result[1][0] =  mAngularVelocity[2];
	result[1][2] = -mAngularVelocity[0];

	result[2][0] = -mAngularVelocity[1];
	result[2][1] =  mAngularVelocity[0];

	return result;
}
