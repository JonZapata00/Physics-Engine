// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the transform class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Transform3D.h"


/**
 * Default constructor
*/
Transform3D::Transform3D() : mPosition(glm::vec3(0, 0, 0))
{
	glm::mat4 rotZ = glm::rotate(0.0f, glm::vec3(0, 0, 1));
	glm::mat4 rotX = glm::rotate(0.0f, glm::vec3(1, 0, 0));
	glm::mat4 rotY = glm::rotate(0.0f, glm::vec3(0, 1, 0));

	mRotationMatrix = rotZ * rotX * rotY;
	mRotation = glm::toQuat(mRotationMatrix);
}

/**
 * Custom constructor
 * @param _pos - position
 * @param _orientation - rotation in matrix form
*/
Transform3D::Transform3D(const glm::vec3& _pos, const glm::mat3& _orientation, const glm::vec3& _scale)
{
	mPosition = _pos;
	mRotationMatrix = _orientation;
	mRotation = glm::normalize(glm::toQuat(mRotationMatrix));
	mScale = _scale;
}

/**
 * Custom constructor
 * @param _pos - position
 * @param _orientation - rotation in vector form (Euler angles)
*/
Transform3D::Transform3D(const glm::vec3& _pos, const glm::vec3& _orientation, const glm::vec3& _scale)
{
	mPosition = _pos;
	glm::mat4 rotZ = glm::rotate(_orientation.z, glm::vec3(0, 0, 1));
	glm::mat4 rotX = glm::rotate(_orientation.x, glm::vec3(1, 0, 0));
	glm::mat4 rotY = glm::rotate(_orientation.y, glm::vec3(0, 1, 0));

	mRotationMatrix = rotZ * rotX * rotY;
	mRotation = glm::normalize(glm::toQuat(mRotationMatrix));
	mScale = _scale;
}

/**
 * Custom constructor
 * @param _pos - position
 * @param _orientation - rotation in quaternion form
*/
Transform3D::Transform3D(const glm::vec3& _pos, const glm::quat& _orientation, const glm::vec3& _scale)
{
	mPosition = _pos;
	mRotation = _orientation;
	mRotationMatrix = glm::toMat3(mRotation);
	mScale = _scale;
}

Transform3D& Transform3D::operator=(const Transform3D& _rhs)
{
	if (this == &_rhs) return *this;

	mPosition = _rhs.mPosition;
	mRotation = _rhs.mRotation;
	mScale = _rhs.mScale;

	return *this;
}

// ------------------------------------------------------------------------
// GetModelToWorld: Constructs and returns model to world matrix
glm::mat4 Transform3D::GetModelToWorld() const
{
	glm::mat4 trans = glm::translate(mPosition);

	glm::mat4 s = glm::scale(mScale);

	return trans * GetRotationMatrix4x4() * s;
}

/**
 * Retrieves the Rotation matrix as a 4x4 (to render)
*/
glm::mat4 Transform3D::GetRotationMatrix4x4() const
{
	return glm::toMat4(mRotation);
	glm::mat4 r;
	r[0] = glm::vec4(mRotationMatrix[0], 0.0f);
	r[1] = glm::vec4(mRotationMatrix[1], 0.0f);
	r[2] = glm::vec4(mRotationMatrix[2], 0.0f);
	r[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
	return r;
}

/**
 * Given a set of Euler angles, computes the new rotation matrix
 * @param _orientation - the new orientation (in Euler angles)
*/
void Transform3D::SetRotationMatrix(const glm::vec3& _orientation)
{
	glm::mat4 rotZ = glm::rotate(_orientation.z, glm::vec3(0, 0, 1));
	glm::mat4 rotX = glm::rotate(_orientation.x, glm::vec3(1, 0, 0));
	glm::mat4 rotY = glm::rotate(_orientation.y, glm::vec3(0, 1, 0));

	mRotationMatrix = rotZ * rotX * rotY;
	mRotation = glm::normalize(glm::toQuat(mRotationMatrix));
}
