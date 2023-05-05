// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the Transform3D class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#pragma once
#include "math.h"//glm vectors and matrices

class Transform3D
{
public:
	Transform3D();
	Transform3D(const glm::vec3& _pos, const glm::mat3& _orientation, const glm::vec3& _scale = glm::vec3(1.0f));
	Transform3D(const glm::vec3& _pos, const glm::vec3& _orientation, const glm::vec3& _scale = glm::vec3(1.0f));
	Transform3D(const glm::vec3& _pos, const glm::quat& _orientation, const glm::vec3& _scale = glm::vec3(1.0f));
	Transform3D& operator= (const Transform3D& _rhs);

	glm::mat4 GetModelToWorld() const;
	glm::mat4 GetRotationMatrix4x4() const;
	void SetRotationMatrix(const glm::vec3& _orientation);

	glm::quat mRotation{};
	glm::mat3 mRotationMatrix{};
	glm::vec3 mPosition{};
	glm::vec3 mScale{1.0f};
};
