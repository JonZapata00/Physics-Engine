// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the Camera class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#pragma once

#include <glm/glm.hpp>
#include "../Utilities/jsonWrapper.h"
class Camera
{
    public:
    void Initialize();
    void SetPosition(const glm::vec3 & newPos);
    void SetTarget(const glm::vec3 & newTarget);
    void SetProjection(float FOV, const glm::vec2 & Dimensions, float near, float far);
    void Update();
    void MoveHorizontally(float _factor);
    void MoveVertically(float _factor);
    void MoveForward(float _factor);
    void Rotate();

    glm::mat4 GetMtx() const;
    glm::mat4 GetProj() const { return mProjection; }
    glm::mat4 GetView() const { return mW2C;}
    glm::vec3 GetPosition() const;
    glm::vec3 GetDirection() const { return mView; }
    glm::vec3 GetTarget() const { return mTarget; }

    friend json& operator<<(nlohmann::json& j, const Camera& val) { val.ToJson(j); return j; }
    friend void operator>>(const nlohmann::json& j, Camera& val) { val.FromJson(j); }

    void ToJson(nlohmann::json& j) const;
    void FromJson(const nlohmann::json& j);

    void SetSpeed(float _s) { speed = _s; }

private:
    glm::mat4 mProjection = glm::mat4();
    glm::mat4 mCameraMatrix = glm::mat4();
    glm::mat4 mW2C = glm::mat4();
    glm::vec3 mPosition = { 0, 0, 10 };
    glm::vec3 mTarget = { 0, 0, 0 };
    glm::vec3 mView = {0, 0, -1};
    glm::vec3 mUp = { 0, 1, 0 };
    glm::vec3 mRight = { 1, 0, 0 };
    glm::vec2 mMousePos = {0, 0};
    float speed = 0.3f;
};
