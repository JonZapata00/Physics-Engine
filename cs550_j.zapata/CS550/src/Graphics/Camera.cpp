// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Camera class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#include "../Math/math.h"
#include "../Input/InputManager.h"
#include "Camera.h"

/**
  * Initializes camera by computing its vectors
*/
void Camera::Initialize()
{
    mView = glm::normalize(mTarget - mPosition);
    mRight = glm::normalize(glm::cross(mView, mUp));
    mUp = glm::normalize(glm::cross(mRight, mView));
}

/**
  * creates the window and initializes opengl, as well as creating the color shader
  * @param newPos - new position of the camera
*/
void Camera::SetPosition(const glm::vec3 & newPos) {mPosition = newPos;}

/**
  * sets a new target for the camera
  * @param newTarget - new target of the camera
*/
void Camera::SetTarget(const glm::vec3 & newTarget) {mTarget = newTarget;}

/**
  * sets perspective projection
  * @param FOV - field of view of the camera
  * @param DImensions - dimensions of the VP
  * @param near - near plane of the camera
  * @param far - far plane of the camera
*/
void Camera::SetProjection(float FOV, const glm::vec2 & Dimensions, float near, float  far)
{
    mProjection = glm::perspective(glm::radians(FOV), Dimensions.x / Dimensions.y, near, far);
}

/**
  * updates the camera matrix and movement
*/
void Camera::Update()
{

    if (MouseDown(MouseKey::RIGHT))
    {
        Rotate();
        mTarget = mPosition + mView;

        mRight = glm::normalize(glm::cross(mView, { 0, 1, 0 }));
        mUp = glm::normalize(glm::cross(mRight, mView));

        if (KeyDown(Key::LShift)) speed = 0.75f;
        if (KeyDown(Key::D)) MoveHorizontally(speed);
        if (KeyDown(Key::A)) MoveHorizontally(-speed);
        if (KeyDown(Key::W)) MoveForward(speed);
        if (KeyDown(Key::S)) MoveForward(-speed);

    }

    //we need to take into account the view matrix as well
    mW2C = glm::lookAt(mPosition, mTarget, { 0, 1, 0 });
    mCameraMatrix = mProjection * mW2C;

    mMousePos = InputManager.WindowMousePos();

}

/**
 * Allows camera to move horizontally
 * @param _factor
*/
void Camera::MoveHorizontally(float _factor)
{
    mPosition += mRight * _factor;
    mTarget += mRight * _factor;
}

/**
 * Allows camera to move vertically
 * @param _factor
*/
void Camera::MoveVertically(float _factor)
{
    mPosition += mUp * _factor;
    mTarget += mUp * _factor;
}

/**
 * Allows camera to move forward and backwards
 * @param _factor
*/
void Camera::MoveForward(float _factor)
{
    mPosition += mView * _factor;
    mTarget += mView * _factor;
}

/**
 * Allows free rotation of camera
*/
void Camera::Rotate()
{
    float dt = 0.01667f;
    auto displacement = mMousePos - InputManager.WindowMousePos();

    mView = glm::vec3(glm::vec4(mView, 0) * glm::rotate(glm::radians(5.0f) * dt * displacement.y, mRight));
    mView = glm::vec3(glm::vec4(mView, 0) * glm::rotate(glm::radians(5.0f) * dt * -displacement.x, mUp));

}

/**
  * retrieves the camera matrix
*/
glm::mat4 Camera::GetMtx() const { return mCameraMatrix; }

/**
  * gets the camera position
  * @return - camera position
 */
glm::vec3 Camera::GetPosition() const { return mPosition; }

void Camera::ToJson(nlohmann::json& j) const
{
    j["View"] << mView;
    j["Position"] << mPosition;
}

void Camera::FromJson(const nlohmann::json& j)
{
    j["View"] >> mView;
    j["Position"] >> mPosition;
    mRight = glm::normalize(glm::cross(mView, { 0, 1, 0 }));
    mUp = glm::normalize(glm::cross(mRight, mView));
    mTarget = mPosition + mView;
}

