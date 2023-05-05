// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Light class
//	Project:		cs300_j.zapata_1
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include "Light.h" //class declaration

//initializing the static variable
LightType Light::mType = LightType::POINT;


Light::Light()
{
	lightPosition = glm::vec3(20.0f, 20.0f, 0.0f);

	//initializing our variables
	lightVector = -lightPosition;
	LightColor = glm::vec3(1, 1, 1);
	ambientColor = glm::vec3(1.0);
	diffuseColor = glm::vec3(0.8, 0.8, 0.8);
	specularColor = glm::vec3(1, 1, 1);
}

// ------------------------------------------------------------------------
// Update: Updates the light's position 
void Light::Update()
{
	lightVector = -lightPosition;
}


