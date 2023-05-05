#pragma once

#include <glm\glm.hpp>

enum LightType { POINT, SPOT, DIRECTIONAL };
struct Light
{
	Light();
	void Update();

	//all variables that will get passed to the shader
	glm::vec3 lightPosition;
	glm::vec3 lightVector;
	glm::vec3 LightColor;
	glm::vec3 ambientColor;
	glm::vec3 diffuseColor;
	glm::vec3 specularColor;
	float attenuation_1; //constant
	float attenuation_2; //linear
	float attenuation_3; //quadratic 
	float innerCosCutOff;
	float outerCosCutOff;
	float fallOff;
	static LightType mType;


	//for the movement
	float horizontalAngle;
	float verticalAngle;
	float random;

};
