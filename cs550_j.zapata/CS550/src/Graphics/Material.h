#include <glm/glm.hpp>
#pragma once

struct Material
{
	glm::vec3 ambient{ 0.3f };
	glm::vec3 diffuse{ 1.0f };
	glm::vec3 specular{ 1.0f };
	float shininess{ 150.0f };
};