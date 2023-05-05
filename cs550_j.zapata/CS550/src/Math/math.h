/**
  * @file demo_renderer.hpp
  * @author Eder Beldad, eder.bl@digipen .edu
  * @date 2020/09/21
  * @brief useful includes for math
  * @copyright Copyright (C) 2020 DigiPen Institute of Technology
*/
#pragma once
#pragma once

#define GLM_FORCE_INLINE
#define GLM_FORCE_NO_CTOR_INIT
#define GLM_FORCE_EXPLICIT_CTOR
#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_XYZW_ONLY
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/gtx/integer.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtc/random.hpp>

#include "geometry.h"
#include "Transform3D.h"

constexpr float cEpsilon = 1e-6f;

#include <iostream>

std::ostream& operator<< (std::ostream& _os, const glm::vec3& _v);
std::ostream& operator<< (std::ostream& _os, const glm::vec4& _v);
std::ostream& operator<< (std::ostream& _os, const glm::mat3& _v);

bool operator< (const glm::vec3& _a, const glm::vec3& _b);
bool operator> (const glm::vec3& _a, const glm::vec3& _b);
