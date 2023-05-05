// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of several json related function
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#pragma once
#include <fstream>
#include "../Math/math.h"
#include "Singleton.h"
#include "json.hpp"

//using json = nlohmann::json;

typedef nlohmann::json json;

// ------------------------------ JSON WRITE ------------------------------
inline json& operator<<(json& j, const int& val) { j = val; return j; }
inline json& operator<<(json& j, const unsigned& val) { j = val; return j; }
inline json& operator<<(json& j, const float& val) { j = val; return j; }
inline json& operator<<(json& j, const double& val) { j = val; return j; }
inline json& operator<<(json& j, const unsigned short& val) { j = val; return j; }
inline json& operator<<(json& j, const bool& val) { j = val; return j; }
inline json& operator<<(json& j, const std::string& val) { j = val.c_str(); return j; }
inline json& operator<<(json& j, const glm::ivec2& val) { j["x"] << val.x; j["y"] << val.y; return j; }
inline json& operator<<(json& j, const glm::vec2& val) { j["x"] << val.x; j["y"] << val.y; return j; }
inline json& operator<<(json& j, const glm::vec3& val) { j["x"] << val.x; j["y"] << val.y; j["z"] << val.z; return j; }
inline json& operator<<(json& j, const glm::vec4& val) { j["x"] << val.x; j["y"] << val.y; j["z"] << val.z; j["w"] << val.w; return j; }
inline json& operator<<(json& j, const glm::quat& val) { j["x"] << val.x; j["y"] << val.y; j["z"] << val.z; j["w"] << val.w; return j; }
inline json& operator<<(json& j, const glm::mat3& val) { j["Row 1"] << val[0]; j["Row 2"] << val[1]; j["Row 3"] << val[2]; return j; }

// ------------------------------ JSON READ ------------------------------
inline int& operator>>(const json& j, int& val) { val = j; return val; }
inline unsigned& operator>>(const json& j, unsigned& val) { val = j; return val; }
inline float& operator>>(const json& j, float& val) { val = j; return val; }
inline double& operator>>(const json& j, double& val) { val = j; return val; }
inline unsigned short& operator>>(const json& j, unsigned short& val) { val = j; return val; }
inline bool& operator>>(const json& j, bool& val) { val = j; return val; }
inline std::string& operator>>(const json& j, std::string& val) { val = j.get<std::string>(); return val; }
inline glm::ivec2& operator>>(const json& j, glm::ivec2& val) { j["x"] >> val.x; j["y"] >> val.y; return val; }
inline glm::vec2& operator>>(const json& j, glm::vec2& val) { j["x"] >> val.x; j["y"] >> val.y; return val; }
inline glm::vec3& operator>>(const json& j, glm::vec3& val) { j["x"] >> val.x; j["y"] >> val.y; j["z"] >> val.z; return val; }
inline glm::vec4& operator>>(const json& j, glm::vec4& val) { j["x"] >> val.x; j["y"] >> val.y; j["z"] >> val.z; j["w"] >> val.w; return val; }
inline glm::quat& operator>>(const json& j, glm::quat& val) { j["x"] >> val.x; j["y"] >> val.y; j["z"] >> val.z; j["w"] >> val.w; return val; }
inline glm::mat3& operator>>(const json& j, glm::mat3& val) { j["Row 1"] >> val[0]; j["Row 2"] >> val[1]; j["Row 3"] >> val[2]; return val; }

//Class to read and write json files
class JsonOutputFormatter
{
	MAKE_SINGLETON(JsonOutputFormatter)
public:
	void Write(const std::string& _path, const json& _j) const
	{
		std::string fullPath = "Resources/Scenes/" + _path;
		if(fullPath.find(".") == std::string::npos) fullPath += ".json";
		std::ofstream outFile(fullPath.c_str());
		if (outFile.good() && outFile.is_open())
		{
			outFile << std::setw(4) << _j;
			outFile.close();
		}
	}
	json Read(const std::string& _path) const
	{
		std::ifstream inFile(_path.c_str());

		json j;

		if (inFile.good() && inFile.is_open()) {
			inFile >> j;
			inFile.close();
		}
		return j;
	}
};