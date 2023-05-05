// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the Shader class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#pragma once

#include <string>
#include <glm/glm.hpp>

class Shader
{
public:
    void GenerateShaderProgram(const std::string& vertShader, const std::string& fragShader);
    void CompileShader(const char* vertShaderCode, const char* fragShaderCode);
    unsigned GetProgramID() const;
    void Use();

    Shader(const std::string& vertShader, const std::string& fragShader);

    void SetUniform(const std::string& name, float x, float y, float z) const;
    void SetUniform(const std::string& name, const glm::vec2& v) const;
    void SetUniform(const std::string& name, const glm::vec3& v) const;
    void SetUniform(const std::string& name, const glm::vec4& v) const;
    void SetUniform(const std::string& name, const glm::mat4& m) const;
    void SetUniform(const std::string& name, const glm::mat3& m) const;
    void SetUniform(const std::string& name, float val) const;
    void SetUniform(const std::string& name, int val) const;
    void SetUniform(const std::string& name, bool val) const;

    Shader();
    ~Shader();

private:
    int  GetUniformLocation(const std::string& name) const;
    int ID;
};
