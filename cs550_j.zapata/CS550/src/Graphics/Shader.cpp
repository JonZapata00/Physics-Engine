// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Shader class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#include "../Utilities/pch.hpp"
#include <sys/stat.h>
#include <GL/glew.h>
#include <GL/GL.h>
#include "Shader.h"


/**
 * Generates a shader program. This was done following learnopengl.
 * @param vertShader - the vertexShader to create
 * @param fragShader - the fragment shader to create
*/
void Shader::GenerateShaderProgram(const std::string& vertShader, const std::string& fragShader)
{
    std::string vertexCode;
    std::string fragmentCode;
    std::ifstream vShaderFile;
    std::ifstream fShaderFile;
    // ensure ifstream objects can throw exceptions:
    vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
    {
        // open files
        vShaderFile.open(vertShader);
        fShaderFile.open(fragShader);
        std::stringstream vShaderStream, fShaderStream;
        // read file's buffer contents into streams
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();
        // close file handlers
        vShaderFile.close();
        fShaderFile.close();
        // convert stream into string
        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();
    }
    catch (std::ifstream::failure& )
    {
        std::cout << "ERROR: Shader file not successfully read" << std::endl;
    }
     CompileShader(vertexCode.c_str(), fragmentCode.c_str());
   
}

/**
 * Compiles the shaders
 * @param vertShaderCodee - the vertexShader code to compile
 * @param fragShaderCodee - the fragment shader code to compile
*/
void Shader::CompileShader(const char * vertShaderCode, const char* fragShaderCode)
{
    unsigned vertex, fragment;
    // creating and compiling vertex shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vertShaderCode, NULL);
    glCompileShader(vertex);

    // creating and compiling Shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fragShaderCode, NULL);
    glCompileShader(fragment);

    //finally, generating the full program
    ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);

    // delete the shaders as they're linked into our program now and no longer necessary
    glDeleteShader(vertex);
    glDeleteShader(fragment);
}

/**
 * Default constructor
*/
Shader::Shader() : ID(0) {}

/**
 * Deletes the program, if any
*/
Shader::~Shader() {if (ID > 0) glDeleteProgram(ID);}

/**
 * returns the program's ID
 * @return The ID of the program
*/
unsigned Shader::GetProgramID() const { return ID; }

void Shader::Use()
{
    glUseProgram((GLint)ID);
}

Shader::Shader(const std::string& vertShader, const std::string& fragShader)
{
    GenerateShaderProgram(vertShader, fragShader);
}

void Shader::SetUniform(const std::string& name, float x, float y, float z) const
{
    int loc = GetUniformLocation(name);

    if (loc >= 0)
    {
        glUniform3f(loc, x, y, z);
    }
    else
    {
        std::cout << "Uniform: " << name << " not found." << std::endl;
    }
}

void Shader::SetUniform(const std::string& name, const glm::vec2& v) const
{
    int loc = GetUniformLocation(name);

    if (loc >= 0)
    {
        glUniform2f(loc, v.x, v.y);
    }
    else
    {
        std::cout << "Uniform: " << name << " not found." << std::endl;
    }
}

void Shader::SetUniform(const std::string& name, const glm::vec3& v) const
{
    this->SetUniform(name, v.x, v.y, v.z);
}

void Shader::SetUniform(const std::string& name, const glm::vec4& v) const
{
    int loc = GetUniformLocation(name);

    if (loc >= 0)
    {
        glUniform4f(loc, v.x, v.y, v.z, v.w);
    }
    else
    {
        std::cout << "Uniform: " << name << " not found." << std::endl;
    }
}

void Shader::SetUniform(const std::string& name, const glm::mat4& m) const
{
    int loc = GetUniformLocation(name);
    if (loc >= 0)
    {
        glUniformMatrix4fv(loc, 1, GL_FALSE, &m[0][0]);
    }
    else
    {
        std::cout << "Uniform: " << name << " not found." << std::endl;
    }
}

void Shader::SetUniform(const std::string& name, const glm::mat3& m) const
{
    int loc = GetUniformLocation(name);

    if (loc >= 0)
    {
        glUniformMatrix3fv(loc, 1, GL_FALSE, &m[0][0]);
    }
    else
    {
        std::cout << "Uniform: " << name << " not found." << std::endl;
    }
}

void Shader::SetUniform(const std::string& name, float val) const
{
    int loc = GetUniformLocation(name);

    if (loc >= 0)
    {
        glUniform1f(loc, val);
    }
    else
    {
        std::cout << "Uniform: " << name << " not found." << std::endl;
    }
}

void Shader::SetUniform(const std::string& name, int val) const
{
    int loc = GetUniformLocation(name);

    if (loc >= 0)
    {
        glUniform1i(loc, val);
    }
    else
    {
        std::cout << "Uniform: " << name << " not found." << std::endl;
    }
}

void Shader::SetUniform(const std::string& name, bool val) const
{
    int loc = GetUniformLocation(name);

    if (loc >= 0)
    {
        glUniform1i(loc, val);
    }
    else
    {
        std::cout << "Uniform: " << name << " not found." << std::endl;
    }
}

int Shader::GetUniformLocation(const std::string& name) const
{
    return glGetUniformLocation(ID, name.c_str());
}
