#version 440 core

layout(location = 0) in vec3 attr_position;
layout(location = 1) in vec3 normal;

layout(location = 0) uniform mat4 uniform_mvp;
uniform mat4 Model2World;
uniform mat4 World2Cam;

out mat4 W2C;
out vec3 FragPos;
out vec3 Normal;

void main()
{
    vec4 vertex = vec4(attr_position, 1.0f);
    FragPos = vec3(World2Cam * Model2World * vec4(attr_position, 1.0));
    gl_Position = uniform_mvp * vertex;
    Normal = mat3(transpose(inverse(World2Cam * Model2World))) * normal;
    W2C = World2Cam;
}