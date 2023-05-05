#version 430 core
layout(location = 0) in vec3 attr_position;

layout(location = 0) uniform mat4 uniform_mvp;

out vec3 FragPos;
void main()
{	
	//this is an optimization
	vec4 pos = uniform_mvp * vec4(attr_position, 1.0);
	gl_Position = pos.xyww;
	FragPos = attr_position;
}