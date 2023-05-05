#version 430 core

uniform samplerCube cubeMap;

in vec3 FragPos;

out vec4 FragColor;

void main()
{     
	//simply sample the cubemap using the position in model space
     FragColor = texture(cubeMap, FragPos).rgba;
}