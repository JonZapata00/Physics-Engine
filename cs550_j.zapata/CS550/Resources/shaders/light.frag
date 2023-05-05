#version 440 core
layout(location = 1) uniform vec4 uniform_color;

out vec4 out_color;

struct Light 
{
	vec3 lightVector;
	vec3 ambientColor;
	vec3 diffuseColor;
	vec3 specularColor;
	int type;
};

struct Material
{
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

uniform Light light;
uniform Material material;

in mat4 W2C;
in vec3 Normal;
in vec3 FragPos;

vec3 ComputeLightColor();

void main()
{
	out_color = vec4(ComputeLightColor(), 1.0f) * uniform_color;
}

vec3 ComputeLightColor()
{
	//first convert everything to camera space
	vec3 lightVec = vec3(W2C * vec4(light.lightVector, 0.0));
	vec3 normal = normalize(Normal);
	vec3 lightDir = vec3(0.0);

	//texture color to apply with diffuse and ambient light
	vec3 textureColor;
	textureColor = vec3(1.0f);

	float attenuation = 1.0;
	float spotLightEffect = 1.0;

	//light direction for directional light
	lightDir = normalize(-lightVec);
	
	//vector from camera pos ((0,0,0) since we are in cam space)
	vec3 viewDir = normalize(-FragPos);
	//and we get the reflection vector as learned in class
	vec3 refVector = 2.0 * dot(normal, lightDir) * normal - lightDir;  

	//ambient light is always the same
	vec3 ambientLight =  light.ambientColor * material.ambient * textureColor;

	//diffuse light according to the light direction
	float diff = max(dot(normal, lightDir), 0.0);
	vec3 diffuseLight = material.diffuse * light.diffuseColor * diff * textureColor;

	//and finally specular considering the reflection vector
	float specular = pow( max( dot(viewDir, refVector), 0.0), material.shininess);
	vec3 specularLight =  material.specular * light.specularColor * specular; 

	//multiplying approprietly
	ambientLight *= attenuation;
	diffuseLight *= attenuation;
	specularLight *= attenuation;

	diffuseLight *= spotLightEffect;
	specularLight *= spotLightEffect;

	vec3 result = diffuseLight + ambientLight + specularLight;

	return result;
}