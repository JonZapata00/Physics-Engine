#include "math.h"

std::ostream& operator<<(std::ostream& _os, const glm::vec3& _v)
{
	// TODO: insert return statement here
	_os << "x: " << _v.x << " y: " << _v.y << " z: " << _v.z << std::endl;
	return _os;
}

std::ostream& operator<<(std::ostream& _os, const glm::vec4& _v)
{
	// TODO: insert return statement here
	_os << "x: " << _v.x << " y: " << _v.y << " z: " << _v.z << " w: " << _v.w << std::endl;
	return _os;
}

std::ostream& operator<<(std::ostream& _os, const glm::mat3& _v)
{
	// TODO: insert return statement here
	for (int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
		_os << _v[i][j] <<" ";
		_os << std::endl;
	}
	return _os;
}

bool operator<(const glm::vec3& _a, const glm::vec3& _b)
{
	return _a.x < _b.x && _a.y < _b.y && _a.z < _b.z;
}

bool operator>(const glm::vec3& _a, const glm::vec3& _b)
{
	return !(_a < _b);
}
