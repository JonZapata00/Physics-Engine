#pragma once
#include "../Physics/Rigidbody.h"
#include "Singleton.h"

class Editor
{
	MAKE_SINGLETON(Editor)
public:
	void Initialize();
	void Update();
	int GetEditingBody() const { return mCurrentEditingBody; }

private:
	void SaveScene(std::string _file);
	void LoadScene(const std::string& _file);
	void MainWindow();
	void Edit() const;
	void EditBodyTransform() const;
	void HelpWindow();
	void ApplyForceOnSelectedObject(float _factor);
	float RayFromMouseToObject(const Rigidbody& _rb) const;
	void SelectBody();

	glm::vec4 ScreenToWorld(const glm::vec2& mousePos) const;
	//Convert from window coordinates to opengl coordinates
	glm::vec3 ScreenToNormalized(const glm::vec2& mousePos) const;
	//I honestly don't know what clip space is
	glm::vec4 NormalizedToClipped(const glm::vec3& mouseScreen) const;
	//Convert to camera
	glm::vec4 ProjToCam(const glm::vec4& mouseCam) const;
	//to world
	glm::vec4 CamToWorld(const glm::vec4& mouse3D) const;

	std::string mCurrentFile = "None";
	int mCurrentEditingBody = -1;
	unsigned frameToReverse = 200;
};

#define EDITOR Editor::Instance()
