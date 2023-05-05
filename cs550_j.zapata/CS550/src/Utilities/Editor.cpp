// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Editor class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#include <filesystem>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_internal.h"
#include "../ImGui/ImGuizmo.h"
#include "../Graphics/DebugRenderer.h"
#include "../Graphics/RenderManager.h"
#include "../Physics/PhysicsManager.h"
#include "../Input/InputManager.h"
#include "../Utilities/FrameRateController.h"
#include "../input/InputManager.h"
#include "Editor.h"

/**
 * Initializes the editor
*/
void Editor::Initialize()
{
	mCurrentEditingBody = -1;
}

/**
 * Updates editor
*/
void Editor::Update()
{
	if (ImGui::BeginMainMenuBar())
	{
		ImGui::Text("CS550 project");
		ImGui::EndMainMenuBar();
	}

	MainWindow();
	HelpWindow();
	SelectBody();
	Edit();

	if (!ImGui::IsAnyItemHovered() && KeyTriggered(Key::F))
		ApplyForceOnSelectedObject(7.0f);
	
}

/**
 * Saves Scene to a file
 * @param _file - the filename to save
*/
void Editor::SaveScene(std::string _file)
{
	if (_file.empty()) _file = mCurrentFile;
	else			   mCurrentFile = _file + ".json";

	json j;
	PhysicsMgr.SaveScene(j);
	GfxManager.SaveScene(j);
	JsonOutputFormatter::Instance().Write(_file, j);
}

/**
 * Loads Scene from a file
 * @param _file - the file to load
*/
void Editor::LoadScene(const std::string& _file)
{
	auto j = JsonOutputFormatter::Instance().Read(_file);
	GfxManager.LoadScene(j);
	PhysicsMgr.LoadScene(j);
	mCurrentFile = _file;
	mCurrentFile.erase(0, std::string("Resources/Scenes/").size());
}

/**
 * Displays the main editing window (ImGui)
*/
void Editor::MainWindow()
{
	if (KeyTriggered(Key::P))
		PhysicsMgr.GetUpdatePhysics() = !PhysicsMgr.GetUpdatePhysics();
	if (ImGui::Begin("Physics Engine"))
	{
		ImGui::Separator();

		static float camSpeed = 0.3f;
		if (ImGui::SliderFloat("Camera speed", &camSpeed, 0.02f, 0.75f))
			GfxManager.GetCamera().SetSpeed(camSpeed);

		ImGui::Separator();
		ImGui::Text("----- PHYSICS -----");
		if (ImGui::Checkbox("Update Physics", &PhysicsMgr.GetUpdatePhysics()))
		{
			PhysicsMgr.GetFrameByFrame() = false;
			PhysicsMgr.GetPrevFrames().clear();
		}
		if (ImGui::Button("Frame by frame"))
		{
			PhysicsMgr.GetFrameByFrame() = true;
			PhysicsMgr.GetUpdatePhysics() = true;
		}

		if (!PhysicsMgr.GetPrevFrames().empty())
		{
			if(ImGui::SliderInt("Go back n Frames", &PhysicsMgr.GetPrevFrameIdx(), 0, (int)PhysicsMgr.GetPrevFrames().size() - 1))
				PhysicsMgr.GetBodies() = PhysicsMgr.GetPrevFrames()[PhysicsMgr.GetPrevFrameIdx()];
		}

		ImGui::Checkbox("Fixed Dt", &PhysicsMgr.GetFixedDt());
		ImGui::SameLine();
		ImGui::Text("Current Dt: %f", PhysicsMgr.GetDt());
		if (PhysicsMgr.GetFixedDt()) ImGui::SliderFloat("Set Dt", &PhysicsMgr.GetDt(), 1.0f / 120.0f, 1.0f / 20.0f);
		else PhysicsMgr.GetDt() = FRC.GetFrameTime();


		ImGui::SliderFloat("Contact solver beta", &PhysicsMgr.GetBeta(), 0.0f, 1.0f);
		ImGui::SliderFloat("Restitution Coefficient", &PhysicsMgr.GetRestitution(), 0.0f, 1.0f);
		ImGui::Checkbox("Debug draw contact manifold", &PhysicsMgr.mbShowManifold);
		ImGui::SliderInt("Contact solver iterations", &PhysicsMgr.GetContactIterations(), 1, 20);
		ImGui::Separator();

		ImGui::Text("----- EDITOR -----");
		if (ImGui::Button("Create Rigidbody") || KeyTriggered(Key::C))
		{
			GfxManager.AddMesh();
			PhysicsMgr.CreateRigidBody();
			mCurrentEditingBody = (int)PhysicsMgr.GetBodies().size() - 1;
			//auto& b = PhysicsMgr.GetBodies()[mCurrentEditingBody];
			//b.mTransform.mPosition = glm::linearRand(glm::vec3(-140.0f, -20.0f, -140.0f), glm::vec3(140.0f, 20.0f, 140.0f));
			//b.mTransform.mScale = glm::vec3(glm::linearRand(1.0f, 5.0f));
			//b.mFrictionCoeff = glm::linearRand(0.0f, 1.0f);
		}

		std::string text = "Current Scene: " + mCurrentFile;

		ImGui::Text(text.c_str());
		if (ImGui::Button("Reset scene") || KeyTriggered(Key::R)) LoadScene("Resources/Scenes/" + mCurrentFile);

		if (ImGui::BeginCombo("Load Scene", "---- Select ----"))
		{
			for (auto& it : std::filesystem::directory_iterator("Resources/Scenes/"))
			{
				ImGui::Bullet(); ImGui::SameLine();
				if (ImGui::Selectable(it.path().string().c_str()))
				{
					LoadScene(it.path().string());
					mCurrentEditingBody = -1;
				}
			}
			ImGui::EndCombo();
		}
		ImGui::Separator();
		static char buff[30] = {};
		ImGui::InputText("Enter scene name", buff, sizeof(buff));

		if (ImGui::Button("Save Scene"))
		{
			SaveScene(std::string(buff));
			memcpy(buff, "\0", sizeof(buff));
		}
	}
	ImGui::End();
}

/**
 * Edits rigidbody properties via ImGui
*/
void Editor::Edit() const
{
	if (mCurrentEditingBody == -1 || mCurrentEditingBody >= PhysicsMgr.GetBodies().size()) return;
	auto& rb = PhysicsMgr.GetBodies()[mCurrentEditingBody];

	auto& P			= rb.mLinearMomentum;
	auto& L			= rb.mAngularMomentum;
	auto& V			= rb.mVelocity;
	auto& w			= rb.mAngularVelocity;
	auto& ILocal	= rb.mInertiaTensorLocal;
	auto& ILocalInv = rb.mInertiaTensorLocalInverse;
	auto& IWorld	= rb.mInertiaTensorWorld;
	auto& IWorldInv = rb.mInertiaTensorWorldInverse;
	auto& pos		= rb.mTransform.mPosition;
	auto& R			= rb.mTransform.mRotation;
	auto& RMat		= rb.mTransform.mRotationMatrix;
	auto& M			= rb.mMass;
	auto& MInv		= rb.mInvMass;

	if (ImGui::Begin("Edit Rigidbody"))
	{
		if (ImGui::Button("Reset"))
		{
			pos = { 0.0f, 0.0f, 0.0f };
			rb.mTransform.SetRotationMatrix(glm::vec3(0.0f));
			w = L = V = P = glm::vec3(0.0f);
			//ILocal = ILocalInv = glm::identity<glm::mat3>();
		}
		ImGui::Separator();
		ImGui::Checkbox("Lock position", &rb.mbLockPosition);
		ImGui::Separator();
		ImGui::Checkbox("Apply Forces", &rb.mbApplyForces);
		ImGui::Separator();
		ImGui::Checkbox("Apply Gravity", &rb.mbApplyGravity);
		ImGui::Separator();
		if (ImGui::BeginCombo("Change Mesh", "---- Select ----"))
		{
			for (auto& it : std::filesystem::directory_iterator("Resources/Meshes/"))
			{
				ImGui::Bullet(); ImGui::SameLine();
				if (ImGui::Selectable(it.path().string().c_str()))
				{
					auto& mesh = GfxManager.GetMeshes()[rb.mID];
					mesh->DestroyMesh();
					mesh->CreateMesh(it.path().string());
					rb.ComputeInertiaTensor(mesh->GetTriangles());
					rb.mCollider->Shutdown();
					rb.mCollider->Initialize(mesh->GetMeshData());
				}
			}
			ImGui::EndCombo();
		}
		ImGui::Separator();
		EditBodyTransform();

		ImGui::DragFloat("Friction Coefficient", &rb.mFrictionCoeff, 0.05f, 0.0f, 1.0f);

		auto prevMass = M;
		if (ImGui::DragFloat("Mass", &M, 0.25f, 0.0f, 100.0f))
		{
			MInv = 1.0f / M;
			if (M < cEpsilon)
			{
				MInv = 0.0f;
				ILocalInv = glm::mat3();
			}

			if (prevMass < cEpsilon)
				ILocalInv = glm::inverse(ILocal);
		}

		ImGui::PushItemFlag(ImGuiItemFlags_::ImGuiItemFlags_Disabled, true);
		ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
		ImGui::DragFloat("Inv Mass", &MInv);
		ImGui::Separator();
		ImGui::PopItemFlag();
		ImGui::PopStyleVar();

		ImGui::DragFloat3("Linear Momentum", &P[0]);
		ImGui::PushItemFlag(ImGuiItemFlags_::ImGuiItemFlags_Disabled, true);
		ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
		ImGui::DragFloat3("Velocity", &V[0]);
		ImGui::Separator();
		ImGui::PopItemFlag();
		ImGui::PopStyleVar();

		ImGui::DragFloat3("Angular Momentum", &L[0]);
		ImGui::PushItemFlag(ImGuiItemFlags_::ImGuiItemFlags_Disabled, true);
		ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
		ImGui::DragFloat3("Angular Velocity", &w[0]);
		ImGui::PopItemFlag();
		ImGui::PopStyleVar();
		ImGui::Separator();

		ImGui::PushID(this);

		ImGui::Text("Local Inertia Tensor");
		if (ImGui::DragFloat3("Row 1", &ILocal[0][0], 0.1f, 0.01f, 1000.0f)) ILocalInv = glm::inverse(ILocal);
		if (ImGui::DragFloat3("Row 2", &ILocal[1][0], 0.1f, 0.01f, 1000.0f)) ILocalInv = glm::inverse(ILocal);
		if (ImGui::DragFloat3("Row 3", &ILocal[2][0], 0.1f, 0.01f, 1000.0f)) ILocalInv = glm::inverse(ILocal);
		ImGui::PopID();

		ImGui::PushItemFlag(ImGuiItemFlags_::ImGuiItemFlags_Disabled, true);
		ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);

		ImGui::Text("Local Inverse Inertia Tensor");
		ImGui::DragFloat3("Row 1", &ILocalInv[0][0], 0.1f, 0.01f, 1000.0f);
		ImGui::DragFloat3("Row 2", &ILocalInv[1][0], 0.1f, 0.01f, 1000.0f);
		ImGui::DragFloat3("Row 3", &ILocalInv[2][0], 0.1f, 0.01f, 1000.0f);

		ImGui::Text("World Inertia Tensor");
		ImGui::DragFloat3("Row 1", &IWorld[0][0], 0.1f, 0.01f, 1000.0f);
		ImGui::DragFloat3("Row 2", &IWorld[1][0], 0.1f, 0.01f, 1000.0f);
		ImGui::DragFloat3("Row 3", &IWorld[2][0], 0.1f, 0.01f, 1000.0f);

		ImGui::PopItemFlag();
		ImGui::PopStyleVar();

	}
	ImGui::End();
}

/**
 * Edits Rigidbody's transform via ImGuizmo
*/
void Editor::EditBodyTransform() const
{
	static glm::vec3 snapTranslation = glm::vec3(1.0f);
	static glm::vec3 snapRotation = glm::vec3(1.0f);
	static glm::vec3 snapScale = glm::vec3(1.0f);

	static ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
	static ImGuizmo::MODE mCurrentGizmoMode = ImGuizmo::LOCAL;

	if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
		mCurrentGizmoMode = ImGuizmo::LOCAL;
	ImGui::SameLine();
	if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
		mCurrentGizmoMode = ImGuizmo::WORLD;

	if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
		mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
	ImGui::SameLine();
	if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
		mCurrentGizmoOperation = ImGuizmo::ROTATE;
	ImGui::SameLine();
	//if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
	//	mCurrentGizmoOperation = ImGuizmo::SCALE;

	static bool useSnap = false;
	ImGui::Checkbox("Snap", &useSnap);
	glm::vec3 snap;
	switch (mCurrentGizmoOperation)
	{
	case ImGuizmo::TRANSLATE:
		snap = snapTranslation;
		ImGui::DragFloat3("Snap", &snapTranslation.x);
		break;
	case ImGuizmo::ROTATE:
		snap = snapRotation;
		ImGui::DragFloat("Angle Snap", &snapRotation.x);
	}
	ImGuiIO& io = ImGui::GetIO();
	ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
	auto& rb = PhysicsMgr.GetBodies()[mCurrentEditingBody];
	auto& pos = rb.mTransform.mPosition;
	auto& sca = rb.mTransform.mScale;
	auto& R = rb.mTransform.mRotation;
	auto& RMat = rb.mTransform.mRotationMatrix;

	//auto translate = glm::translate(glm::vec3(0.0f));
	auto translate = glm::translate(pos);
	auto scale = glm::scale(sca);
	auto rotate = rb.mTransform.GetRotationMatrix4x4();

	auto mModel2World = translate * rotate;
	auto w2c = GfxManager.GetCamera().GetView();
	auto c2p = GfxManager.GetCamera().GetProj();

	ImGuizmo::Manipulate(&w2c[0][0], &c2p[0][0], mCurrentGizmoOperation, mCurrentGizmoMode,
		&mModel2World[0][0], nullptr, useSnap ? &snap.x : nullptr);
	//ImGuizmo::DecomposeMatrixToComponents(&mModel2World[0][0], matrixTranslation,
	//	matrixRotation, matrixScale);

	ImGui::Separator();
	if (ImGui::DragFloat3("Position", &mModel2World[3][0], 0.1f, -10000.0f, 10000.0f))
		translate = glm::translate(glm::vec3(mModel2World[3]));
	if (ImGui::DragFloat3("Rotation", &R[0], 0.1f, -2.0f * glm::pi<float>(), 2.0f * glm::pi<float>()))
	{
		R = glm::normalize(R);
		RMat = glm::toMat3(R);
		rotate = rb.mTransform.GetRotationMatrix4x4();
		mModel2World = translate * rotate;
	}
	if (ImGui::DragFloat("Scale", &sca[0], 0.25f, 0.01f, 300.0f))
		sca.z = sca.y = sca.x;


	ImGui::PushItemFlag(ImGuiItemFlags_::ImGuiItemFlags_Disabled, true);
	ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);

	ImGui::Text("Rotation Matrix");
	ImGui::DragFloat3("Row 1", &RMat[0][0], 0.1f, 0.01f, 1000.0f);
	ImGui::DragFloat3("Row 2", &RMat[1][0], 0.1f, 0.01f, 1000.0f);
	ImGui::DragFloat3("Row 3", &RMat[2][0], 0.1f, 0.01f, 1000.0f);
	ImGui::PopItemFlag();
	ImGui::PopStyleVar();

	pos = glm::vec3(mModel2World[3]);
	R = glm::quat_cast(mModel2World);
}

/**
 * Dsiplays help window
*/
void Editor::HelpWindow()
{
	if (ImGui::Begin("Help"))
	{
		ImGui::Text("Use right click to rotate camera around");
		ImGui::Text("Use right click + WASD to move");
		ImGui::Text("Create/select Rigidbody. You can alter its properties, including mesh");
		ImGui::Text("Press F with the mouse hovered on the selected object to apply a force to it");
	}
	ImGui::End();
}

/**
 * Applies a force on the object that is currently selected
 * @param _factor - how strong the force will be
*/
void Editor::ApplyForceOnSelectedObject(float _factor)
{
	if (mCurrentEditingBody == -1) return;
	auto rb = &PhysicsMgr.GetBodies()[mCurrentEditingBody];

	auto origin = GfxManager.GetCamera().GetPosition();
	auto mousePos = InputManager.RawMousePos();
	auto dir = glm::vec3(ScreenToWorld({ mousePos.x, mousePos.y }));
	Geometry::Ray ray{ origin,  dir };
	Geometry::Sphere s;
	s.radius = 1.0f;
	s.center = rb->mTransform.mPosition;
	float t = Geometry::IntersectionRaySphere(ray, s);

	if (t > 0.0f)
		rb->ApplyForceAndTorque(dir * rb->mMass * _factor, origin + dir * t);
}

/**
 * Throws a ray from the mouse to the given object
 * @param _rb - the object to throw the ray to
*/
float Editor::RayFromMouseToObject(const Rigidbody& _rb) const
{
	auto origin = GfxManager.GetCamera().GetPosition();
	auto mousePos = InputManager.RawMousePos();
	auto dir = glm::vec3(ScreenToWorld({ mousePos.x, mousePos.y }));
	Geometry::Ray ray{ origin,  dir };
	Geometry::Sphere s;
	s.radius = _rb.mTransform.mScale.x / 2.0f;
	s.center = _rb.mTransform.mPosition;
	return Geometry::IntersectionRaySphere(ray, s);
	
}

/**
 * iterates throught the body list to select one in particular
*/
void Editor::SelectBody()
{
	if (ImGui::IsAnyItemHovered() || ImGuizmo::IsUsing() || ImGuizmo::IsOver()) return;

	auto& bodies = PhysicsMgr.GetBodies();
	float minT = std::numeric_limits<float>().max();
	int i = 0;
	int minIdx = -1;
	for (auto& b : bodies)
	{
		float t = RayFromMouseToObject(b);
		if (t >= 0.0f)
		{
			if (t < minT)
			{
				minT = t;
				minIdx = i;
			}
		}
		++i;
	}

	if (minIdx != -1)
	{
		auto min = bodies[minIdx].mTransform.mPosition - bodies[minIdx].mTransform.mScale;
		auto max = bodies[minIdx].mTransform.mPosition + bodies[minIdx].mTransform.mScale;
		if (mCurrentEditingBody != minIdx)
		{
			//DebugRenderMgr.DebugDrawCube({ min, max }, glm::vec4(0.0f, 1.0f, 0.0f, 1.0f));
			if(MouseTriggered(MouseKey::LEFT))
				mCurrentEditingBody = minIdx;
		}
	}
}

/**
 * Converts the mouse position from screen coordinates to world coordinates
 * @param mousePos - mouse position in screen coords
 * @return - mouse position in world coords
*/
glm::vec4 Editor::ScreenToWorld(const glm::vec2& mousePos) const
{
	glm::vec3 normalized = ScreenToNormalized(mousePos);
	glm::vec4 clipped = NormalizedToClipped(normalized);
	glm::vec4 cam = ProjToCam(clipped);
	glm::vec4 world = CamToWorld(cam);

	return world;
}

/**
 * Converts the mouse position from screen coordinates to normalized coordinates
 * @param mousePos - mouse position in screen coords
 * @return - mouse position in normalized coords
*/
glm::vec3 Editor::ScreenToNormalized(const glm::vec2& mousePos) const
{
	glm::vec3 normalized;

	float windowW = static_cast<float>(GfxManager.GetWindow().GetWindowSize().x);
	float windowH = static_cast<float>(GfxManager.GetWindow().GetWindowSize().y);

	normalized.x = (2.0f * mousePos.x) / windowW - 1.0f;
	normalized.y = 1.0f - (2.0f * mousePos.y) / windowH;
	normalized.z = 1.0f;

	return normalized;
}

/**
 * Converts the mouse position from normalized coordinates to clipped coordinates
 * @param mousePos - mouse position in screen coords
 * @return - mouse position in clipped coords
*/
glm::vec4 Editor::NormalizedToClipped(const glm::vec3& mouseScreen) const
{
	return glm::vec4(mouseScreen.x, mouseScreen.y, -1.0f, 1.0f);
}

/**
 * Converts the mouse position from projected coordinates to camera coordinates
 * @param mousePos - mouse position in projected coords
 * @return - mouse position in camera coords
*/
glm::vec4 Editor::ProjToCam(const glm::vec4& mouseClipped) const
{
	glm::vec4 c = glm::inverse(GfxManager.GetCamera().GetProj()) * mouseClipped;
	c = glm::vec4(c.x, c.y, -1.0f, 0.0f);

	return c;
}

/**
 * Converts the mouse position from camera coordinates to world coordinates
 * @param mousePos - mouse position in camear coords
 * @return - mouse position in world coords
*/
glm::vec4 Editor::CamToWorld(const glm::vec4& mouse3D) const
{
	glm::mat4 mat = glm::inverse(GfxManager.GetCamera().GetView());
	glm::vec4 w = mat * mouse3D;
	w = glm::normalize(w);

	return w;
}

