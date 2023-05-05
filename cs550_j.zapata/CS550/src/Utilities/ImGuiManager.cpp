// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the ImGui Manager class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_sdl.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/ImGuizmo.h"
#include "../Graphics/RenderManager.h"
#include "ImGuiManager.h"

/**
 * Initializes ImGui Context
*/
void ImGuiManager::Initialize() const
{
	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// Setup Platform/Renderer bindings
	const char* glsl_version = "#version 130";
	ImGui_ImplSDL2_InitForOpenGL(GfxManager.GetWindow().GetHandle(), GfxManager.GetWindow().GetContext());
	ImGui_ImplOpenGL3_Init(glsl_version);

}

/**
 * Starts ImGui frame
*/
void ImGuiManager::StartFrame() const
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplSDL2_NewFrame(GfxManager.GetWindow().GetHandle());
	ImGui::NewFrame();
	ImGuizmo::BeginFrame();
	ImGuizmo::Enable(true);
}

/**
 * Ends ImGui Frame
*/
void ImGuiManager::EndFrame() const
{
	ImGui::EndFrame();
}

/**
 * Renders ImGui
*/
void ImGuiManager::Render() const
{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

/**
 * Cleans up ImGui resources
*/
void ImGuiManager::CleanUp() const
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();
}