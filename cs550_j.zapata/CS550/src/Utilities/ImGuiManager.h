// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the ImGui Manager class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#pragma once
#include "Singleton.h"

class ImGuiManager
{
	MAKE_SINGLETON(ImGuiManager)

public:
	void Initialize() const;
	void StartFrame() const;
	void EndFrame() const;
	void Render() const;
	void CleanUp() const;
};

#define ImGuiMgr (ImGuiManager::Instance())