// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		Main file of the program
//	Project:		cs300_j.zapata_0
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#include <iostream> //std::cout
#include <SDL2/SDL.h> //SDL_Event, init, etc
#include "Utilities/Editor.h"
#include "Utilities/FrameRateController.h"
#include "Graphics/DebugRenderer.h"
#include "Graphics/RenderManager.h"
#include "Physics/PhysicsManager.h" //GraphicsManager class
#include "Input\InputManager.h" //input manager

#undef main
int main(int argc, char* args[])
{
	//variables for wireframe and texture mode
	bool quit = false;

	GfxManager.Initialize(1920, 1080);
	DebugRenderMgr.Initialize();
	PhysicsMgr.Initialize();
	EDITOR.Initialize();
	FRC.Initialize();

	while (!quit)
	{
		//check for input
		InputManager.HandleEnvents(&quit);

		if (KeyTriggered(Key::Esc))
			quit = true;
		FRC.Update();
		GfxManager.StartFrame();
		PhysicsMgr.Update();
		EDITOR.Update();
		GfxManager.RenderAll();
		GfxManager.EndFrame();
	}

	return 0;
}