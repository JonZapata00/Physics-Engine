// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the Window class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------
#pragma once

#include "../Utilities/pch.hpp"
#include "../Math/math.h"
#include <SDL2/SDL.h>

/**
 * Simple window class, it holds a SDL window and initializes it and destroys it
 */
class Window
{
public:
	~Window() { DestroyWindow(); }
	bool GenerateWindow(std::string name, glm::ivec2 window_size);
	glm::ivec2 GetWindowSize () const { return mWindowSize; }
	void Swap();

	SDL_Window* GetHandle() const { return mWindowHandle; };
	SDL_GLContext		GetContext() const { return mGLContext; };
	const std::string& GetName() const { return mWindowName; }


private:
	/* Clean the resources for the OpenGL context and the window */
	void DestroyWindow();

	SDL_Window* mWindowHandle = nullptr;
	SDL_GLContext	mGLContext = {};

	std::string		mWindowName = {};
	glm::ivec2		mWindowSize = {};
	glm::ivec2		mScreenSize = {};
};

