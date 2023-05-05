// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Window class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#include <GL/glew.h>
#include "Window.h"

/**
 * Creates the window
 * @param _name - window name
 * @param window_size
 * @return - true if success, false otherwise
*/
bool Window::GenerateWindow(std::string name, glm::ivec2 window_size)
{
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		std::cout << "Could not initialize SDL: " << SDL_GetError() << std::endl;
		exit(1);
		return false;
	}
	mWindowSize = window_size;
	mWindowName = name;
	mWindowHandle = SDL_CreateWindow(mWindowName.c_str(), 100, 100, mWindowSize.x, mWindowSize.y, SDL_WINDOW_OPENGL);
	if (mWindowHandle == nullptr)
	{
		std::cout << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
		SDL_Quit();
		exit(1);
		return false;
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

	mGLContext = SDL_GL_CreateContext(mWindowHandle);
	if (mGLContext == nullptr)
	{
		SDL_DestroyWindow(mWindowHandle);
		std::cout << "SDL_GL_CreateContext Error: " << SDL_GetError() << std::endl;
		SDL_Quit();
		exit(1);
		return false;
	}

	glewExperimental = true;
	if (glewInit() != GLEW_OK)
	{
		std::cout << "GLEW Error: Failed to init" << std::endl;
		DestroyWindow();
		exit(1);
		return false;
	}

	return true;
}

/**
 * Swap window
*/
void Window::Swap()
{
	SDL_GL_SwapWindow(mWindowHandle);
}

/**
 * Clears resources
*/
void Window::DestroyWindow()
{
	SDL_GL_DeleteContext(mGLContext);
	SDL_DestroyWindow(mWindowHandle);
	SDL_Quit();
}
