// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the implementation of the Input manager class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#include "../Graphics/RenderManager.h"
#include "../ImGui/imgui_impl_sdl.h"
#include "InputManager.h"

/**
 * Initializes arrays to false
*/
bool InputHandler::Initialize()
{
	// Set both Current and previous arrays of booleans to false
	for (unsigned i = 0; i < MOUSE_KEY_AMOUNT; ++i)
	{
		mMouseCurrent[i] = 0;
		mMousePrevious[i] = 0;
	}

	for (unsigned i = 0; i < KEYBOARD_KEY_AMOUNT; ++i)
	{
		mKeyCurrent[i] = 0;
		mKeyPrevious[i] = 0;
	}

	return true;
}

/**
 * Initializes a frame by resetting arrays
*/
void InputHandler::StartFrame()
{
	/* Reset the Current and Previous arrays */
	for (unsigned i = 0; i < MOUSE_KEY_AMOUNT; ++i)
		mMousePrevious[i] = mMouseCurrent[i];

	for (unsigned i = 0; i < KEYBOARD_KEY_AMOUNT; ++i)
		mKeyPrevious[i] = mKeyCurrent[i];

	mWheelCurrent = false;
	mWheelScroll = 0;
}

/**
 * Handles receiving events
 * @param _quit - whether to quit the app or not
*/
void InputHandler::HandleEnvents(bool* _quit)
{
	GetRawMouse();

	SDL_Event event;
	while (SDL_PollEvent(&event))
	{
		ImGui_ImplSDL2_ProcessEvent(&event);


		switch (event.type)
		{
		case SDL_QUIT:
			*_quit = true;
			break;
		case SDL_KEYDOWN:
		case SDL_KEYUP:
			InputManager.HandleKeyEvent(event);
			break;
		case SDL_MOUSEBUTTONDOWN:
		case SDL_MOUSEBUTTONUP:
			InputManager.HandleMouseEvent(event);
			break;
		case SDL_MOUSEWHEEL:
			InputManager.HandleMouseWheel(event);
			break;
		case SDL_MOUSEMOTION:
			mRelMouse = glm::ivec2(event.motion.xrel, event.motion.yrel);
			break;
		}
	}
}

#pragma region // MOUSE //

/**
 * Handles events from the mouse
 * @param event
*/
void InputHandler::HandleMouseEvent(SDL_Event event)
{
	// Access the index with -1 beacuse they go:
	// LEFT = 1, MIDDLE = 2, RIGHT = 3
	mMouseCurrent[event.button.button - 1] = event.button.state ? true : false;
}

/**
 * Checks whether the mouse wheel was triggered
 * @return true if triggered, false otherwise
*/
bool InputHandler::WheelTriggered()
{
	return mWheelCurrent;
}

/**
 * Checks whether a mouse button is down
 * @param index - the mouse button to check
 * @return true if held down, false otherwise
*/
bool InputHandler::MouseIsDown(MouseKey index)
{
	return mMouseCurrent[static_cast<unsigned>(index)];
}

/**
 * Checks whether a mouse button is up
 * @param index - the mouse button to check
 * @return true if held up, false otherwise
*/
bool InputHandler::MouseIsUp(MouseKey index)
{
	return !mMouseCurrent[static_cast<unsigned>(index)];
}

/**
 * Checks whether a mouse button is triggered
 * @param index - the mouse button to check
 * @return true if triggered, false otherwise
*/
bool InputHandler::MouseIsTriggered(MouseKey index)
{
	if (mMouseCurrent[static_cast<unsigned>(index)] == true)
	{
		if (mMouseCurrent[static_cast<unsigned>(index)] != mMousePrevious[static_cast<unsigned>(index)])
			return true;
	}
	return false;
}

/**
 * Checks whether a mouse button was released
 * @param index - the mouse button to check
 * @return true if released, false otherwise
*/
bool InputHandler::MouseIsReleased(MouseKey index)
{
	if (mMouseCurrent[static_cast<unsigned>(index)] == false)
	{
		if (mMouseCurrent[static_cast<unsigned>(index)] != mMousePrevious[static_cast<unsigned>(index)])
			return true;
	}
	return false;
}

/**
 * Gets the wheel scroll value
 * @return wheel scroll value
*/
int InputHandler::GetWheelScroll() const
{
	return mWheelScroll;
}

#pragma endregion

#pragma region // KEYBOARD //

/**
 * Handles key events
 * @param event
*/
void InputHandler::HandleKeyEvent(SDL_Event event)
{
	SDL_Keycode ScanCode = event.key.keysym.scancode;

	if (ScanCode > 0 && ScanCode < KEYBOARD_KEY_AMOUNT)
		mKeyCurrent[ScanCode] = event.key.state ? true : false;
}

/**
 * Checks whether a key is down
 * @param index - the keyn to check
 * @return true if held down, false otherwise
*/
bool InputHandler::KeyIsDown(Key index)
{
	if (ImGui::GetIO().WantCaptureKeyboard) return false;

	return mKeyCurrent[static_cast<unsigned>(index)];
}

/**
 * Checks whether a key is up
 * @param index - the keyn to check
 * @return true if up, false otherwise
*/
bool InputHandler::KeyIsUp(Key index)
{
	if (ImGui::GetIO().WantCaptureKeyboard) return false;
	return !KeyIsDown(index);
}

/**
 * Checks whether a key is triggered
 * @param index - the keyn to check
 * @return true if triggered, false otherwise
*/
bool InputHandler::KeyIsTriggered(Key index)
{
	if (ImGui::GetIO().WantCaptureKeyboard) return false;

	if (mKeyCurrent[static_cast<unsigned>(index)] == true)
	{
		if (mKeyCurrent[static_cast<unsigned>(index)] != mKeyPrevious[static_cast<unsigned>(index)])
			return true;
	}
	return false;
}

/**
 * Checks whether a key is released
 * @param index - the key to check
 * @return true if released, false otherwise
*/
bool InputHandler::KeyIsReleased(Key index)
{
	if (ImGui::GetIO().WantCaptureKeyboard) return false;

	if (mKeyCurrent[static_cast<unsigned>(index)] == false)
	{
		if (mKeyCurrent[static_cast<unsigned>(index)] != mKeyPrevious[static_cast<unsigned>(index)])
			return true;
	}
	return false;
}
#pragma endregion

#pragma region // GAMEPAD //

/**
 * Handles mouse wheel events
 * @param event
*/
void InputHandler::HandleMouseWheel(SDL_Event event)
{
	mWheelCurrent = true;
	mWheelScroll = event.wheel.y;
}
#pragma endregion

#pragma region // MOUSE POSITIONS //

#include "../Graphics/Window.h"

/**
 * Gets the raw mouse position (in screen coords)
 * @return raw mouse pos
*/
const glm::ivec2& InputHandler::RawMousePos() const
{
	return mRawMouse;
}

/**
 * Gets the window mouse position (in screen coords)
 * @return window mouse pos
*/
const glm::vec2& InputHandler::WindowMousePos() const
{
	return mWindowMouse;
}

/**
 * Sets mouse position (in screen coords)
 * @param pos
*/
void InputHandler::SetMousePos(glm::vec2 pos)
{
	mWindowMouse = pos;
}

/**
 * Computes the raw mouse position
*/
void InputHandler::GetRawMouse()
{
	SDL_GetMouseState(&mRawMouse[0], &mRawMouse[1]);
	mWindowMouse = mRawMouse - GfxManager.GetWindow().GetWindowSize() / 2;
	mWindowMouse.y *= -1;
}

#pragma endregion
