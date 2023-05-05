// ----------------------------------------------------------------------------
//	Copyright (C)DigiPen Institute of Technology.
//	Reproduction or disclosure of this file or its contents without the prior
//	written consent of DigiPen Institute of Technology is prohibited.
//
//	Purpose:		This file contains the declaration of the InputManager class
//	Project:		cs550_j.zapata
//	Author:			Jon Zapata (j.zapata@digipen.edu)
// ----------------------------------------------------------------------------

#pragma once
#include "../Utilities/pch.hpp"
#include "../Math/math.h"
#include <SDL2/SDL.h>

#include "../Utilities/Singleton.h"

static const int KEYBOARD_KEY_AMOUNT = SDL_NUM_SCANCODES;
static const int MOUSE_KEY_AMOUNT = 3;

enum class MouseKey
{
	LEFT, MID, RIGHT
};

enum class Key : unsigned
{
	A = SDL_SCANCODE_A, B, C, D, E, F, G, H, I, J, K, L, M,
	N, O, P, Q, R, S, T, U, V, W, X, Y, Z,

	Num1 = SDL_SCANCODE_1, Num2, Num3, Num4, Num5, Num6, Num7,
	Num8, Num9, Num0,

	Tab = SDL_SCANCODE_TAB,
	Control = SDL_SCANCODE_LCTRL,
	LShift = SDL_SCANCODE_LSHIFT,
	LAlt = SDL_SCANCODE_LALT,
	Enter = SDL_SCANCODE_RETURN,
	Delete = SDL_SCANCODE_DELETE,
	Esc = SDL_SCANCODE_ESCAPE,
	Space = SDL_SCANCODE_SPACE,

	Plus = SDL_SCANCODE_KP_PLUS,
	Minus = SDL_SCANCODE_KP_MINUS,

	Supr = SDL_SCANCODE_BACKSPACE,

	F1 = SDL_SCANCODE_F1,
	F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,

	Right = SDL_SCANCODE_RIGHT,
	Left = SDL_SCANCODE_LEFT,
	Down = SDL_SCANCODE_DOWN,
	Up = SDL_SCANCODE_UP,
};

class InputHandler
{
	MAKE_SINGLETON(InputHandler)

public:
	bool Initialize();
	void StartFrame();

	void HandleEnvents(bool* _quit);

	bool MouseIsDown(MouseKey index);
	bool MouseIsUp(MouseKey index);
	bool MouseIsTriggered(MouseKey index);
	bool MouseIsReleased(MouseKey index);
	int  GetWheelScroll() const;

	bool KeyIsDown(Key index);
	bool KeyIsUp(Key index);
	bool KeyIsTriggered(Key index);
	bool KeyIsReleased(Key index);

	bool WheelTriggered();

	const glm::ivec2& RawMousePos() const;
	const glm::vec2& WindowMousePos() const;
	void  SetMousePos(glm::vec2 pos);

private:
	void GetRawMouse();
	void HandleMouseEvent(SDL_Event event);
	void HandleKeyEvent(SDL_Event event);
	void HandleMouseWheel(SDL_Event event);

	/* Mouse */
	glm::ivec2 mRawMouse = {};
	glm::ivec2 mRelMouse = {};
	glm::vec2  mWindowMouse = {};
	int  mWheelScroll = 0;
	bool mMouseCurrent[MOUSE_KEY_AMOUNT] = { false };
	bool mMousePrevious[MOUSE_KEY_AMOUNT] = { false };

	/* KeyboardKeys (256) */
	bool mKeyCurrent[KEYBOARD_KEY_AMOUNT] = { false };
	bool mKeyPrevious[KEYBOARD_KEY_AMOUNT] = { false };

	bool mWheelCurrent = false;
	//Gamepad
	glm::vec2 mCurrentJoyStickValue = {};
	float offset = 0.2f;
	float lapse = 0.0f;
};

#define InputManager (InputHandler::Instance())

#define KeyDown(i)				InputManager.KeyIsDown(i)
#define KeyUp(i)					InputManager.KeyIsUp(i)
#define KeyTriggered(i)		InputManager.KeyIsTriggered(i)
#define KeyReleased(i)		InputManager.KeyIsReleased(i)

#define MouseWheel()			InputManager.WheelTriggered()
#define MouseDown(i)			InputManager.MouseIsDown(i)
#define MouseUp(i)				InputManager.MouseIsUp(i)
#define MouseTriggered(i) InputManager.MouseIsTriggered(i)
#define MouseReleased(i)	InputManager.MouseIsReleased(i)


