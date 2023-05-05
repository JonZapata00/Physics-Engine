// ---------------------------------------------------------------------------
// Project Name		:	Alpha Engine
// File Name		:	AEFrameRateController.cpp
// Author			:	Sun Tjen Fam
// Creation Date	:	2007/04/27
// Purpose			:	implementation of the frame rate controller
// History			:
// - 2007/04/27		:	- initial implementation
// - 2010/08/17 : Fixed a bug that resulted in the gAEFrameRate variable not
//                being updated. - Dan Weiss
// ---------------------------------------------------------------------------
#include "FrameRateController.h"
#include <Windows.h> //QueryPerformance... functions.

// ---------------------------------------------------------------------------
// Defines

#define FRAME_RATE_SYNC_TO_RETRACE 0

	// ---------------------------------------------------------------------------
	// get the current time in seconds

	double FrameRateController::GetCPUTime()
	{
		signed long long f, t;
		double r, r0, r1;

		QueryPerformanceFrequency((LARGE_INTEGER*)(&f));
		QueryPerformanceCounter((LARGE_INTEGER*)(&t));

		//@FIXED - precision warning
		r0 = double(t / f);
		r1 = (t - ((t / f) * f)) / (double)(f);
		r = r0 + r1;

		return r;//r0 + r1;
	}

	// ---------------------------------------------------------------------------
	// Functions implementations

	bool FrameRateController::Initialize()
	{
		FrameRateController::sFrameCounter = 0;
		FrameRateController::sFrameRateMax = 60.0;
		FrameRateController::sFrameRate = FrameRateController::sFrameRateMax;
		FrameRateController::sFrameTime = 1.0 / FrameRateController::sFrameRate;
		FrameRateController::sFrameTimeMin = 1.0 / FrameRateController::sFrameRateMax;
		return true;
	}

	// ---------------------------------------------------------------------------

	void FrameRateController::Reset()
	{
		//AE_ASSERT_MESG(gAEFrameRateMax > 0.0, "maximum frame rate MUST be greater than 0");

		FrameRateController::sFrameCounter = 0;
		FrameRateController::sFrameRate = sFrameRateMax;
		FrameRateController::sFrameTime = 1.0 / FrameRateController::sFrameRate;
		FrameRateController::sFrameTimeMin = 1.0 / FrameRateController::sFrameRateMax;
	}
	void FrameRateController::Update()
	{ 
		if (sFrameCounter)
			EndFrame();
		else
			sFrameCounter++;
		if (sFrameTime >= 0.1f) sFrameTime = 0.016666f;
		StartFrame(); 
	}

	// ---------------------------------------------------------------------------

	void FrameRateController::StartFrame()
	{
		FrameRateController::sFrameTimeStart = FrameRateController::GetCPUTime();
	}

	// ---------------------------------------------------------------------------

	void FrameRateController::EndFrame()
	{

		// if the total time spent is less than the minimum required time to 
		// maintain the maximum frame rate, wait
		do
		{
			FrameRateController::sFrameTimeEnd = GetCPUTime();
			// calculate the amount of time spend this frame
			sFrameTime = sFrameTimeEnd - sFrameTimeStart;
		}
		while (bFrameRateLocked && (sFrameTime) < FrameRateController::sFrameTimeMin);


		//@FIXED - Reset the frame rate variable
		sFrameRate = 1.0 / sFrameTime;

		// increment the total number of counter
		sFrameCounter++;
	}

	double FrameRateController::GetMaxFrameRate()
	{
		return sFrameRateMax;
	}
	float FrameRateController::GetFrameRate()
	{
		return (float)sFrameRate;
	}
	float FrameRateController::GetFrameTime()
	{
		return (float)sFrameTime;
	}
	double FrameRateController::GetFrameCounter()
	{
		return sFrameCounter;
	}

	// Setters
	void FrameRateController::SetMaxFrameRate(double fps)
	{
		sFrameRateMax = fps;
		FrameRateController::sFrameTimeMin = 1.0 / FrameRateController::sFrameRateMax;
	}
	// ---------------------------------------------------------------------------
	// Static functions implementations

	// ---------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// ----------------------------------------------------------------------------
	// Timer
	AEXTimer::AEXTimer()
		: startTime_(0.0)
		, timeSinceLastTick_(0.0)
		, timeScale_(1.0f)
		, isPaused_(false)
	{
		Reset();
	}
	float AEXTimer::Tick()
	{
		// if the timer is paused, return 0.0 delta time
		if (isPaused_)
		{
			// Update the time stamp to avoid having 
			// a big delta time when the game isn't paused
			timeSinceLastTick_ = FrameRateController::GetCPUTime();
			return 0.0f;
		}

		double dt = FrameRateController::GetCPUTime() - timeSinceLastTick_;
		timeSinceLastTick_ = FrameRateController::GetCPUTime();
		return static_cast<float>(dt)* timeScale_;
	}
	void AEXTimer::Reset()
	{
		// reset the time stamps to the current time
		startTime_ =
			timeSinceLastTick_ = FrameRateController::GetCPUTime();
	}
	void AEXTimer::Start()
	{
		isPaused_ = false;
	}
	void AEXTimer::Pause()
	{
		isPaused_ = true;
	}
	float AEXTimer::GetTimeSinceStart()
	{
		double total_time = FrameRateController::GetCPUTime() - startTime_;
		return static_cast<float>(total_time)* timeScale_;
	}
