// ---------------------------------------------------------------------------
// Project Name		:	Alpha Engine
// File Name		:	AEFrameRateController.h
// Author			:	Sun Tjen Fam
// Creation Date	:	2007/04/26
// Purpose			:	header file for the frame rate controller
// History			:
// - 2007/04/26		:	- initial implementation
// ---------------------------------------------------------------------------
#ifndef AEX_TIME_H
#define AEX_TIME_H

#include "Singleton.h"

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// TIME CLASS - Static interface for framerate controller
class  FrameRateController
{
	MAKE_SINGLETON(FrameRateController)

public:
	virtual bool Initialize();
	virtual void Update();

	// call between each frame for frame statistics
	void StartFrame();
	void EndFrame();
	void Reset();

	double GetMaxFrameRate();
	float GetFrameRate();
	float GetFrameTime();
	double GetFrameCounter();

	// set max frame rate
	void        LockFrameRate(bool enabled){ bFrameRateLocked = enabled; }
	inline bool FrameRateLocked(){ return bFrameRateLocked; }
	void        SetMaxFrameRate(double fps);

	// uses the CPU clock to return a time in seconds.
	static double GetCPUTime();

private:

	bool bFrameRateLocked = true;
	double	sFrameRateMax = 0.0;	// clamp the frame rate to at most this number
	double	sFrameRate = 0.0;		// the frame rate based on the last frame
	double	sFrameTime = 0.0;		// time taken to process the last frame(in seconds)
	unsigned	sFrameCounter = 0u;	// number of frame since the last reset
	double	sFrameTimeMin = 0.0;
	double	sFrameTimeStart = 0.0;
	double	sFrameTimeEnd = 0.0;
};


// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// TIMER - Simple timer class that makes use of the FRC singleton system.
struct AEXTimer
{
	double startTime_;
	double timeSinceLastTick_;
	float timeScale_;
	bool isPaused_;

	// ------------------------
	// METHODS
	AEXTimer();
	float Tick();		// Returns the time since last tick. Call each frame.
	void Reset();	// Resets the timer values
	void Start();	// Sets paused to false
	void Pause();	// Sets paused to true
	float GetTimeSinceStart(); // returns the time since the last reset
};


// Easy access to singleton
#define FRC FrameRateController::Instance()
// ---------------------------------------------------------------------------

#endif // AEX_TIME_H

