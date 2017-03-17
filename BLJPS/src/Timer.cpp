#include "Timer.h"

#pragma comment(lib,"Winmm.lib")
#include <Windows.h>

Timer::Timer()
{
	elapsedTime = 0;
}

void Timer::StartTimer()
{
	elapsedTime = timeGetTime();
}
unsigned long Timer::GetTime()
{
	return timeGetTime()-elapsedTime;
}
unsigned long Timer::EndTimer()
{
	elapsedTime=timeGetTime()-elapsedTime;
	return elapsedTime;
}
