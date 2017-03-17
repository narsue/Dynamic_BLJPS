#pragma once

class Timer {	

	unsigned long elapsedTime; // milliseconds

public:
	Timer();
	~Timer(){}

	void StartTimer();

	unsigned long EndTimer();
	unsigned long GetTime();
	unsigned long GetElapsedTime(){ return elapsedTime; }

};