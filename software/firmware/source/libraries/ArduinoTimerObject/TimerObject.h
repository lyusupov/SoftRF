#include "stdlib.h"
#include "Arduino.h"

#ifndef TIMER_OBJECT_H
#define TIMER_OBJECT_H
typedef void (*CallBackType)();


class TimerObject{
private:
	void Create(unsigned long int ms, CallBackType callback, bool isSingle);
	unsigned long int msInterval;
	bool blEnabled;
	bool blSingleShot;
	CallBackType onRun;
	bool Tick();
	unsigned long LastTime;
	unsigned long DiffTime;//used when I pause the Timer and need to resume

public:
	TimerObject(unsigned long int ms);
	TimerObject(unsigned long int ms, CallBackType callback);
	TimerObject(unsigned long int ms, CallBackType callback, bool isSingle);
	~TimerObject(){Stop();};

	void setInterval(unsigned long int ms);
	void setEnabled(bool Enabled);
	void setSingleShot(bool isSingle);
	void setOnTimer(CallBackType callback);
	void Start();
	void Resume();
	void Pause();
	void Stop();
	void Update();


	unsigned long int getInterval();
	unsigned long int getCurrentTime();
	CallBackType getOnTimerCallback();

	bool isEnabled();
	bool isSingleShot();

};


#endif // TIMER_OBJECT_H
