# Arduino Timer Object

#IMPORTANT
### This library was moved to a new repository: https://github.com/aron-bordin/PNG-Arduino-Framework
I'll update just the new repository. Check and star the .PNG Arduino Framework, there are more objects and Arduino components. This new repository has a better documentation.

The **Arduino Timer Object** allow us to easily create Timer callbacks. In a predetermined interval, your function will be called. Works as a "thread", where a secondary function will run when necessary. 


## Installation

1. "Download":https://github.com/aron-bordin/ArduinoTimerObject/archive/master.zip the Master branch from GitHub.
2. Unzip and modify the folder name to "TimerObject"
3. Move the modified folder on your Library folder (On your `Libraries` folder inside Sketchbooks or Arduino software).


## How to use

First, include the TimerObject to your project:
```c++
	#include "TimerObject.h"
```

Now, you can create a new object:
```c++
	TimerObject *timer1 = new TimerObject(1000); //will call the callback in the interval of 1000 ms
	timer1->setOnTimer(&FunctionCallback);
	timer1->Start(); //start the thread.
```

In your loop(), add:
```c++
	timer1->Update(); //will check the Timer and if necessary, will run it.
```


## IMPORTANT
If you use delay(), the Timer will be ignored!! You cannot use delay() command with TimerObject. Instead of using delay, you can use the Timer itself. For example, if you need that your loop run twice per second, just create a timer with 500 ms. It will have the same result that delay(500), but your code will be always running.

## Example

Complete example. Here we created two timers, you can run it and test the result in the Serial monitor.
```c++
	#include "TimerObject.h"

	TimerObject *timer1 = new TimerObject(1000);
	TimerObject *timer2 = new TimerObject(500);


	void setup(){
		Serial.begin(9600);
		timer1->setOnTimer(&PrintHello1);
		timer1->Start();

		timer2->setOnTimer(&PrintHello2);
		timer2->Start();

	}

	void PrintHello1(){
		Serial.println("Hello timer 1 !!");
	}

	void PrintHello2(){
		Serial.println("Hello timer 2!!");
	}
	void loop(){
		timer1->Update();
		timer2->Update();
	}

```

## Documentation

### Constructors
**TimerObject(unsigned long int ms);<br>
TimerObject(unsigned long int ms, CallBackType callback);<br>
TimerObject(unsigned long int ms, CallBackType callback, bool isSingle);**
	
### Functions

**void setInterval(unsigned long int ms);**
Set callback interval

**void setEnabled(bool Enabled);**
Set if is Enabled.

**void setSingleShot(bool isSingle);**
If isSingle is True, the callback will be called once, until you call Start() or Resume() again

**void setOnTimer(CallBackType callback);**
Set function callback

**void Start();**
Start the Timer. Will count the interval from the moment that you start it. If is pause, will restart the Timer.

**void Resume();**
Resume the Timer. If not started, will start it. If paused, will resume it. For example, in a timer of 5 seconds, if it was paused in 3 seconds, the resume in continue in 3 seconds. Start will set passed time to 0 and restart until get 5 seconds.

**void Pause();**
Pause the timer, so you can resume it.

**void Stop();**
Stop the timer.

**void Update();**
Must to be called in the loop(), will check the timer, and if necessary, will run the callback


**unsigned long int getInterval();**
Get the interval

**unsigned long int getCurrentTime();**
Get time passed since the last tick

**CallBackType getOnTimerCallback();**
Get the Timer Callback

**bool isEnabled();**
Check if it is enabled

**bool isSingleShot();**
Check if it is Single Shot
