# Peanat's Improved PID Library 
Written for arduino.<br>
Named for peanuts because peanuts are yummy.

### To start:
Download PID_peanut.h and PID_peanut.cpp and add to the folder with your arduino code.
```
#include "PID_peanut.h"

// Initialize all the variables (change as needed; these are default)
double kp = 0.0;
double ki = 0.0;
double kd = 0.0;
double init_output = 0.0;
bool reverse = false;
unsigned long sample_time = 100;
double min_output = 0.0;
double max_output = 255.0;
double deadband = 0.0;

// Create PID object (rename myPID to whatever you need)
PID myPID(kp, ki, kd, init_output, reverse, sample_time, min_output, max_output, deadband)
```

### Calculating PID output:
```
double setpoint;
double input;
double output;

setpoint = YOUR SETPOINT HERE;
input = YOUR INPUT HERE;
// Calculaing PID output
output = myPID.compute(setpoint, input);
```

### If you want to change the PID parameters
```
// If you need to change the PID parameters
myPID.changeSettings(kp, ki, kd, reverse, sample_time, min_output, max_output, deadband);
```
