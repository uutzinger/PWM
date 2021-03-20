# PWM
Allows setting any of the Teenys 3.2 pins to create PWM signals.
Uses analogwrite on the PWM pins and SoftPWM on the others.
- Adjusts Resolution (PWM pins only)
- Adjusts Frequency (PWM pins only)
- Adjusts Dutycyle
? provides
```
-------------------------------------------------
PWM Controller
-------------------------------------------------
Frequency: 187500.000000
Duty: +50.000
Resolution:  8
Pin:  5
CPU: 96
PWM Max:  255
Sample rate: 1918
-------------------------------------------------
m/M to disable/enable PWM pin: 
s/S to disable/enable data streaming
v/V to disable/enable human readable data display
------- Data Input--------------------------------
p5   to set PWM pin 5
d50  to set duty cyle to 50%
f512 to set frequency to 512Hz
r8   to set PWM resolution to 8 bits
-------------------------------------------------
Maximum Values:
Teensy 3.2 96MHz Resolution 16bit, 723.42 Hz PWM 
SoftPWN          Resoltuion  8bit,  60    Hz PWM 
-------------------------------------------------
List pins PWM or SOFT or NA here
-------------------------------------------------
Press S to continue.
Streaming was turned off to hold this diplay.
-------------------------------------------------
Urs Utzinger, 2019-21
-------------------------------------------------
```