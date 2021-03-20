////////////////////////////////////////////////////////////////////////////
//
//  This program controls PWM output on any pin
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
////////////////////////////////////////////////////////////////////////////

#include "SoftPWM.h"
#include <iterator>
#include <algorithm>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DISPLAY_INTERVAL      100000    // interval in microseconds between data updates over serial port: 100000=10Hz
#define CHECKINPUT_INTERVAL    50000    // interval in microseconds between polling serial input
#define LEDBLINK_INTERVAL     500000    // interval in microseconds between turning LED on/off for status report
#define POLL_INTERVAL          10000    // interval in microseconds between reading sensor
#define SERIAL_PORT_SPEED    2000000    // Serial Baud Rate
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long pollInterval;                    //
unsigned long lastDisplay;                     //
unsigned long lastRate;                        //
unsigned long elapsedTime;                     //
unsigned long lastInAvail;                     //
unsigned long lastTimestamp;                   //
unsigned long lastBlink;                       //
unsigned long lastPoll;                        //
//
unsigned long timeout;                         //
unsigned long currentTime;                     //
unsigned long mainCount;                       //
unsigned long sampleRate;                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Controlling Reporting and Input Output
bool REPORT = false;                           // Do we need to produce data for serial reporting?
bool STREAM = false;                           // Are we continously streaming data?
bool checkINAVAIL = false;                     // Is it time to check for user input ?
bool VERBOSE = true;                           // Set HEX streamn or text stream
bool SERIAL_REPORTING = false; 	        			 // Boot messages on/off
bool DEBUG_ENABLE = false;                     // Debug on/off
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Indicators
const int ledPin = 13;                         // Check on https://www.pjrc.com/teensy/pinout.html; pin should not interfere with I2C and SPI
bool ledStatus = false;                        // Led should be off at start up

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int  PWM_Pin        = 5;              //
long          CPU_Frequency  = F_CPU / 1E6;    //
float         PWM_Frequency  = 488.28;         // Default Teensy 3.2
unsigned int  PWM_Resolution = 8;              // 
unsigned int  PWM_Max_Value  = pow(2, PWM_Resolution)-1;
float         PWM_Duty       = 50.0;           //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float GetMaxPWMFreqValue(long FREQ, int PWM_Resolution)
{
  /* for Teensy CPU frequency
      24MHZ
      48MHZ
      72MHZ
      96MHZ
      120MHZ
  */
  int FREQ_pointer = -1;
  float PWM_ideal_frequency[5][15]
  {
    {6000000 , 3000000, 1500000, 750000 , 375000, 187500, 93750 , 46875   , 23437.5 , 11718.75 , 5859.375, 2929.687, 1464.843, 732.421, 366.2109},
    {12000000, 6000000, 3000000, 1500000, 750000, 375000, 187500, 93750   , 46875   , 23437.5  , 11718.75 , 5859.375, 2929.687, 1464.843, 732.4218},
    {9000000 , 4500000, 2250000, 1125000, 562500, 281250, 140625, 70312   , 35156.25, 17578.12 , 8789.062, 4394.531, 2197.265, 1098.632, 549.3164},
    {12000000, 6000000, 3000000, 1500000, 750000, 375000, 187500, 93750   , 46875   , 23437.5  , 11718.75 , 5859.375, 2929.687, 1464.843, 732.4218},
    {15000000, 7500000, 3750000, 1875000, 937500, 468750, 234375, 117187.5, 58593.75, 29296.875, 14648.537, 7324.219, 3662.109, 1831.055, 915.527 }
  };

  switch (FREQ) {
    case 24:
      FREQ_pointer = 0;
      break;
    case 48:
      FREQ_pointer = 1;
      break;
    case 72:
      FREQ_pointer = 2;
      break;
    case 96:
      FREQ_pointer = 3;
      break;
    case 120:
      FREQ_pointer = 4;
      break;
    default:
      FREQ_pointer = -1;
      break;
  }
  if (FREQ_pointer >= 0) {
    return (PWM_ideal_frequency[FREQ_pointer][PWM_Resolution - 2]);
  } else {
    return (488.28);
  }
} // end of getMaxPWMFreqValue

void listFrequencies()
{
  Serial.println("-------------------------------------------------");
  for(int i=2; i<=16; i++) {
    Serial.printf("Resolution: %2d bit, Frequency: %f\n", i, GetMaxPWMFreqValue(CPU_Frequency, i));
  }
  Serial.println("-------------------------------------------------");  
} 

/////////////////////////////////////////////////////////

// PWM pins
// FTM1 3, 4,
// FTM0 5, 6, 9, 10, 20, 21, 22, 23,
// FTM2 25, 32

boolean isPWM(uint8_t mypin) {
  const uint8_t last = 12;
  const uint8_t pwmpins[] = {3, 4, 5, 6, 8, 9, 10, 20, 21, 22, 23, 25, 32};
  return std::binary_search(pwmpins, &pwmpins[last], mypin);
}

boolean isSoftPWM(uint8_t mypin) {
  const uint8_t last = 20;
  const uint8_t softpwmpins[] = {0, 1, 2, 7, 8, 11, 12, 14, 15, 16, 17, 18, 19, 24, 26, 27, 28, 29, 30, 31, 33};
  return std::binary_search(softpwmpins, &softpwmpins[last], mypin);
}

void listPins() {
  char pinType[] = "Soft";
  Serial.println("-------------------------------------------------");
  for (int i=0; i<33; i++){
    if      (isPWM(i))     {strcpy(pinType, "PWM");}
    else if (isSoftPWM(i)) {strcpy(pinType, "Soft");}
    else                   {strcpy(pinType, "N.A.");}
    Serial.printf("Pin: %2d, %s\n", i, pinType );
  }
  Serial.println("-------------------------------------------------");  
  
}
/////////////////////////////////////////////////////////
// Serial
char   inBuff[] = "----------------";
int    bytesread;
/////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Setup monitor pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  ledStatus = false;

  // Setup PWM
  /////////////////////////////////////////////////////////
  PWM_Frequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
  analogWriteFrequency(PWM_Pin,PWM_Frequency);
  analogWriteResolution(PWM_Resolution);
  PWM_Max_Value = pow(2, PWM_Resolution)-1;
  analogWrite(PWM_Pin,PWM_Max_Value/2);

  // Setup SofPWM
  /////////////////////////////////////////////////////////
  SoftPWMBegin();
  //SoftPWMSet(SoftPWM_Pin, 0);  // Create and set pin 13 to 0 (off)
  //SoftPWMSetFadeTime(SoftPWM_Pin, 100, 500);   // Set fade time for pin 13 to 100 ms fade-up time, and 500 ms fade-down time

  // Setup Serial
  /////////////////////////////////////////////////////////
  Serial.begin(SERIAL_PORT_SPEED);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  if (SERIAL_REPORTING) {
    Serial.println("Starting PWM");
  }

  // Setup House keeping
  /////////////////////////////////////////////////////////
  pollInterval = 1000; // 1 m sec

  // House keeping & Initializing
  lastDisplay = lastRate = lastTimestamp = lastInAvail = micros();
  mainCount = 0;

  // Ready for input commands
  if (SERIAL_REPORTING) {
    Serial.println("Send S to start streaming.");
  }

  // setup timers
  lastPoll = micros();
  timeout = (unsigned long) (50 * (pollInterval));

  // LED blinking status
  /////////////////////////////////////////////////////////
  digitalWrite(ledPin, HIGH); // initialization completed
  ledStatus = true;
  lastBlink = micros();

  /////////////////////////////////////////////////////////
  if (SERIAL_REPORTING) {
    Serial.println("System ready.");
  }

} // END setup

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  // Main loop delay.
  // Limit the how often the main loop is running
  /////////////////////////////////////////////////////////
  currentTime = micros();                                         // whats the time
  int pollDelay = (pollInterval - (int)(currentTime - lastPoll)); // how long do we need to wait?
  if (pollDelay > 0) {
    delayMicroseconds(pollDelay);  // wait
  }
  lastPoll = currentTime;

  // check if system stalled
  /////////////////////////////////////////////////////////
  if ( (currentTime - lastPoll) > timeout ) {
    // The system stalled and can not keep up with requested main loop interval
    Serial.println("!!!!!!!!!!!!!!!!!!!! RESET SYSTEM: Main loop taking too long !!!!!!!!!!!!!!!!!!!!");
    lastPoll = currentTime = micros();
  }

  // main Loop Counter
  mainCount++;

  // Compute update rates in Hz
  /////////////////////////////////////////////////////////
  elapsedTime = currentTime - lastRate;
  if (elapsedTime >= 1000000) { // 1 second = 1000000 us
    lastRate = currentTime;
    sampleRate = (mainCount * 1000000) / elapsedTime;
    mainCount = 0;
  }

  // Do we want to stream data ?
  /////////////////////////////////////////////////////////
  if ((currentTime - lastDisplay) >= DISPLAY_INTERVAL) {
    lastDisplay = currentTime;
    if (STREAM) {
      REPORT = true;
    } else {
      REPORT = false;
    }
  }

  // Do we want to check input ?
  /////////////////////////////////////////////////////////
  if ((currentTime - lastInAvail) >= CHECKINPUT_INTERVAL) {
    lastInAvail = currentTime;
    checkINAVAIL = true;
  } else {
    checkINAVAIL = false;
  }

  // Do we want to report data ?
  /////////////////////////////////////////////////////////
  if (REPORT) {
    if (VERBOSE) { // human readable reporting
      Serial.println("-Data----");
      Serial.printf("Frequency: %f\n", PWM_Frequency);
      Serial.printf("Duty: %+4.3f\n", PWM_Duty);
      Serial.printf("Resolution: %2d\n", PWM_Resolution);
      Serial.printf("Pin: %2d\n", PWM_Pin);
      Serial.printf("CPU: %2d\n", CPU_Frequency);
      Serial.printf("PWM Max: %4d\n" , PWM_Max_Value);
      Serial.println("-System--");
      Serial.printf("Sample rate: %3d", sampleRate);
      Serial.print("Data Transmission Time [us]: ");
      Serial.println((micros() - currentTime)); //
      Serial.println("-End-----");
    } else { // streaming reporting
      serialLongPrint(lastTimestamp);
      Serial.print(',');
      serialFloatPrint(PWM_Frequency);
      Serial.print(',');
      serialFloatPrint(PWM_Duty);
      Serial.print(',');
      serialIntPrint(PWM_Resolution);
      Serial.print(',');
      serialIntPrint(PWM_Pin);
      Serial.println();
    }
  } // end REPORT

  // Input Commands
  /////////////////////////////////////////////////////////

  if (checkINAVAIL) {
    if (Serial.available()) {
      bytesread = Serial.readBytesUntil('\n', inBuff, 16); // Read from serial until CR is read or timeout exceeded
      inBuff[bytesread] = '\0';
      String instruction = String(inBuff);
      processInstruction(instruction);
    }
  }

  // Blink LED
  /////////////////////////////////////////////////////////

  if ((currentTime - lastBlink) > LEDBLINK_INTERVAL) {
    if (ledStatus == false) {
      digitalWrite(ledPin, HIGH);
      ledStatus = true;
    } else {
      digitalWrite(ledPin, LOW);
      ledStatus = false;
    } // flip LED
    lastBlink = currentTime;
  } // System blinking

} // end of main loop

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Support Routines
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// What are the software options?
void printHelp() {
  Serial.println("-------------------------------------------------");
  Serial.println("PWM Controller");
  Serial.println("-------------------------------------------------");
  Serial.printf( "Frequency: %f\n", PWM_Frequency);
  Serial.printf( "Duty: %+4.3f\n", PWM_Duty);
  Serial.printf( "Resolution: %2d\n", PWM_Resolution);
  Serial.printf( "Pin: %2d\n", PWM_Pin);
  Serial.printf( "CPU: %2d\n", CPU_Frequency);
  Serial.printf( "PWM Max: %4d\n" , PWM_Max_Value);
  Serial.printf( "Sample rate: %3d\n", sampleRate);
  Serial.println("-------------------------------------------------");
  Serial.println("m/M to disable/enable PWM pin: "); 
  Serial.println("s/S to disable/enable data streaming");
  Serial.println("v/V to disable/enable human readable data display");
  Serial.println("------- Data Input--------------------------------");
  Serial.println("p5   to set PWM pin 5");
  Serial.println("d50  to set duty cyle to 50%");
  Serial.println("f512 to set frequency to 512Hz");
  Serial.println("r8   to set PWM resolution to 8 bits");
  Serial.println("-------------------------------------------------");
  Serial.println("Maximum Values:");
  listFrequencies();
  Serial.println("SoftPWM:"); 
  Serial.println("Resolution: 8 bit, Frequency: 60.0");
  listPins();
  Serial.println("Press S to continue.");
  Serial.println("Streaming was turned off to hold this diplay.");
  Serial.println("-------------------------------------------------");
  Serial.println("Urs Utzinger, 2019-21");
  Serial.println("-------------------------------------------------");
}

void processInstruction(String instruction) {
  String value    = "0.01";
  String command  = "o";
  int instructionLength = instruction.length();
  if (instructionLength > 0) {
    command = instruction.substring(0, 1);
  }
  if (instructionLength > 1) {
    value = instruction.substring(1, instructionLength);
  }

  if        (command == 'm') { // turn off PWM
    // ENABLE/DISABLE PWM //////////////////////////////////////////////////////////
    if (isPWM(PWM_Pin))          {
      analogWrite(PWM_Pin, 0);
      Serial.println("off");
    }
    else if (isSoftPWM(PWM_Pin)) {
      SoftPWMSet(PWM_Pin,  0);
      Serial.println("off");
    }
    else                         {
      Serial.print(PWM_Pin);
      Serial.println(" is not a PWM pin.");
    }
  } else if (command == 'M') { // turn on PWM
    if (isPWM(PWM_Pin))          {
      analogWrite(PWM_Pin,       (unsigned int)(PWM_Duty / 100.0 * float(PWM_Max_Value)));
      Serial.println("on");
    }
    else if (isSoftPWM(PWM_Pin)) {
      SoftPWMSetPercent(PWM_Pin, (uint8_t)(PWM_Duty));
      Serial.println("on");
    }
    else                         {
      Serial.print(PWM_Pin);
      Serial.println(" is not a PWM pin.");
    }

  } else if (command == 'd') { // duty cycle
    // SET DUTY CYCL //////////////////////////////////////////////////////////////
    PWM_Duty = value.toFloat();
    if ((PWM_Duty < 0.0) || (PWM_Duty > 100.0)) {
      Serial.println("Duty cyle out of valid Range.");
    } else {
      if (isPWM(PWM_Pin))          {
        analogWrite(PWM_Pin, (unsigned int)(PWM_Duty / 100.0 * float(PWM_Max_Value)));
        Serial.printf("Duty Cycle set to: %+4.3f\n", PWM_Duty);
      }
      else if (isSoftPWM(PWM_Pin)) {
        SoftPWMSetPercent(PWM_Pin, (uint8_t)(PWM_Duty));
        Serial.printf("Duty Cycle set to: %+4.3f\n", PWM_Duty);
      }
    }

  } else if (command == 'f') { // frequency
    // SET Frequency //////////////////////////////////////////////////////////////
    if (isPWM(PWM_Pin)) {
      float newFrequency = value.toFloat();
      Serial.printf("Desired Frequency: %f\n", newFrequency);
      float idealFrequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
      if (newFrequency <= idealFrequency) {
        PWM_Frequency = newFrequency;
        analogWriteFrequency(PWM_Pin, PWM_Frequency);
        Serial.printf("Frequency set to: %f\n", PWM_Frequency);
      } else {
        Serial.println("Frequency to high. Not changed.");
      }
    } else {
      Serial.print("Can not change frequency on soft PWM pin: ");
      Serial.println(PWM_Pin);
      PWM_Frequency = 60.0;
    }

  } else if (command == 'r') { // resolution of pulse width
    // SET Resolution //////////////////////////////////////////////////////////////
    PWM_Resolution = value.toInt();
    if (isPWM(PWM_Pin)) {
      if ((PWM_Resolution < 2) || (PWM_Resolution > 15)) {
        Serial.println("PWM Resolution out of valid Range.");
      } else {
        analogWriteResolution(PWM_Resolution);
        Serial.printf("PWM Resolution set to: %2d\n", PWM_Resolution);
        PWM_Max_Value = pow(2, PWM_Resolution)-1;
        Serial.printf("PWM Max Value: %5d\n", PWM_Max_Value);
        if (PWM_Frequency > GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
          PWM_Frequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
          analogWriteFrequency(PWM_Pin, PWM_Frequency);
          Serial.printf("PWM Frequency adjusted to: %f\n", PWM_Frequency);
        }
      }
    } else {
      Serial.printf("Can not change resolution on soft PWM pin: %2d\n", PWM_Pin);
      PWM_Resolution = 8;
      Serial.printf("PWM Resolution: %2d\n", PWM_Resolution);
    }

  } else if (command == 'p') { // choose pin
    // Choose the pin //////////////////////////////////////////////////////////////
    PWM_Pin = (uint8_t)(value.toInt());
    if (isPWM(PWM_Pin)) {
    Serial.println("Resetting the pin to max frequency and current duty cycle and resolution");
      PWM_Frequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
      pinMode(PWM_Pin, OUTPUT);
      analogWriteFrequency(PWM_Pin, PWM_Frequency);
      analogWrite(PWM_Pin, (unsigned int)(PWM_Duty / 100.0 * float(PWM_Max_Value)));
      Serial.printf("Changed PWM pin: %2d\n", PWM_Pin);
      Serial.printf("PWM Frequency: %f\n", PWM_Frequency);
      Serial.printf("PWM Duty: %4.3f\n", PWM_Duty);
    } else if (isSoftPWM(PWM_Pin)) {
      pinMode(PWM_Pin, OUTPUT);
      SoftPWMSet(PWM_Pin, 0);
      SoftPWMSetFadeTime(PWM_Pin, 100, 200); // fade-up, fade-down speed
      SoftPWMSetPercent(PWM_Pin, PWM_Duty);
      Serial.printf("Changed PWM pin: %2d\n", PWM_Pin);
      Serial.printf("PWM Duty: %4.3f\n", PWM_Duty);
    } else {
      Serial.println("Pin not available for PWM.");
    }

  } else if (command == 's') { // turn off streaming
    // ON/OFF STREAMING REPORT //////////////////////////////////////////////////////////////
    STREAM = false;
  } else if (command == 'S') { // turn on  streaming
    STREAM = true;

  } else if (command == 'V') { // send verbose
    // ON/OFF VERBOSE MODE //////////////////////////////////////////////////////////////
    VERBOSE = true;
  } else if (command == 'v') { // send HEX
    VERBOSE = false;

  } else if (command == '\n') { // ignore
    // Ignore Carriage Return //////////////////////////////////////////////////////////////

  } else if ((command == '?') || (command == 'h')) { // send HELP information
    // HELP //////////////////////////////////////////////////////////////
    printHelp();
    STREAM = false;
  }
} // end process instruction

// CONVERT FLOAT TO HEX AND SEND OVER SERIAL PORT
void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for (int i = 3; i >= 0; i--) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial.print(c1);
    Serial.print(c2);
  }
}

// CONVERT BYTE TO HEX AND SEND OVER SERIAL PORT
void serialBytePrint(byte b) {
  byte b1 = (b >> 4) & 0x0f;
  byte b2 = (b & 0x0f);

  char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
  char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

  Serial.print(c1);
  Serial.print(c2);
}

// CONVERT LONG TO HEX AND SEND OVER SERIAL PORT
void serialLongPrint(unsigned long l) {
  byte * b = (byte *) &l;
  for (int i = 3; i >= 0; i--) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial.print(c1);
    Serial.print(c2);
  }
}

// CONVERT INT TO HEX AND SEND OVER SERIAL PORT
void serialIntPrint(unsigned int I) {
  byte * b = (byte *) &I;
  for (int i = 1; i >= 0; i--) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial.print(c1);
    Serial.print(c2);
  }
}
