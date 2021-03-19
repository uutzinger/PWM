////////////////////////////////////////////////////////////////////////////
//
//  This file is part of my Teensy Template
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  DISPLAY_INTERVAL sets the rate at which results are transferred to host
#define DISPLAY_INTERVAL      100       // interval in microseconds between data updates over serial port: 100000=10Hz
//  CHECKINPUT_INTERVAL sets the rate at which serial input is checked from host
#define CHECKINPUT_INTERVAL   50000     // 50ms
//  LEDBLINK_INTERVAL sets the rate at which the built in LED blinks
#define LEDBLINK_INTERVAL     100000    // 100ms
//  POLL_INTERVAL
#define POLL_INTERVAL          10000    // 10ms
//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  2000000
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
bool REPORT=false;                             // Do we need to produce output data?
bool STREAM=true;                              // Are we continously streaming output?
bool checkINAVAIL=false;                            // Is it time to check for user input ?
bool VERBOSE=false;                            // Set HEX streamn or text stream
bool SERIAL_REPORTING = false; 	        			 // Boot messages on/off
bool DEBUG_ENABLE = false;                     // Debug on/off 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Indicators
const int ledPin = 13;                         // Check on https://www.pjrc.com/teensy/pinout.html; pin should not interfere with I2C and SPI
bool ledStatus = false;                        // Led should be off at start up

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PWM pins  
// FTM1 3, 4, 
// FTM0 5, 6, 9, 10, 20, 21, 22, 23, 
// FTM2 25, 32
// Software PWM pins 
// All remaining pins 0,1,2,7,8,11-18,19,24-31,33,
unsigned int  PWM_Pin        = 5;
long          CPU_Frequency  = F_CPU/1E6;
long          PWM_Frequency  = 488;
unsigned int  PWM_Resolution = 8;
unsigned int  PWM_Max_Value  = pow(2,PWM_Resolution); 
float         PWM_Duty       = 50.0;
/////////////////////////////////////////////////////////
float GetMaxPWMFreqValue(long FREQ, int PWM_Resolution)
{
  /* for Teensy CPU frequency 
   *  24MHZ
   *  48MHZ
   *  72MHZ
   *  96MHZ
   *  120MHZ
   */
  int FREQ_pointer=-1;
  float PWM_ideal_frequency[5][15]
  {
    {6000000 ,3000000,1500000,750000 ,375000,187500,93750 ,46875   ,23437.5 ,11718.75 , 5859.375,2929.687,1464.843, 732.421,366.2109},
    {12000000,6000000,3000000,1500000,750000,375000,187500,93750   ,46875   ,23437.5  ,11718.75 ,5859.375,2929.687,1464.843,732.4218},
    {9000000 ,4500000,2250000,1125000,562500,281250,140625,70312   ,35156.25,17578.12 , 8789.062,4394.531,2197.265,1098.632,549.3164},
    {12000000,6000000,3000000,1500000,750000,375000,187500,93750   ,46875   ,23437.5  ,11718.75 ,5859.375,2929.687,1464.843,732.4218},
    {15000000,7500000,3750000,1875000,937500,468750,234375,117187.5,58593.75,29296.875,14648.537,7324.219,3662.109,1831.055,915.527 }
  };
  
  switch(FREQ){
    case 24:
    FREQ_pointer=0;
    break;
    case 48:
    FREQ_pointer=1;
    break;
    case 72:
    FREQ_pointer=2;
    break; 
    case 96:
    FREQ_pointer=3;
    break; 
    case 120:
    FREQ_pointer=4;
    break;          
    default:
    FREQ_pointer=-1;
    break;          
  }
  if (FREQ_pointer >= 0) {  
    return(PWM_ideal_frequency[FREQ_pointer][PWM_Resolution-2]);
  } else {
    return(488.28);
  }
} // end of getMaxPWMFreqValue

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial
char   inBuff[] = "----------------";
int    bytesread;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // Setup monitor pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  ledStatus = false;

  // Setup PWM
  PWM_Frequency=GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
  analogWriteFrequency(PWM_Pin,PWM_Frequency);
  analogWriteResolution(PWM_Resolution); 
  PWM_Max_Value=pow(2,PWM_Resolution); 
  analogWrite(PWM_Pin,PWM_Max_Value/2);
  
  Serial.begin(SERIAL_PORT_SPEED);
  while (!Serial) {
      ; // wait for serial port to connect. 
  }
  if (SERIAL_REPORTING) {Serial.println("Starting PWM");}
  
  pollInterval = 1000; // 1 m sec
    
  // House keeping & Initializing
  lastDisplay = lastRate = lastTimestamp = lastInAvail = micros();
  mainCount = 0;

  // Ready for input commands
  if (SERIAL_REPORTING) { Serial.println("Send S to start streaming."); }

  // LED blinking status
  digitalWrite(ledPin, HIGH); // initialization completed
  ledStatus = true;
  lastBlink = micros();

  // setup timers
  lastPoll = micros();
  timeout= (unsigned long) (50*(pollInterval));

  if (SERIAL_REPORTING) { Serial.println("System ready."); }

} // END setup

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{  
  // Main loop delay. 
  // Limit the how often the main loop is running  
  currentTime = micros();                                         // whats the time
  int pollDelay = (pollInterval - (int)(currentTime - lastPoll)); // how long do we need to wait?
  if (pollDelay > 0) { delayMicroseconds(pollDelay); }            // wait
  lastPoll = currentTime;

  // check if system stalled
  if ( (currentTime - lastPoll) > timeout ) {
    // The system stalled and can not keep up with requested main loop interval
    Serial.println("!!!!!!!!!!!!!!!!!!!! RESET SYSTEM: Main loop taking too long !!!!!!!!!!!!!!!!!!!!");
    lastPoll = currentTime = micros();
  }
  
  // main Loop Counter
  mainCount++;

  /////////////////////////////////////////////////////////////
  // Compute update rates in Hz
  elapsedTime = currentTime - lastRate;
	if (elapsedTime >= 1000000) { // 1 second = 1000000 us
		lastRate = currentTime;
		sampleRate = (mainCount*1000000)/elapsedTime;
		mainCount = 0;
	}
    
  /////////////////////////////////////////////////////////////
	// Do we want to display data?
  // Expand
	if ((currentTime-lastDisplay) >= DISPLAY_INTERVAL) {
	  lastDisplay = currentTime;
	  if (STREAM) { REPORT = true; } else { REPORT = false; }
	}

  /////////////////////////////////////////////////////////////
	// Do we want to check input ?
	if ((currentTime-lastInAvail) >= CHECKINPUT_INTERVAL) {
	  lastInAvail = currentTime;
	  checkINAVAIL = true;
	} else {
	  checkINAVAIL = false;
	}

	///////////////////////////////////////////////////////////////
	// Data Reporting
	if (REPORT) {
	  if (VERBOSE) { // human readable reporting
		  Serial.println("-Data----");
		  Serial.printf("Frequency: %+4.3f\n", PWM_Frequency);
      Serial.printf("Duty: %+4.3f\n", PWM_Duty);
      Serial.printf("Resolution: %2d\n", PWM_Resolution);
      Serial.printf("Pin: %2d\n", PWM_Pin);
      Serial.printf("CPU: %2d\n", CPU_Frequency);
      Serial.printf("PWM Max: %4d\n" ,PWM_Max_Value);
		  Serial.println("-System--");
		  Serial.printf("Sample rate: %3d", sampleRate);
   	  Serial.print("Data Transmission Time [us]: ");
		  Serial.println((micros()-currentTime)); //
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
	
  ///////////////////////////////////////////////////////////////
  // Input Commands

  if (checkINAVAIL) {
    if (Serial.available()) {
       bytesread = Serial.readBytesUntil('\n', inBuff, 16); // Read from serial until CR is read or timeout exceeded
       inBuff[bytesread]='\0';
       String instruction = String(inBuff);
       processInstruction(instruction);
    }
  }

  ///////////////////////////////////////////////////////////////
  // Blink LED 
	
  // Keep LED blinking when system is ready
  if ((currentTime - lastBlink) > LEDBLINK_INTERVAL) {
      if (ledStatus == false) {
        digitalWrite(ledPin, HIGH);
        ledStatus = true;       
      } else {
        digitalWrite(ledPin, LOW);
        ledStatus = false;
      } // flip LED
      lastBlink = currentTime;
  } else {
    if (ledStatus == false) {  
       digitalWrite(ledPin, HIGH);
       ledStatus = true;
    }  // keep LED on but no blinking
  } // System blinking
    
} // end of main loop

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Support Routines
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// What are the software options?
void printHelp() {
  Serial.println("--HELP-------------------------------------------");
  Serial.println("PWM Controller");
  Serial.println("--HELP-------------------------------------------");
  Serial.println("m/M to disable/enable PWM");
  Serial.println("s/S to disable/enable data streaming");
  Serial.println("v/V to disable/enable human readable data display");
  Serial.println("--HELP Data Input--------------------------------");
  Serial.println("Enter number then ");
  Serial.println("p   to set output pin");
  Serial.println("d   to set duty cyle in %");
  Serial.println("f   to set frequency");
  Serial.println("r   to set resolution in # of bit");
  Serial.println("-------------------------------------------------");
  Serial.println("Press S to continue. Streaming was turned off to hold this diplay.");
  Serial.println("--HELP END---------------------------------------");
  Serial.println("Urs Utzinger, 2019,20,21");
  Serial.println("--HELP END---------------------------------------");
}

void processInstruction(String instruction) {
  String value    = "0.01";
  String command  = "o";
  float  tempFloat;
  long   tempInt;
  int instructionLength = instruction.length();
  if (instructionLength > 0) { command = instruction.substring(0,1); } 
  if (instructionLength > 1) {   value = instruction.substring(1,instructionLength); }

  // ENABLE/DISABLE PWM //////////////////////////////////////////////////////////
  if        (command == 'm') { // turn off PWM
    analogWrite(PWM_Pin, 0);
  } else if (command == 'M') { // turn on PWM
    analogWrite(PWM_Pin, (unsigned int)(PWM_Duty/100.0 * float(PWM_Max_Value)));
  // SET DUTY CYCL //////////////////////////////////////////////////////////////
  } else if (command == 'd') { // duty cycle
    PWM_Duty=value.toFloat();
    if ((PWM_Duty < 0.0) || (PWM_Duty > 100.0)) { 
      Serial.println("Duty cyle out of valid Range"); 
    } else {
      analogWrite(PWM_Pin, (unsigned int)(PWM_Duty/100.0 * float(PWM_Max_Value)));
    }
  // SET Frequency //////////////////////////////////////////////////////////////
  } else if (command == 'f') { // frequency
    float newFrequency = value.toFloat();
    float idealFrequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
    if (newFrequency <= idealFrequency) { 
      // adjust frequency only if resolution matches
      analogWriteFrequency(PWM_Pin,PWM_Frequency);
    } else {
      Serial.println("Frequency to high"); 
      // dont change frequency when its higher than the maximum possible for current resolution
    }
  // SET Resolution //////////////////////////////////////////////////////////////
  } else if (command == 'r') { // resolution
    PWM_Resolution=value.toInt();
    if ((PWM_Resolution < 2) || (PWM_Resolution > 15)) { 
      Serial.println("PWM Resolution out of valid Range"); 
    } else {
      analogWriteResolution(PWM_Resolution); 
      PWM_Max_Value=pow(2,PWM_Resolution); 
      if (PWM_Frequency > GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution)) {
        PWM_Frequency = GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
        analogWriteFrequency(PWM_Pin,PWM_Frequency);
        Serial.println("Reduced Frequency to match Resolution"); 
      }
    }
  // Choose the pin //////////////////////////////////////////////////////////////
  } else if (command == 'p') { // choose pin
    PWM_Pin=(unsigned int)(value.toInt());
    if ( (PWM_Pin==3) || (PWM_Pin==4) || (PWM_Pin==5)  | (PWM_Pin==6) || (PWM_Pin==9) || (PWM_Pin== 10) || (PWM_Pin==20) || (PWM_Pin==21) || (PWM_Pin==22) || (PWM_Pin==23) || (PWM_Pin==25) || (PWM_Pin==32)) {
      Serial.println("Resetting the pin to max frequency and current duty cycle and resolution");
      PWM_Frequency=GetMaxPWMFreqValue(CPU_Frequency, PWM_Resolution);
      pinMode(PWM_Pin, OUTPUT);
      analogWriteFrequency(PWM_Pin,PWM_Frequency);
      analogWrite(PWM_Pin,(unsigned int)(PWM_Duty/100.0 * float(PWM_Max_Value)));
    } else {
      Serial.println("Pin not available for PWM");
    }
  // ON/OFF STREAMING REPORT //////////////////////////////////////////////////////////////
  } else if (command == 's') { // turn off streaming
    STREAM=false;
  } else if (command == 'S') { // turn on  streaming
    STREAM=true;
  // ON/OFF VERBOSE MODE //////////////////////////////////////////////////////////////
  } else if (command == 'V') { // send verbose
    VERBOSE=true;
  } else if (command == 'v') { // send HEX
    VERBOSE=false;
  // Ignore Carriage Return //////////////////////////////////////////////////////////////
  } else if (command == '\n') { // ignore
  // HELP //////////////////////////////////////////////////////////////
  } else if ((command == '?') || (command == 'h')) { // send HELP information
    printHelp();
    STREAM=false;
  }
} // end process instruction



// CONVERT FLOAT TO HEX AND SEND OVER SERIAL PORT
void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=3; i>=0; i--) {
    
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
  for(int i=3; i>=0; i--) {
    
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
  for(int i=1; i>=0; i--) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}
