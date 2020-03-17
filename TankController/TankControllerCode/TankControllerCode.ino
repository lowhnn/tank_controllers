/*

TANK CONTROLLER CODE

This is code that accompanies the manuscript:

Low NHN, Ng CA, Micheli F. A low-cost modular control system for multi-stressor experiments.

Photos, diagrams, schematics, component descriptions and part numbers, and a general description of the 
controller's functions are all available in the manuscript and stored on the Github repository 
(github.com/lowhnn/tank_controllers). 



//////////////////////////////////////////////////////////////////////////////////////
ABOUT THE HARDWARE:

The Tank Controller Shield is designed to work with a standard Arduino UNO board.

It contains the following components:

 - DS3234 Real Time Clock (http://datasheets.maximintegrated.com/en/ds/DS3234.pdf) 
 - Battery holder for a CR1220 3V coin battery to power the clock
 - Micro SD card reader with a LP2985 3V voltage regulator (http://www.ti.com/lit/ds/symlink/lp2985-n.pdf) 
   and a HC4050M logic shifter to convert TTL and CMOS (http://www.ti.com/lit/ds/symlink/cd74hc4049.pdf)
 - ADS1118 4-channel, 16-bit ADC (http://www.ti.com/lit/ds/symlink/ads1118.pdf)
 - ULN2003A transistor array (http://www.ti.com/lit/ds/symlink/uln2003a.pdf)
 - A toggle switch to switch the ramp cycle on
 - 2 LEDs to indicate (a) whether the ramp is on; and (b) if there is an error with the SD card
 - Screw terminals for connecting sensor, actuator, power, and I2c communication wires to the board


Notes on SPI devices:
 - The RTC, SD card reader, and ADC are on the SPI bus and use pins D8-D13 on the Arduino. 
 - The DS3234 RTC runs in SPI Mode 3 (it can theoretically also run in Mode 1) while the other 2 components
   run in SPI Mode 0. The getRTCTime() function automatically switches between SPI modes when communicating 
   with the RTC.
 - The ADS1118 ADC is connected to the 'S' ("signal") -marked channels on the screw terminals JP1-4. Temperature,
   pH, and dissolved oxygen signal channels correspond to channels 0, 1, and 2 on the ADC, respectively.

Notes on actuators:
 - There are four channels for actuators, connected to the ULN2003A relay and controlled by the four digital 
   output pins D4-D7. Actuators are connected to the board via the screw terminals JP5-8. Pins D5 and D6 have
   built-in pulse width modulation (PWM) capabilities. These were originally assigned to the temperature control
   actuators (TC1 and TC2, heater and chiller) with the intent to ustilize the Arduino's PWM, but testing showed
   that the built-in PWM operates at too high a frequency for effective control; so a pulse type response was
   implemented using the function pulseActuator() and the duty cycle variables.


Notes on power:
 - There are two screw terminals (JP9-10) for connection to DC power supplies. JP10 (marked MCU) supplies power
   to the Arduino and Arduino and all board components except the ULN2003 relay and connected actuators. This 
   should be connected to a 9-12VDC power supply (we have used a 12V, 1.5A supply). JP9 (marked ACT) powers the
   ULN2003 and actuators. This should be connected to a power supply that is sufficient for the actuators used.
   The separate power terminals allows for higher-voltage actuators (e.g., 16- or 24-V solenoids) to be used
   with the controller even though the maximum voltage for the Arduino is 12V.

 Notes on I2C:
 - The screw terminal JP11 allows for connection to a "Master" Arduino, or to an I2C-enabled LCD screen. 
 
 
 List of Arduino UNO/Atmega328 pins used by the shield and the code:
  A0 - Toggle switch for turning on the ramp. Has external pulldown so on = HIGH
  A1 - Indicator LED for SD errors
  A2 - Indicator LED for active ramp
  A4 - internally connected to SDA pin
  A5 - internally connected to SCL pin
  D4 - SOL2 (CO2 solenoid)
  D5 - TC2 (chiller)
  D6 - TC1 (heater)
  D7 - SOL1 (N2 solenoid)
  D8 - ADC chip select for SPI
  D9 - SD chip select for SPI
  D10 - RTC chip select for SPI
  D11 - standard SPI MOSI pin
  D12 - standard SPI MISO pin
  D13 - standard SPI clock pin

  
//////////////////////////////////////////////////////////////////////////////////////
ABOUT THE CODE:

These tank controller Arduinos have been designed to both regulate and record the values of
temperature, DO, and pH in the tank.

This is a quick overview of what the code does:

(1) On startup, input/output pins are defined, serial communications and SPI devices (SD card, ADC,
    RTC) are initialised.
    
(2) The getSDSettings() function retrieves parameters from text files on the SD card:
    - It retrieves its 1-2 digit tank ID number from the TANKID.TXT file and assigns itself the 
      corresponding I2C address. It also assigns the right filenames for the ramp file (RAMPxx.TXT)
      and the log file (LOGxx.TXT), where xx = the tank ID number.
    - It retrieves the number of lines (possible values 1-999) in the ramp file from the RAMPLEN.TXT
      file for use in running/tracking the ramp schedule
    - It retrieves the current ramp line position from the RAMPPOS.TXT file to determine which line
      on the ramp to start from. If running a new experiment this value is set to 1. If the Arduino
      resets in the middle of an experiment (e.g., power loss) this allows the ramp to start from 
      where it left off. 
    - It retrieves sensor calibration coefficients from the files TCAL.TXT, DOCAL.TXT, and PHCAL.TXT 
      for use in converting the ADC output to tank variables. See the calibration sample code on the 
      Github repository.
      
    Note: Sample text files for the SD card and a readme are available on the Github repository.

(3) [optional based on user selection] - I2C communications to another Arduino or an I2C-compatible 
    LCD screen are initialised.

(4) Initialises variables for timekeeping of datalogging and visual feedback functions (sdPoint,
    serialPoint, lcdPoint).

(5) Enters a loop, which continues indefinitely until the ramp is initiated via the switch. 
    - The getRTCTime() function reads the current time from the real-time clock.
    - The readSensors() function reads the sensor input values from the ADC and converts them to
      current temperature, DO, and pH values using the stored sensor coeffients.
    - The logAndReport() function logs data to the SD card and reports it to the serial monitor
      and (if selected) the LCD screen, based on the timing loops user-defined by the sdInterval, 
      serialInterval, and lcdInterval variables.
    - If the I2C_MASTER option has been selected, a request from the Master Arduino will trigger
      the requestEvent() function, which sends current sensor values and ramp actuator statuses
      to the Master Arduino via the I2C connection.

(6) When the ramp switch is flipped, the ramp indicator LED is turned on and the program enters
    a new loop, which continues until all lines in the ramp have been completed.
    - The getRampLine() function reads the current line, as specified in RAMPPOS.TXT, from the 
      ramp file (RAMPxx.TXT). It parses the comma-separated values and stores the timepoint
      and setpoints for temperature, DO, and pH.
    - The getRTCTime() function reads the current time from the real-time clock.
    - The readSensors() function reads the sensor input values from the ADC and converts them to
      current temperature, DO, and pH values using the stored sensor coeffients.
    - The updateActuators() function
    - The logAndReport() function logs data to the SD card and reports it to the serial monitor
      and (if selected) the LCD screen, based on the timing loops user-defined by the sdInterval, 
      serialInterval, and lcdInterval variables.
    - If the I2C_MASTER option has been selected, a request from the Master Arduino will trigger
      the requestEvent() function, which sends current sensor values and ramp actuator statuses
      to the Master Arduino via the I2C connection.
    

(7) When the program has run through all the lines in the ramp file, it turns off the ramp indicator
    LED and all the actuators, and enters a loop that is identical to the loop described in (5). This 
    post-ramp loop runs indefinitely.

//////////////////////////////////////////////////////////////////////////////////////

The code is open-source and free to be adapted and modified as desired. Here are some notes for making 
the most likely modifications we anticipate:

(1) I2C Communications: 
    Code is provided for I2C communication with a LCD screen or for I2C communication with a "Master"
    Arduino - set the variables 'LCD_I2C' and 'MASTER_I2C' to 0/1 respectively and comment/uncomment out
    the relevant sections of code as indicated.

(2) Logging Frequency: 
    Current frequency of logging values to the SD card is 5 minutes (300000 ms) - this can be modified
    in the variable 'sdInterval'. If the LCD is being used, LCD update frequency can be modified with the
    variable 'lcdInterval'. If using with a Master Arduino, the Master Arduino will control the frequency of
    communication.

(3) Actuator duty cycles: 
    Depending on tank size, actuator type, and other equipment characteristics, parameters of the actuator
    duty cycles may be modified to finetune control of tank variables. The length (in seconds) of the duty
    cycle can be altered in the input arguments of the pulseActuator() functions in the ramp loop, and the 
    duty cycle values themselves can also be altered in the code for the updateActuators() function at the 
    bottom of the sketch.
    
(4) Sensor coefficients:
    The default code assumes linear, 2-coefficient (intercept/slope) sensor calibration curves. To use sensors
    with more than 2 calibration coefficients , the following changes should be made:
     -  Modify the length of the array variable(s), tSens[], doSens[], pHSens[] to accomodate the number
        of coefficients (current lengths are 2)
     -  Modify code within the function getSensorCoef() to read a third coefficient value from the appropriate
        SD card file. Sample code (commented out) is provided.
     -  Modify code within the function readSensors() to use the parameters for conversion of ADC readings into
        sensor readings. Sample code (commented out) is provided for using 3 parameters for a thermistor.
     - Enter the appropriate number of parameters in the sensor coefficient files on the SD card (TCAL.TXT, 
       DOCAL.TXT, PHCAL.TXT)
    


*/



////////////////////////////////////////////////////////////////////////////////////////
// CORE LIBRARIES
#include <Wire.h>
#include <SdFat.h>
#include <SPI.h> 
#include <RTC_DS3234.h>

////////////////////////////////////////////////////////////////////////////////////////
// Tank-specific and sensor-specific variables
// These will be defined by the settings file on the SD card

byte myAddress; // I2C address of this Arduino
char logFile [10]; // name of the text file to log to
char rampFile [11]; // name of the text file with ramp values
int rampLength; // number of lines in ramp file
float tSens [3], doSens [2], phSens [2]; // sensor regression coefficients


////////////////////////////////////////////////////////////////////////////////////////
// DEFINE PINS

  // Actuator pins
  #define solN2Pin 7 // SOL1
  #define heaterPin 6 // TC 1
  #define chillerPin 5 // TC 2 
  #define solCO2Pin 4 // SOL2 
  
  // SPI devices
  #define adcCSPin 8
  #define RTCPin 10
  #define SDcardPin 9 
  
  // Ramp switch and indicator
  #define switchPin A0
  #define sdLEDPin A1
  #define rampLEDPin A2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// Define ADC Channels
#define TempCh 0
#define DOCh 2
#define pHCh 1


// Instantiate SPI devices
SdFat sd;
SdFile myFile;
RTC_DS3234 RTC(RTCPin);


// Store SD error messages in flash memory 
#define error(s) sd.errorHalt_P(PSTR(s))


////////////////////////////////////////////////////////////////////////////////////////
// DEFINE GLOBAL VARIABLES

  // Variable to store the next line position in the ramp file
  int lineToLoad; 

  // Variables to store sensor readings
  float currentTemp, currentDO, currentpH;

  // Variables to store ramp and actuator statuses
  boolean rampStat, chillerStat, heaterStat, N2Stat, CO2Stat;

  // Variables to store ramp values
  float timeMinutes, tempMax, tempMin, DOMax, DOMin, pHMax, pHMin; 
  
  // Variables for timekeeping 
  unsigned long rampStartTime; // variable for ramp timekeeping

  unsigned long serialPoint;
  int serialInterval = 10000; // reports to serial every 10 sec 
  
  unsigned long sdPoint;
  unsigned long sdInterval = 300000; // Logs to SD card every 5 min

  // Variable for storing RTC time
  DateTime now;

  // Variables to store duty cycles for actuators
  // Not all actuators have to be run on duty cycles; 
  float chillerDutyCycle, heaterDutyCycle, N2bubbleRate, CO2bubbleRate;



////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// Options to run or not run the code for communicating with a LCD screen (LCD_I2C)
// or with another Arduino that serves as a 'master' to consolidate data from multiple
// controllers (see sample code: github.com/lowhnn/XXXXXX)
// Both of these communications use the I2C protocol and involve wired connections to
// the screw connector JP11

#define LCD_I2C 0     // this is 1 if using with an LCD screen, 0 if not
#define MASTER_I2C 1    // this is 1 if using with a master Arduino, 0 if not  

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// For I2C communication with a "Master" Arduino
// see xxxxxxxxxxx for 
// Comment out this entire section if not using a Master Arduino setup

// Define union with of one variable represented as float and byte array
// this is for conversion of float sensor values to bytes during I2C transmission
typedef union float2bytes_t { 
  float f; 
  byte b[sizeof(float)]; 
}; 

// Variable to store incoming commands from Master
  char command = 0;

// Define list of possible commands from Master Arduino, coded as bytes
  #define VALUE_TEMP 1
  #define VALUE_DO 2
  #define VALUE_PH 3
  #define STATUS_RAMP 4
  #define STATUS_SOL_DO 5
  #define STATUS_SOL_CO2 6
  #define STATUS_HEATER 7
  #define STATUS_CHILLER 8

// Function to receive and store data-request command from Master Arduino 
// via the I2C connection.
void receiveEvent (int howMany){
  command = Wire.read ();  // remember command for when we get request
}  // end function receiveEvent

// Function to send the requested data to Master over the I2C connection, based on 
// the command received and stored by the receiveEvent() function. Calls the 
//  sendSensor() function.
void requestEvent (){
  switch (command){
    case VALUE_TEMP: sendSensor(currentTemp); break;// send Temperature reading
    case VALUE_DO: sendSensor(currentDO); break;   // send DO reading
    case VALUE_PH: sendSensor(currentpH); break; // send pH reading
    case STATUS_SOL_DO: Wire.write(N2Stat); break; // send N2 solenoid status
    case STATUS_SOL_CO2: Wire.write(CO2Stat); break; // send CO2 solenoid status
    case STATUS_HEATER: Wire.write(heaterStat); break; // send heater status
    case STATUS_CHILLER: Wire.write(chillerStat); break; // send chiller status
    case STATUS_RAMP: Wire.write(rampStat); break; // send ramp status
  }
} // end function requestEvent

// Function to convert a floating-point sensor value into a byte array to send
// to the Master via the I2C connection. Called by the requestEvent() function.
void sendSensor(float floatSensor){
  float2bytes_t f2bSensor;
  byte bytesSensor [4];
  f2bSensor.f = floatSensor; 
  Wire.write(f2bSensor.b,4);
} // end function sendSensor


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

// For I2C communication with an LCD screen
// see xxxxxxxxxxx for 
// Comment out entire section if not using directly with an LCD screen

//#include <LiquidCrystal.h>
//
//// Instantiate LCD
//LiquidCrystal lcd(0);
//
//// define variables for LCD refresh timekeeping
//unsigned long lcdPoint;
//int lcdInterval = 5000; // refreshes the LCD every 5 sec
//
//// Function to display/refresh data on the LCD
//// This is set up to work with both 16x2 and 20x4 LCDs; modify as desired
//void lcdReport() {
//  lcd.clear();
//  lcd.setCursor(0, 0); lcd.print(currentTemp,1);lcd.print((char)223);lcd.print(F("C"));
//  if (heaterStat==1) lcd.print(F("+"));
//  else if (chillerStat==1) lcd.print(F("-"));
//  lcd.setCursor(8, 0); lcd.print(currentpH,2);lcd.print(F("pH"));
//  if (CO2Stat==1) lcd.print(F("-"));
//  lcd.setCursor(0, 1); lcd.print(currentDO,1);lcd.print(F("mg/L"));
//  if (N2Stat==1) lcd.print(F("-")); 
//}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  //////////////////////////////////////////////////////////////////////////////////////
  // SETUP FUNCTIONS
  // 
     
  // Pin setup
    // Output Pins for actuators
    pinMode(solN2Pin, OUTPUT);
    pinMode(heaterPin, OUTPUT);
    pinMode(chillerPin, OUTPUT);
    pinMode(solCO2Pin, OUTPUT);

    // Pins for ramp switch and indicator LEDs
    pinMode(switchPin, INPUT); 
    pinMode(rampLEDPin, OUTPUT);
    pinMode(sdLEDPin, OUTPUT);
    
    // Pins for SPI devices
    pinMode (adcCSPin, OUTPUT);
    digitalWrite(adcCSPin, HIGH); 
    
  // Initialize serial communications for testing and debugging code
    Serial.begin(9600);
    
  // Initialise SD card 
  if (!sd.begin(SDcardPin, SPI_HALF_SPEED)) {
    digitalWrite(sdLEDPin,HIGH);
    Serial.println(F("Failed to initialise SD card."));
  }
   
  // Begin standard SPI for ADC
  SPI.begin();
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE0); 
  SPI.setClockDivider(SPI_CLOCK_DIV16); 

  // Initialise RTC
  startRTC();

   // Load settings from SD card
  getSDSettings();

  // Begin I2C communication with Master Arduino (if using)
  #if MASTER_I2C
    Wire.begin (myAddress); // begin I2C
    Wire.onReceive (receiveEvent);  // interrupt handler for incoming messages
    Wire.onRequest (requestEvent);  // interrupt handler for when data is wanted
  #endif

  //Initialise LCD (if using)
  #if LCD_I2C
      lcd.begin(20,4); // initialise a 20x4 LCD
      //lcd.begin(16,2); // initialise a 16x2 LCD
      lcd.setBacklight(HIGH);
      lcd.print(F("Starting..."));
  #endif 
  
    
  // Initialise timepoints for timekeeping of SD card logging, serial reporting,
  // and LCD refresh (if LCD is in use)
  sdPoint = millis();
  serialPoint = millis();
  #if LCD_I2C
      lcdPoint = millis();
  #endif 
  
  //////////////////////////////////////////////////////////////////////////////////////
  // START BASIC PRE-RAMP LOOP 
  // This loop reads sensors, logs and reports data at the specified time intervals
  // It runs until the ramp switch is turned on to initialise treatment start

  while (digitalRead(switchPin)==LOW) { // while ramp switch is off

    // Keep green ramp indicator LED off
    digitalWrite(rampLEDPin, LOW); 

    // Ramp status is off
    rampStat = 0;
    
    // Read the current time from RTC
    getRTCTime(); 

    // Read sensor values from ADC
    readSensors(); 

    // Log and report data to SD card, serial monitor, and LCD (if in use),
    // if the specified time intervals for those functions have elapsed
    logAndReport();


      delay(500);  // I2C communication with Master is improved with a short delay here
                  // increase the delay time if having I2C issues 
      
      
  } // end basic pre-ramp loop when ramp switch is flipped

  //////////////////////////////////////////////////////////////////////////////////////
  // START OF TREATMENT RAMP 
  // Initialises the ramp start time and then enters the ramp loop.
  // Ramp loop: For each line in the ramp, retrieve the timepoint and all variable setpoints. 
  // For timepoint duration, turn on/off the actuators to regulate variables, log and report data.
  // When timepoint is reached, retrieve new line from the ramp and repeat with new setpoints.
  
  // Store ramp start time 
  rampStartTime = millis(); 

  // Turn on green ramp indicator LED
  digitalWrite(rampLEDPin, HIGH); 

  // Ramp status is on
  rampStat = 1;
  
} // end setup()


void loop() {

  // Loop to run while the ramp is on (i.e. the program hasn't reached the last line in the ramp file)
  while (lineToLoad <= rampLength) { 

    // Get values from the next line from the ramp file
    getRampLine(); 
    Serial.print(F("Ramp timepoint: "));Serial.println(timeMinutes);
    
    
    // Loop to run while the next time-point in the ramp has not been reached
    // This reads sensors and regulates variables against the setpoints in the current ramp line
    while (millis()-rampStartTime < timeMinutes*60000) {
      
      // Read the current time from RTC
      getRTCTime(); 

      // Read sensor values from ADC
      readSensors(); 
      
      // Check tank values against the current setpoint and turn actuators on/off or define duty cycles
        // To fine-tune actuator responses, duty cycle values can be altered in the code for the 
        // updateActuators() function at the bottom of the sketch
      updateActuators(); 

      // Run duty cycles for actuators
        // The third argument for pulseActuator() is the length (seconds) of the duty cycle
        // This can be altered to fine-tune actuator response
      pulseActuator(heaterPin, heaterDutyCycle, 1);
      pulseActuator(solN2Pin, N2bubbleRate, 1);
      pulseActuator(solCO2Pin, CO2bubbleRate, 3);      

      // Log and report data to SD card, serial monitor, and LCD (if in use),
      // if the specified time intervals for those functions have elapsed
      logAndReport();

      delay(500); // delay to allow I2C communication with Master

    } // end of one time point on ramp (one line of the ramp)

    lineToLoad++; // update the number for the next ramp line to be read
    Serial.print(F("New Ramp Line: "));Serial.println(lineToLoad);

    updateRampPos(); // write the new lineToLoad value to the SD card
    
    
  } // end of ramp while-loop (the last line is reached)
  
  Serial.println(F("Ramp end."));

  // Turn off green ramp indicator LED; ramp is complete
  digitalWrite(rampLEDPin, LOW); rampStat = 0;
  
  // Turn off all actuators if they are still on
  // Update corresponding indicator boolean variables
  digitalWrite(heaterPin, LOW); heaterStat = 0;
  digitalWrite(chillerPin, LOW); chillerStat = 0;
  digitalWrite(solN2Pin, LOW); N2Stat = 0;
  digitalWrite(solCO2Pin,LOW); CO2Stat = 0;
  
  
  // END OF RAMP
  //////////////////////////////////////////////////////////////////////////////////////
  // START BASIC POST-RAMP LOOP 
  // This is identical to the pre-ramp loop.
  // This loop reads sensors, logs and reports data at the specified time intervals.
  // It runs indefinitely.

  // Read the current time from RTC
  getRTCTime(); 

  // Read sensor values from ADC
  readSensors(); 

  // Log and report data to SD card, serial monitor, and LCD (if in use),
  // if the specified time intervals for those functions have elapsed
  logAndReport();

  delay(500); 

    
} // end loop()


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


// ------------------------------------------------------------------
// ------------------------- FUNCTIONS USED -------------------------
// ------------------------------------------------------------------


// Sub-functions used in the main code, in approximate order of use:
// (functions specific to the optional I2C communications with a Master
//  Arduino or LCD screen are defined above, with the variables relevant 
//  to those communications.)
//
//    getSDSettings
//      getTankID 
//      getRampLength
//      getRampPos
//      loadSensors
//        getSensorCoef
//    startRTC
//    getRTCTime
//    readSensors
//      readADC
//    logAndReport
//      logToSD
//      serialReport
//    getRampLine
//    updateActuators
//    pulseActuators
//    updateRampPos



// Function to retrieve all user-defined settings off the SD card files. 
// Runs the functions getTankID(), getRampLength(), getRampPos(), loadSensors(),
// and reports the retrieved settings to the serial monitor
void getSDSettings() {
  
// Retrieve tank ID
    getTankID(); 
    Serial.print(F("My I2C address: "));Serial.println(myAddress);
    Serial.print(F("Logging to "));Serial.println(logFile);
    Serial.print(F("Loading ramp from "));Serial.println(rampFile);

    // Retrieve length of ramp file
    getRampLength();
    Serial.print(F("Ramp length = "));Serial.println(rampLength);

    // Retrieve current ramp position
    getRampPos(); 
    Serial.print(F("Ramp starts from line: "));Serial.println(lineToLoad);
      
    // Retrieve sensor calibration values
    loadSensors();
    Serial.print(F("Temp intercept: "));Serial.print(tSens[0]);Serial.print(F(", Temp slope: "));Serial.println(tSens[1]);
    Serial.print(F("DO intercept: "));Serial.print(doSens[0]);Serial.print(F(", DO slope: "));Serial.println(doSens[1]);
    Serial.print(F("pH intercept: "));Serial.print(phSens[0]);Serial.print(F(", pH slope: "));Serial.println(phSens[1]);

}


// Function to retrieve the ID number of the tank/controller from the TANKID.TXT file
// on the SD card, and use that to assign this Arduino the correct I2C address and determine
// the filenames of the ramp file and log file to read from/write to
void getTankID()  {
  char readChar;
  char tempAddress [3];
  
  if (!myFile.open("TANKID.TXT", O_READ)) sd.errorHalt("Failed to load tank ID");
  
  while (myFile.available()) { 
    readChar = myFile.read();
    for (int i = 0; i < 3; i++) {
      if (readChar != ';') {
        tempAddress[i] = readChar;
        readChar = myFile.read();
      } else {
        tempAddress[i] = '\0';
        break;
      }
    }
    
    myAddress = atoi(tempAddress);

    // define ramp and log file names based on tank number
    strcpy(rampFile,"RAMP");
    strcpy(logFile,"LOG");

    if (myAddress < 10){
      rampFile[4] = '0';
      rampFile [5] = tempAddress [0];
      logFile[3] = '0';
      logFile[4] = tempAddress [0];
    } else {
      logFile [3] = tempAddress [0];
      logFile [4] = tempAddress [1];
      rampFile [4] = tempAddress [0];
      rampFile [5] = tempAddress [1];
    }

    rampFile[6]='.';rampFile[7]='T';rampFile[8]='X';rampFile[9]='T';
    logFile[5]='.';logFile[6]='T';logFile[7]='X';logFile[8]='T';
  }
  myFile.close();
} // end function getTankID



// Function to retrieve the ramp length (# of lines) in the ramp file
void getRampLength() {
  char readChar;
  char tempVal [4];
   
  if (!myFile.open("RAMPLEN.TXT", O_READ)) sd.errorHalt("Failed to load ramp length");
  while (myFile.available()) { 
    readChar = myFile.read();
    for (int i = 0; i < 4; i++) {
      if (readChar != ';') {
        tempVal[i] = readChar;
        readChar = myFile.read();
      } else {
      tempVal[i] = '\0';
      break;
      }
    }
    rampLength = atoi(tempVal); // store ramp length as int
  }
  myFile.close(); 
} // end function getRampLength


// Function to retrieve the current line position in the ramp from the 'RAMPPOS.TXT' file
// and store it in the variable lineToLoad. When starting a new experiment, lineToLoad
// is set to 1.
void getRampPos() {
  char readChar;
  char tempVal [4];
  if (!myFile.open("RAMPPOS.TXT", O_READ)) sd.errorHalt("Failed to load ramp position");
  while (myFile.available()) { 
    readChar = myFile.read();
    for (int i = 0; i < 5; i++) {
        if (readChar != ';') {
        tempVal[i] = readChar;
        readChar = myFile.read();
      } else {
      tempVal[i] = ';';
      break;
      }
    }
    lineToLoad = atoi(tempVal); // store ramp position as int
  }
  myFile.close(); 
} // end function getRampPos



// Function to retrieve calibration coefficients for all sensors. Calls getSensorCoef() function.
void loadSensors() {
  getSensorCoef(TempCh);
  getSensorCoef(DOCh);
  getSensorCoef(pHCh);
} // end function loadSensors



// Function to retrieve calibration coefficients (the intercept and slope from the regression  
// of Temp/DO/pH against the digital output from the ADC). Called by loadSensors().
// Note that "tempVal" in this means temporary value, not anything to do with temperature!!
void getSensorCoef(byte sensor) {
  char readChar;
  char tempVal [15];
  float intercept, slope, resistor;
 
  switch (sensor) {
    case 0: // temperature sensor on channel 0
      if (!myFile.open("TCAL.TXT", O_READ)) sd.errorHalt("Failed to load temperature sensor coefficients");
      break;
    case 1: // pH sensor on channel 1
      if (!myFile.open("PHCAL.TXT", O_READ)) sd.errorHalt("Failed to load pH sensor coefficients");
      break;   
    case 2: // do sensor on channel 2
      if (!myFile.open("DOCAL.TXT", O_READ)) sd.errorHalt("Failed to load DO sensor coefficients");
      break;
  }

  while (myFile.available()) {
    readChar = myFile.read();
    for (byte i = 0; i < 13; i++) { // first parameter is the intercept
      if (readChar !=',') {
        tempVal[i] = readChar;
        readChar = myFile.read();
      } else {
        tempVal[i] = '\0';
        break;
      }
    }
    intercept = atof(tempVal);

    readChar = myFile.read();
    for (byte i = 0; i < 13; i++) {   // second parameter is the slope
      if (readChar !=';') {
        tempVal[i] = readChar;
        readChar = myFile.read();
      } else {
        tempVal[i] = '\0';
        break;
      }
    }
    slope = atof(tempVal);

//  // Read a third calibration coefficient/parameter    
//  if (sensor == 0) {  // thermistor has an extra parameter for the fixed resistor
//      readChar = myFile.read();
//      for (byte i = 0; i < 10; i++) {  
//        if (readChar !=';') {
//          tempVal[i] = readChar;
//          readChar = myFile.read();
//        } else {
//          tempVal[i] = '\0';
//          break;
//        } 
//      }
//      resistor = atof(tempVal);
//    }

  }
  myFile.close();

  switch (sensor) {
    case 0:
      tSens [0] = intercept; 
      tSens [1] = slope; 
//      tSens [2] = resistor; // third parameter
      break;
    case 1:
      phSens [0] = intercept;
      phSens [1] = slope;
      break;
    case 2:
      doSens [0] = intercept;
      doSens [1] = slope;
      break;

  }
} // end function getSensorCoef


// Function to initialise the DS3234 RTC and set clock if it is not running.
// Switches SPI mode to mode 3 for the RTC and then switches the mode back to 0
// for the other SPI devices.
void startRTC(){
  SPI.setDataMode(SPI_MODE3); 
  RTC.begin();
  if (! RTC.isrunning()) { // check if RTC is running
    Serial.println(F("RTC is NOT running!"));
  } else {
    now = RTC.now(); 
    Serial.print(F("RTC is running. Time is "));
    Serial.print(now.year(), DEC);Serial.print(F("-"));
    Serial.print(now.month(), DEC);Serial.print(F("-"));
    Serial.print(now.day(), DEC);Serial.print(F(" "));
    Serial.print(now.hour(), DEC);Serial.print(F(":"));
    Serial.print(now.minute(), DEC);Serial.print(F(":"));
    Serial.print(now.second(), DEC);Serial.println();
  }
  SPI.setDataMode(SPI_MODE0); 
} // end function startRTC


// Function to get the current time from the DS3234 RTC.
// Switches SPI mode to mode 3 for the RTC and then switches the mode back to 0
// for the other SPI devices
void getRTCTime() {
  SPI.setDataMode(SPI_MODE3); 
  now = RTC.now(); 
  SPI.setDataMode(SPI_MODE0);
} // end function getTime


// Function to read 20 consecutive values of all tank sensors via the ADC, average 
// the readings, and convert the averaged digital value into temp/DO/pH values using
// the sensor regression coefficients. Calls the readADC() function.
void readSensors() {

  // Read and average ADC values
  long pHSum = 0;
  long tempSum = 0;
  long DOSum = 0;
  int adcpH, adcTemp, adcDO;
  byte n = 0;
  SPI.setDataMode(SPI_MODE1);
  for (byte i = 0; i < 20; i++) {
    adcpH = readADC(0);
    delay(10);
    adcpH = readADC(0);
    adcTemp = readADC(1);
    delay(10);
    adcTemp = readADC(1);
    adcDO = readADC(2);
    delay(10);
    adcDO = readADC(2);

    pHSum = pHSum + adcpH;
    tempSum = tempSum + adcTemp;
    DOSum = DOSum + adcDO;
    n++;
  }
  int avgpH = pHSum/n;
  int avgTemp = tempSum/n;
  int avgDO = DOSum/n;
  
  // Convert averaged ADC readings to tank values using sensor coefficients

    // For sensors with linear calibration curves and 2 coefficients (intercept and slope)
    currentTemp = tSens[0] + avgTemp*tSens[1];
    currentDO = doSens[0] + avgDO*doSens[1];
    currentpH = phSens[0] + avgpH*phSens[1];
  
    // For a 10k thermistor (adafruit.com/product/372) in a voltage divider circuit with the thermistor
    // as the second resistor in the circuit (R2) and a fixed resistor is the first (R1)
    // Linearizing the simplified B-parameter equation 1/T = 1/T0 + (1/B)ln(R/R0), where T is in Kelvins
    // [1/T] = (1/B)[ln(R)] + [1/T0 - (1/B)ln(R0)] --> [1/T] = M[ln(R)] + C
    // where the intercept C = 1/T0 - (1/B)ln(R0), and the slope M = 1/B
    // Three calibration parameters: intercept, slope, and R1, the value of the first resistor in the
    // voltage divider circuit
//    float resTemp = (0.000125*tSens[2]*(float)avgTemp)/(5-(0.000125*(float)avgTemp));
//    currentTemp = (1/(tSens[0]+(tSens[1]*log(resTemp))))-273.15;
    
    
  SPI.setDataMode(SPI_MODE0);
}


// Reads the ADS 1118 ADC
int readADC(byte channel) {
  int rawVal = 0; // Raw value received back from the ADS1118
  byte MSB, LSB, MSBConf, LSBConf; //The most and least significant bits read from the ADS1118
  switch(channel) { // Most Significant Bit configuration register - this specifies which channel to read from
    case 0: MSBConf = 0b11000010;
    break;
    case 1: MSBConf = 0b11010010;
    break;
    case 2: MSBConf = 0b11100010;
    break;
  } 
  LSBConf=0b10101011; // Least Significant Bit configuration register
  noInterrupts();
  digitalWrite(adcCSPin, LOW);
  MSB = SPI.transfer(MSBConf);
  LSB = SPI.transfer(LSBConf);
  digitalWrite(adcCSPin, HIGH);
  interrupts();  

  // Build the raw value from the most and least significant bits
  rawVal = (MSB << 8) | LSB;
  return rawVal;
} // end function readADC



// Function that checks timekeeping variables and executes LogToSD(), serialReport(), and
// lcdReport() if their respective defined intervals have elapsed
void logAndReport() {

  // If time interval for SD logging has elapsed, log data to SD card
  if (millis() - sdPoint > sdInterval) {
    logToSD(); // Log tank data to SD card
    sdPoint = millis(); // Update sdPoint
  }
    
  // If time interval for reporting to serial monitor has elapsed, report date to serial monitor
  if (millis() - serialPoint > serialInterval) {
    serialReport(); // Report tank data to serial monitor
    serialPoint = millis(); // Update serialPoint
  }

  // If time interval for refreshing LCD has elapsed, refresh LCD
  #if LCD_I2C
    if (millis() - lcdPoint > lcdInterval) {
      lcdReport(); // Report tank data to LCD
      lcdPoint = millis(); // Update lcdPoint
    }
  #endif
}


// Function to log tank values and actuator statuses to the SD card.
void logToSD() {
  if (myFile.open(logFile, O_RDWR | O_CREAT | O_AT_END)) {
    digitalWrite(sdLEDPin, LOW); // turn off SD error LED
    myFile.print(now.year(),DEC);myFile.print("-");
    myFile.print(now.month(),DEC);myFile.print("-");
    myFile.print(now.day(),DEC);myFile.print(" ");
    myFile.print(now.hour(),DEC);myFile.print(":");
    myFile.print(now.minute(),DEC);myFile.print(":");
    myFile.print(now.second(),DEC);myFile.print(",");
    myFile.print(currentTemp);myFile.print(",");
    myFile.print(currentDO),myFile.print(",");
    myFile.print(currentpH),myFile.print(",");
    myFile.print(rampStat);myFile.print(",");
    myFile.print(chillerStat);myFile.print(",");
    myFile.print(heaterStat);myFile.print(",");
    myFile.print(N2Stat);myFile.print(",");
    myFile.println(CO2Stat);
    myFile.close();
  } else {
    Serial.println(F("Failed to open or create log file"));
    digitalWrite(sdLEDPin, HIGH); // turn on SD error LED
  }
} // end function logToSD


//Function to print sensor reading and ramp values to the serial monitor.
void serialReport(){
  Serial.print(now.year(), DEC);Serial.print(F("-"));
  Serial.print(now.month(), DEC);Serial.print(F("-"));
  Serial.print(now.day(), DEC);Serial.print(F(" "));
  Serial.print(now.hour(), DEC);Serial.print(F(":"));
  Serial.print(now.minute(), DEC);Serial.print(F(":"));
  Serial.print(now.second(), DEC);Serial.println();
  Serial.print(F("Temp: "));Serial.print(currentTemp);Serial.print(F(" degC. "));
  if(rampStat==1){
    Serial.print(F("Allowed range: "));Serial.print(tempMin);Serial.print(F(" - "));
    Serial.print(tempMax);Serial.print(F(", "));
    if (heaterStat==1) Serial.print(F("HEATER"));
    else if (chillerStat==1) Serial.print(F("CHILLER"));
    else Serial.print(F("NOTHING"));
    Serial.println(F(" is on."));
  }
  Serial.println(F(""));
  Serial.print(F("DO: "));Serial.print(currentDO);Serial.print(F(" mg/L. "));
  if(rampStat==1){
    Serial.print(F("Allowed range: "));Serial.print(DOMin);Serial.print(F(" - "));
    Serial.print(DOMax);Serial.print(F(", "));
    if (N2Stat==1) Serial.print(F("N2"));
    else Serial.print(F("NOTHING"));
    Serial.println(F(" is on."));
  }
  Serial.println(F("")); 
  Serial.print(F("pH: "));Serial.print(currentpH);Serial.print(F(". "));
  if(rampStat==1){
    Serial.print(F("Allowed range: "));Serial.print(pHMin);Serial.print(F(" - "));
    Serial.print(pHMax);Serial.print(F(", "));
    if (CO2Stat==1) Serial.print(F("CO2"));
    else Serial.print(F("NOTHING"));
    Serial.println(F(" is on."));
  } 
  Serial.println(F(""));Serial.println(F(""));  
} // end function serialReport


// Function to open comma-separated text file on SD card containing ramp info, 
// read, process and store time point (in minutes) and threshold values for temp/DO
// for the current line of the ramp.
void getRampLine(){
  int lineIndex = 0;
  int lineNumber = 0;
  char rampLine [40]; 
  char readChar;
  
  if (myFile.open(rampFile, O_READ)) {
    Serial.println(F("Opened ramp file"));
    while (myFile.available()) {
      readChar = myFile.read(); // Gets one character from serial buffer
      if (readChar != '\n'){ 
        rampLine[lineIndex] = readChar; // store incoming chars to buffer
        lineIndex++; // Increment buffer index to write next char
      } 
      else if (readChar == '\n'){ // if end of line is reached
        lineNumber++; // update the line number
        lineIndex++; 
        if (lineNumber == lineToLoad){  //if this is the correct line to load
          Serial.print(F("Line loaded: "));Serial.println(lineNumber);
          lineNumber = 0;
          lineIndex = 0;
          break; // break out of the "while" loop to process data
        } 
        else { //not the correct line to load, reset and continue
          lineIndex = 0; // reset buffer index so next line of ramp replaces current one in buffer
        }
      }
    }
    myFile.close(); //close file
    
    // process the data in the line buffer
    char* lineposition;
    lineposition = strtok(rampLine, ",");
    float tempvals [7]; 
    for (int i = 0; i < 7; i++){ 
        tempvals [i] = atof(lineposition); 
        lineposition = strtok(NULL,",");
    }
      
    // store each value for the current ramp line
    timeMinutes = tempvals [0];
    tempMax = tempvals [1];
    tempMin = tempvals [2];
    DOMax = tempvals [3];
    DOMin = tempvals [4]; 
    pHMax = tempvals [5];
    pHMin = tempvals [6];   
  } else {
    digitalWrite(sdLEDPin, HIGH);
    Serial.println(F("Failed to open ramp file")); 
  } 
} // end function getRampLine


// Function to check current tank conditions against ramp values and turn actuators
// off or on as needed.
void updateActuators() {

  // ------------- TEMPERATURE CONTROL -------------
  if (currentTemp > tempMax) { // Temp is above upper setpoint
    
    // CHILLER ON
    // chiller status on
    chillerStat = 1;
    
    // if using duty cycle, set duty cycle to 1 (or tweak if necessary)
//    chillerDutyCycle = 1; // comment out if using on/off

    // if using on/off, turn on chiller
    digitalWrite(chillerPin, HIGH); // comment out if using duty cycle

    // heater off
    heaterStat = 0;
    heaterDutyCycle = 0;
    digitalWrite(heaterPin, LOW);
    
  } else if (currentTemp + 2 < tempMin) { // Temp is more than 2 degC below lower setpoint
    
    // HEATER ON
    // heater status on
    heaterStat = 1;

    // if using duty cycle, set duty cycle to 1 for full duty cycle
    heaterDutyCycle = 1; // comment out if using on/off

    // if using on/off, turn on heater
//    digitalWrite(heaterPin, HIGH); // comment out if using duty cycle

    // chiller off
    chillerStat = 0;
    chillerDutyCycle = 0;
    digitalWrite(chillerPin, LOW);

  } else if ((currentTemp + 2 > tempMin) & (currentTemp + 1 < tempMin)) { // Temp is between 1-2 degC below lower setpoint

    // HEATER ON
    // heater status on
    heaterStat = 1;

    // if using duty cycle, set duty cycle to 75%
    heaterDutyCycle = 0.75; // comment out if using on/off

    // if using on/off, turn on heater
//    digitalWrite(heaterPin, HIGH); // comment out if using duty cycle

    // chiller off
    chillerStat = 0;
    chillerDutyCycle = 0;
    digitalWrite(chillerPin, LOW);

  } else if ((currentTemp + 1 > tempMin) & (currentTemp < tempMin)) { // Temp is <1 degC below lower setpoint

    // HEATER ON
    // heater status on
    heaterStat = 1;

    // if using duty cycle, set duty cycle to 50%
    heaterDutyCycle = 0.5; // comment out if using on/off

    // if using on/off, turn on heater
//    digitalWrite(heaterPin, HIGH); // comment out if using duty cycle

    // chiller off
    chillerStat = 0;
    chillerDutyCycle = 0;
    digitalWrite(chillerPin, LOW);
    
  } else { // Temp is within desired range

    // HEATER OFF
    heaterStat = 0;
    heaterDutyCycle = 0; 
    digitalWrite(heaterPin, LOW); 

    // CHILLER OFF
    chillerStat = 0;
    chillerDutyCycle = 0; 
    digitalWrite(chillerPin, LOW); 
  }

  // ------------- DO CONTROL -------------
   if (currentDO > DOMax + 1) {

    N2Stat = 1; // N2 status on
    N2bubbleRate = 0.6; // solenoid duty cycle at 40%

   } else if ((currentDO < DOMax + 1) & (currentDO > DOMax + 0.3)) {
    
    N2Stat = 1; // N2 status on
    N2bubbleRate = 0.4; // solenoid duty cycle at 20%

   } else if ((currentDO < DOMax + 0.3) & (currentDO > DOMax)) {
    
    N2Stat = 1; // N2 status on
    N2bubbleRate = 0.25; // solenoid duty cycle at 20%

  } else if (currentDO < DOMax) {
    
    N2Stat = 0; // N2 status off
    N2bubbleRate = 0; // solenoid duty cycle at 0%
  }

   // ------------- PH CONTROL -------------
   if (currentpH > pHMax + 0.03) { 

    CO2Stat = 1; // CO2 status on
    CO2bubbleRate = 0.03; // solenoid duty cycle at 3%

   } else if (currentpH < pHMax + 0.03) {

    CO2Stat = 0; // CO2 status off
    CO2bubbleRate = 0; // solenoid duty cycle at 0%
   
   }
    
} // end function updateActuators


// Function to control an actuator (solenoids, heater) on a duty cycle
// The dutyCycle argument will be supplied by ratios determined in the 
// updateActuators() function. cycleSeconds defines the cycle duration
// (in seconds) and allows for some manual tweaking of the system.
void pulseActuator(int actuatorPin, float dutyCycle, int cycleSeconds) {
  unsigned long time1 = millis();
  unsigned long time2 = millis();
  while (time2 < (time1 + cycleSeconds*1000*dutyCycle)) {
    digitalWrite(actuatorPin, HIGH);
    time2 = millis();
  }
  digitalWrite(actuatorPin, LOW);
} // end function pulseActuator


// Function to write the current lineToLoad value to the SD card
// This allows the value to be retrieved and the ramp to continue
// where it left off, if the Arduino loses power and resets
void updateRampPos() { 
  char buffer [4];
  // open RAMPPOS.TXT and clear current value before writing
  if (myFile.open("RAMPPOS.TXT", O_RDWR | O_TRUNC)) {
    digitalWrite(sdLEDPin, LOW); // turn off SD error LED
    myFile.print(lineToLoad);myFile.print(';');
    myFile.close();
  } else {
    Serial.println(F("Failed to open or create RAMPPOS file"));
    digitalWrite(sdLEDPin, HIGH); // turn on SD error LED
  }
} // end function updateRampPos
