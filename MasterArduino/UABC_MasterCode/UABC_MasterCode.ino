/*

MASTER ARDUINO FOR UABC SYSTEM 

updated 2019-07-16

This is code for the 'Master' Arduino used in the UABC implementation of the control system 
documented in the manuscript:

Low NHN, Ng CA, Micheli F. A low-cost control system for multi-stressor climate change experiments.

Photos, diagrams, schematics, component descriptions and part numbers, and a general description
of the controller's functions are all available in the manuscript and stored on the Github
repository (github.com/lowhnn/tank_controllers). 

//////////////////////////////////////////////////////////////////////////////////////

This implementation of the control system includes four tank controllers (code and schematics are
available on the Github repository), connected on an I2C bus to an Arduino Mega 2560 fitted with a
W5500 Ethernet Shield. The Arduino Mega is also connected to a 20x4 LCD screen on the same I2C bus,
and to a gas pressure sensor (on the N2 supply cylinder) using one of the analog pins. It does not
utilize a custom shield.

NOTES ON POWER: 
Although the Arduino Mega 2560 board is rated for a 7-13VDC power supply, we have found that 
powering the board and Ethernet shield with 12V may cause the voltage regulator on the board
to overheat, due to the high current draw from the Ethernet shield. 7-9VDC has worked well.


//////////////////////////////////////////////////////////////////////////////////////
ABOUT THE CODE:

The Master Arduino is programmed to retrieve tank conditions from multiple tank controller
Arduino Uno boards, consolidate the data, and log/stream/display the data.

This is a quick overview of what the code does:

(1) On startup, serial, I2C, and SPI communications are initialized. LCD is initialized and
    the ethernet connection is established. Through the network connection, current date/time
    are obtained from the NTP server and connection to the Thingspeak platform (thingspeak.com)
    is established. Calibration coefficients for the pressure sensor connected to the N2 cylinder
    are retrieved from the SENSOR.TXT file on the SD card.

(2) The getTankVals() and getTankActs() functions send requests for data to the tank controllers,
    over the I2C connections. These values are displayed on the LCD screen. 

(3) The pressureRead() function reads the input from the gas pressure sensor and converts the value
    to a current pressure reading using the sensor coeffients.

(4) Variables for timekeeping of datalogging, streaming, and visual feedback functions (sdPoint,
    serialPoint, lcdPoint, streamPoint) are initialised.

(5) The program enters a loop that runs indefinitely:
    - The getTankVals() and getTankActs() functions send requests for data to the tank controllers,
      over the I2C connections.
    - The pressureRead() function reads the input from the gas pressure sensor and converts the value
      to a current pressure reading using the sensor coeffients.
    - If the specified time intervals for logging, streaming, and updating the LCD screen (sdInterval,
      lcdInterval, serialInterval, streamInterval) have elapsed, those actions are executed via the
      DataToSD(), LCDUpdate(), DataToSerial(), and StreamTank() functions. 

//////////////////////////////////////////////////////////////////////////////////////

The code is open-source and free to be adapted and modified as desired. Here are some guidelines for
making things work/customising basic options:

(1) Update the mac[] variable with the actual MAC address of your Ethernet shield

(2) Update the ip[], netmask[], gateway[], dns[] variables with your network settings.

(3) Update the channelNumber[] and tankAPI[] variables with the values from your Thingspeak channels

(4) If desired, logging and streaming time-intervals can be modified by changing the variables
    sdInterval and streamInterval. They are currently set to log and stream every 5 min (300000 ms)

(5) If desired, the code can be modified for a different number of networked tank controllers.
    Make the following changes:
    - Change length of the arrays TempVals, DOVals, pHVals, RampStat, SolN2Stat, HeaterStat,
      ChillerStat, SolCO2Stat to match the number of desired tanks. Current length is 4.
    - Change the numTanks variable to match the number of desired tanks. Current value is 4.
     


*/



////////////////////////////////////////////////////////////////////////////////////////
// LIBRARIES
#include <Wire.h>
#include <LiquidCrystal.h>
#include <SdFat.h>
#include <SPI.h> 
#include <Ethernet.h>
#include <ezTime.h>
#include <ThingSpeak.h>

////////////////////////////////////////////////////////////////////////////////////////
// DEFINE PINS
#define SDcardPin 4 
#define EthernetPin 10
#define cyPressurePin A0 


////////////////////////////////////////////////////////////////////////////////////////
// DEFINE GLOBAL VARIABLES

// Define number of tanks --------------------------------------------------------------
byte numTanks = 4;


// Variables to store data from tank controllers ---------------------------------------
float TempVals[4], DOVals[4], pHVals[4];
boolean RampStat[4], SolN2Stat[4], HeaterStat[4], ChillerStat[4], SolCO2Stat[4];


// Variables for reading/logging N2 cylinder pressure ----------------------------------
float cyPressure;
float cyCoefs[2];


// For LCD screen ----------------------------------------------------------------------
LiquidCrystal lcd(0);


// For logging to SD card --------------------------------------------------------------
SdFat sd;
SdFile myFile;
char logFile[] = "MLOG.TXT";
#define error(s) sd.errorHalt_P(PSTR(s))


// For Ethernet connection --------------------------------------------------------------
EthernetClient client;
byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x0D, 0xAE };
byte ip[] = { 148,231,228,56 };
byte netmask[] = { 255,255,255,192 };
byte gateway[] = { 148,231,228,1 };
byte dns[] = { 148,231,192,6 };


// For NTP timekeeping -----------------------------------------------------------------
Timezone myTZ;

// For connection to ThingSpeak --------------------------------------------------------
  // channel numbers
  unsigned long channelNumber [4] = {796055, 796057, 796059, 796060}; 

  // API keys
  const char * tankAPI [4] = {"S82QBXXS5NXPUHPG", "ETNDNQU9NTZZ1HHW","6L3DMHPZF8BUPCHR","4K7RECP4J1A796UI"}; 


// For communication with tank controller Arduinos -------------------------------------
  // I2C addresses for tank controllers
  byte TANK_ID [4] = {1, 2, 3, 4}; 

  // List of possible commands to give tank controller Arduinos
  #define VALUE_TEMP 1
  #define VALUE_DO 2
  #define VALUE_PH 3
  #define STATUS_RAMP 4
  #define STATUS_SOL_N2 5
  #define STATUS_SOL_CO2 6
  #define STATUS_HEATER 7
  #define STATUS_CHILLER 8

  // Define union with of one variable represented as float and byte array
  // this is for the conversion of received bytes to float sensor values
  typedef union float2bytes_t {    
    float f; 
    char b[sizeof(float)]; 
  }; 
  

// For contolling timing loops ------------------------------------------------------------
  // Timing for each function is controlled by a logpoint variable (unsigned long) and an 
  // interval variable (int or long, depending on how long the interval is)

  // For refreshing the LCD screen
  unsigned long lcdPoint;
  int lcdInterval = 5000; // 5-second interval

  // For printing to the serial monitor
  unsigned long serialPoint;
   int serialInterval = 10000; // 10-second interval

  // For logging to the SD card
  unsigned long sdPoint;
  long sdInterval = 300000; // 5-minute interval
//  long sdInterval = 60000; // 1-minute interval (for testing)


  //For streaming data to ThingSpeak platform
  unsigned long streamPoint;
  long streamInterval = 300000; // 5-minute interval
//  long streamInterval = 60000; // 1-minute interval (for testing)





////////////////////////////////////////////////////////////////////////////////////////
  
void setup () {
   
  // Initialise I2C and Serial communications
  Serial.begin (9600);
  Serial.println(F("Serial initialized"));
  
  Wire.begin();   
  Serial.println(F("I2C initialized"));

  // Begin standard SPI 
  SPI.begin();
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE0); 
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  Serial.println(F("SPI initialized"));

  // Initialize LCD
  lcd.begin(20, 4); // initialize LCD
  Serial.println(F("LCD initialized"));
  lcd.setBacklight(HIGH);
  lcd.print(F("Starting Master..."));

  // Initialise Ethernet
  lcd.setCursor(0, 1);
  lcd.print(F("Ethernet..."));
  startEthernet();
  lcd.setCursor(12,1);
  lcd.print(F("done."));

  // Initialize NTP time
  lcd.setCursor(0, 2);
  lcd.print(F("NTP sync..."));
  startNTP();
  lcd.setCursor(13,2);
  lcd.print(F("done."));
  delay(500);

  // Initialise SD card or print error
  lcd.setCursor(0, 3);
  lcd.print(F("SD card..."));
  if (!sd.begin(SDcardPin, SPI_HALF_SPEED)) sd.initErrorHalt(F("SD initialization failed"));
  Serial.println(F("SD card initialized."));Serial.println();
  lcd.setCursor(12,3);
  lcd.print(F("done."));
  delay(500);

  // Initialise ThingSpeak connection
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Starting Master..."));
  lcd.setCursor(0, 1);
  lcd.print(F("ThingSpeak..."));
  ThingSpeak.begin(client);
  Serial.println(F("Connected to ThingSpeak."));
  lcd.setCursor(13, 1);
  lcd.print("done.");
  delay(500);

  // Load sensor coefficients for N2 pressure sensor
  lcd.setCursor(0, 2);
  lcd.print(F("Coefficients..."));
  loadCoeffs();  
  lcd.setCursor(15, 2);
  lcd.print("done.");
  delay(1000);

 

  // First sensor readings and LCD update
    // Get data from all tanks
    getTankData(numTanks);

    // Report to LCD
    LCDUpdate();

  // First check of cylinder pressure
  pressureRead();
  Serial.print(F("N2 Cylinder Pressure = "));Serial.print(cyPressure);Serial.println(F(" psi"));
  Serial.println();

  // Initialise timepoints for timing loops
  sdPoint = millis();
  lcdPoint = millis();
  serialPoint = millis();
  streamPoint = millis();
  Serial.println(F("TimePoints initialized"));




}  

//////////////////////////////////////////////

void loop(){

  // Get data from all tanks
  getTankData(numTanks);
  
  // Read cylinder pressure
  pressureRead();
  
  // If next logging timepoint has been reached, log and report data
  if (millis() - sdPoint > sdInterval) {

    // Log all tank data to SD card
    DataToSD();

    // update sdPoint
    sdPoint = millis();
  }

    // If next lcd refresh timepoint has been reached, refresh LCD with latest values
    if (millis() - lcdPoint > lcdInterval) {
      
      // Refresh LCD screen
      LCDUpdate();
      
      // update lcdPoint
      lcdPoint = millis(); 
  }


  // If next serial-report timepoint has been reached, log and report data
  if (millis() - serialPoint > serialInterval) {
    
    // Send all tank data to serial port
    DataToSerial();

    // update serialPoint
    serialPoint = millis();
  }


  // If next ThingSpeak data-streaming timepoint has been reached, stream data
  if (millis() - streamPoint > streamInterval) {
    
    for (byte i = 0; i < numTanks; i++) StreamTank(i);

    // update streamPoint
    streamPoint = millis();
    
  }


}
  
//////////////////////////////////////////////
// FUNCTIONS USED

// Function to initialize Ethernet connection
// Attempts to configure via DHCP first
void startEthernet(){
  Serial.println(F("Setting up Ethernet..."));
  Ethernet.init(10);
  
  // start the Ethernet connection:

//  Ethernet.begin(mac, ip, netmask, gateway, dns);
//  Serial.print(F("My IP address: "));
//  Serial.println(Ethernet.localIP());

  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  } else {
    Serial.print(F("My IP address: "));
    Serial.println(Ethernet.localIP());
  }

  // give the Ethernet shield a second to initialize:
  delay(1000);
}


// Function to initialise NTP and set timezone
void startNTP() {

  // synchronize time using ezTime
  Serial.println(F("Syncing time..."));
  waitForSync();
  
  myTZ.setLocation("America/Tijuana");
  Serial.print(F("America/Tijuana: "));
  Serial.println(myTZ.dateTime("Y-m-d H:i:s"));
 
}

// Function to read coefficients for the N2 pressure sensor from the sensor calibration
// text file on the SD card
void loadCoeffs() {
  char readChar;
  char tempVal [15];

  Serial.println(F("Loading N2 pressure sensor coefficients..."));

  if (!myFile.open("SENSOR.TXT", O_READ)) sd.errorHalt("Failed to load sensor coefficients");
  
  while (myFile.available()) {
    readChar = myFile.read();
    for (byte i = 0; i < 15; i++) { // first value is the regression intercept
      if (readChar !=',') {
        tempVal[i] = readChar;
        readChar = myFile.read();
      } else {
        tempVal[i] = '\0';
        break;
      }
    }
    cyCoefs[0] = atof(tempVal);

     readChar = myFile.read();
     for (byte i = 0; i < 15; i++) {   // second param is the regression slope
      if ((readChar !=';') & (readChar !=',')) {
        tempVal[i] = readChar;
        readChar = myFile.read();
      } else {
        tempVal[i] = '\0';
        break;
      }
    }
    cyCoefs[1] = atof(tempVal);
  }
  myFile.close();

  Serial.print(F("Intercept = ")); Serial.print(cyCoefs[0]);
  Serial.print(F(", Slope = ")); Serial.println(cyCoefs[1]);
   
}




// Function to initialize LCD screen
void startLCD() {
  lcd.begin(20, 4); // initialise LCD
  Serial.println(F("LCD initialized"));
  lcd.setBacklight(HIGH);
  delay(1000);
}


void pressureRead() {
  unsigned int pressureSum = 0;
  for (byte i = 0; i < 30; i++) {
    pressureSum = pressureSum + analogRead(A0);
  }
  int avgPressure = pressureSum/30;
  cyPressure = cyCoefs[0] + cyCoefs[1]*avgPressure; 
}


// Function to request all sensor and actuator data from all tanks
// Calls the getTanksVals() and getTankActs() functions

void getTankData(byte numTanks) {
  
    for (byte i = 0; i < numTanks; i++) getTankVals(i);
    Serial.println(F("Tank values received."));

    // Get actuator statuses from each tank 
    for (byte i = 0; i < numTanks; i++) getTankActs(i);
    Serial.println(F("Tank statuses received."));
}



// Function to request all sensor data from a given tank and store them in
// the data arrays TempVals, DOVals, and pHVals.
// It checks the status of tanks and sensor types to only request data from active 
// tanks, and only for the variables of interest.
  // Note that tankNo starts from 0, not 1, because it is indexing values in arrays,
  // and indexing in C++ starts from 0. 
void getTankVals(const byte tankNo){
  
  TempVals[tankNo] = getSensorVal(TANK_ID[tankNo], VALUE_TEMP);
  
  DOVals[tankNo] = getSensorVal(TANK_ID[tankNo], VALUE_DO);
  
  pHVals[tankNo] = getSensorVal(TANK_ID[tankNo], VALUE_PH);
  
} 


// Function to request sensor data from a tank, receive the 4-byte array,
// then convert and return the corresponding float value. 
// This function is called from the getTankVals() function
float getSensorVal(const byte tankID, const byte whichVal){
  byte bytesSensor[4];
  float2bytes_t f2bSensor;
  float floatSensor;
  byte i=0, j;
  sendCommand (tankID,whichVal, 4);
  while(Wire.available()) bytesSensor[i++] = Wire.read ();
  for (j = 1; j <= 4; j++) f2bSensor.b[j] = bytesSensor[j];
  floatSensor = f2bSensor.f;
  return(floatSensor);
}

// Function to send command to slave to request a given type of data
// This function is called from the getTankVals() function, through  getSensorVal()
void sendCommand (const byte tankID, const byte cmd, const int responseSize){
  //command and expected response size (bytes) - figure out how to transfer floats?
  Wire.beginTransmission (tankID);
  Wire.write (cmd);
  Wire.endTransmission ();
  Wire.requestFrom (tankID, 4);
 }


// Function to request actuator status from a tank.
boolean getActStat(const byte tankID, const byte whichVal){
 sendCommand(tankID, whichVal, 1);
 boolean statActuator = Wire.read(); 
 return(statActuator);
}


// Function to obtain the status of all actuators in a tank
// Checks statuses only for tanks that have been listed as active
void getTankActs(const byte tankNo){
  RampStat[tankNo] = getActStat(TANK_ID[tankNo], STATUS_RAMP);
  SolN2Stat[tankNo] = getActStat(TANK_ID[tankNo], STATUS_SOL_N2);
  HeaterStat[tankNo] = getActStat(TANK_ID[tankNo], STATUS_HEATER);
  ChillerStat[tankNo] = getActStat(TANK_ID[tankNo], STATUS_CHILLER);
  SolCO2Stat[tankNo] = getActStat(TANK_ID[tankNo], STATUS_SOL_CO2);
}


// Function to display T/DO/pH values on the LCD screen

void LCDUpdate(){
  lcd.clear();
  
  for (byte i = 0; i < 4; i++){
    lcd.setCursor(0, i); 
    lcd.print(i+1); lcd.print(F(" "));
    lcd.print(TempVals[i],1); lcd.print(F("C "));
    lcd.print(DOVals[i],1); lcd.print(F("mgL "));
    lcd.print(pHVals[i],2);
  }
}



// Function to write the timestamped conditions of each tank to the SD card
void DataToSD() {
  if (!myFile.open(logFile, O_RDWR | O_CREAT | O_APPEND)) {
  sd.errorHalt(F("opening log file for write failed"));
  }
  
  // Log time
  myFile.print(myTZ.dateTime("Y-m-d H:i:s"));
  
  // Log values for each active tank
  for (byte i = 0; i <4; i++){
    myFile.print(F(","));myFile.print(TempVals[i]);
    myFile.print(F(","));myFile.print(DOVals[i]);
    myFile.print(F(","));myFile.print(pHVals[i]);
  }
  
  // Log cylinder pressure
  myFile.print(F(","));myFile.println(cyPressure);
  myFile.close();

  // Report to Serial
  Serial.println(F("Logged values to SD card."));
  Serial.println();
}





// Function to report time and values of all tanks to serial monitor

void DataToSerial() {

  Serial.println(myTZ.dateTime("Y-m-d H:i:s"));

  Serial.print(F("Cylinder is at "));Serial.print(cyPressure);Serial.println(F(" psi."));
  
  for (int i = 0; i < 4; i++){
    Serial.print(F("Tank "));Serial.print(i+1);Serial.print(F(": ")); 
    Serial.print(TempVals[i]);Serial.print(F(" degC"));
    if (HeaterStat[i]==1) Serial.print(F("+")); // this is if heater is on
    else if (ChillerStat[i]==1) Serial.print(F("-")); // this is if chiller is on
    Serial.print(F(" "));Serial.print(DOVals[i],1);
    if (SolN2Stat[i]==1) Serial.print(F("mg/L-"));
    else Serial.print(F("mg/L"));
    Serial.print(" ");Serial.print(pHVals[i],2);
    if (SolCO2Stat[i]==1) Serial.println(F("pH-"));
    else Serial.println(F("pH"));
    
  }
  Serial.println(F(""));
 }



// Function to update one channel with values and statuses of one tank

void StreamTank(byte tankNumber) {

  // populate fields with values to stream
  if ((TempVals[tankNumber] < 1000) & (TempVals[tankNumber] > 0)) {
    ThingSpeak.setField(1, round_to_dp(TempVals[tankNumber],2));
  }

  if ((DOVals[tankNumber] < 1000) & (DOVals[tankNumber] > 0)) {
    ThingSpeak.setField(2, round_to_dp(DOVals[tankNumber],2));
  }

  if ((pHVals[tankNumber] < 1000) & (pHVals[tankNumber] > 0)) {
    ThingSpeak.setField(3, round_to_dp(pHVals[tankNumber],2));
  }
  ThingSpeak.setField(4, HeaterStat[tankNumber]);
  ThingSpeak.setField(5, ChillerStat[tankNumber]);
  ThingSpeak.setField(6, SolN2Stat[tankNumber]);
  ThingSpeak.setField(7, SolCO2Stat[tankNumber]);
  ThingSpeak.setField(8, cyPressure);

  Serial.print(F("Streaming channel data for Tank "));
  Serial.print(tankNumber+1); Serial.print(F("..."));
  
  // stream values to ThingSpeak channel
  int x = ThingSpeak.writeFields(channelNumber[tankNumber], tankAPI[tankNumber]); 

  if(x == 200){
    Serial.println(F("update successful."));
  }
  else
  {
    Serial.print(F("Problem updating channel. ")); Serial.println("HTTP error code " + String(x));
  }

  Serial.println();

}


 float round_to_dp(float in_value, int decimal_place) {
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier)/multiplier;
  return in_value;
}
