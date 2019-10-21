/*

CALIBRATION CODE

This is a simple program that can be used to calibrate sensors with the the tank controllers described
in the manuscript:

Low NHN, Ng CA, Micheli F. A low-cost control system for multi-stressor climate change experiments.

Photos, diagrams, schematics, component descriptions and part numbers, and a general description of the 
controller's functions are all available in the manuscript and stored on the Github repository 
(github.com/lowhnn/FiolabAquaria). 

//////////////////////////////////////////////////////////////////////////////////////
ABOUT THE CODE:

The program reads the inputs from the sensors on each of the 3 channels on the 16-bit ADC every second
and logs those values (between 0 - 65535) to the SD card. They are also printed to the serial monitor.
When the switch is flipped, the program adds an indicator (CAL) to the end of the logged values. The 
switch can be flipped when the sensor(s) are in a known value/standard and the readings have stabilized,
and those values extracted to obtain calibration curves for each sensor. The calibration coefficients
can then be input into the relevant text files (TCAL.TXT, DOCAL.TXT, PHCAL.TXT).


*/


// Sketch to calibrate sensors from the ADC

// Load libraries
#include <Wire.h>
#include <SdFat.h>
#include <SPI.h> 


// Define pins 
#define adcCSPin 8
#define SDcardPin 9
 #define RTCPin 10
#define buttonPin A0  


// Define ADC Channels
#define TempCh 0
#define DOCh 2
#define pHCh 1

// Instantiate SdFat and RTC
SdFat sd;
SdFile myFile;


// Variables
boolean buttonState1;
boolean buttonState2;
boolean calPoint = 0;
char logFile[] = "CALFILE.TXT";

  // Variables to store sensor readings
  int currentTemp, currentDO, currentpH;

//////////////////////////////////////////////////////

void setup() {
// Set up pins
pinMode(buttonPin,INPUT);  
pinMode (adcCSPin, OUTPUT);
digitalWrite(adcCSPin, HIGH); 
pinMode (RTCPin, OUTPUT);
digitalWrite(RTCPin, HIGH);
    
//Serial communications for testing and debugging code
Serial.begin(9600);

// Begin standard SPI for SD card and ADC
SPI.begin();
SPI.setBitOrder(MSBFIRST); 
SPI.setDataMode(SPI_MODE0); 
SPI.setClockDivider(SPI_CLOCK_DIV16); 
    
// Initialise SD card or print error
Serial.println("Initialise SD...");
if (!sd.begin(SDcardPin, SPI_HALF_SPEED)) sd.initErrorHalt();


}

//////////////////////////////////////////////////////

void loop() {


// Read sensor values from ADC
readSensors(); 


// Check button
buttonState1 = digitalRead(buttonPin);
if (buttonState1 == 1) {
    calPoint = 1;
} else {
  calPoint = 0;
}



// Display on serial monitor
Serial.print("TCh:");
Serial.print(currentTemp);
Serial.print(", DOCh: "); 
Serial.print(currentDO);
Serial.print(", pHCh: "); 
Serial.print(currentpH);
if (calPoint == 1) Serial.println(" CAL");
else Serial.println();  


// Log sensor value to SD card
LogToSD();


delay(1000);
}

//////////////////////////////////////////////////////

// FUNCTIONS USED:

// Function to read 20 consecutive values of all tank sensors via the ADC, average 
// the readings, and convert the averaged digital value into temp/DO/pH values using
// the sensor regression coefficients. Calls the readADC() function
void readSensors() {
  long pHSum = 0;
  long tempSum = 0;
  long DOSum = 0;
  int adcpH, adcTemp, adcDO;
  byte n = 0;
  SPI.setDataMode(SPI_MODE1);
  for (byte i = 0; i < 100; i++) {
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
  currentpH = pHSum/n;
  currentTemp = tempSum/n;
  currentDO = DOSum/n;
    
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
//  delay(10);   //delay of 7 or less leads to erratic readings from each channel

  // Build the raw value from the most and least significant bits
  rawVal = (MSB << 8) | LSB;
  return rawVal;
} // end function readADC




// Function to log digital reading from sensor/ADC to the SD card,
// and record calibration point
void LogToSD(){
  if (!myFile.open(logFile, O_RDWR | O_CREAT | O_APPEND)) sd.errorHalt("Failed to open log file"); 
  myFile.print(currentTemp);
  myFile.print(",");
  myFile.print(currentDO);
  myFile.print(",");
  myFile.print(currentpH);
  if (calPoint == 1){
    myFile.print(" ");myFile.println("CALPT");
  } else {
    myFile.println();
  }
  myFile.close();
  calPoint = 0;
}
