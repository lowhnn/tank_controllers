README - Tank Controller SD Card Files


	User-created files - 

	TANKID.TXT
	1-2 digit integer followed by a semicolon. Assigns the controller an ID number
	which is used to determine its I2C address and the filenames for the ramp and
	log files.


	TEMPCAL.TXT, DOCAL.TXT, PHCAL.TXT
	Contain sensor calibration coefficients. 2 comma-separated float values followed
	by a semicolon, representing an intercept and slope value for the calibration of
	temperature/DO/pH value against the digital ADC value (0-65535). See notes in the
	TankControllerCode file for information on how to use sensor calibrations with
	more than 2 coefficients/parameters.


	RAMPxx.TXT 
	Where xx is a 1-2 digit integer matching the one in TANKID.TXT. The file contains
	lines of 7 comma-separated values with no delimiters at the end of each row. The
	values represent the timepoints and setpoint ranges for the ramp:

	[time in min],[maxTemp],[minTemp],[maxDO],[minDO],[maxpH],[minpH]


	Sample ramp with 3 timepoints:

		60,20.0,14.0,6.5,0.0,8.0,7.3
		150,20.0,15.0,5.0,0.0,7.8,7.3
		240,20.0,16.5,4.5,0.0,7.7,7.3

	The program will read the first line and use the actuators to maintain temperature
	in the range of 14-20ºC, the dissolved oxygen in the range of 0-6.5 mg/L, and the
	pH in the range of 7.3-8.0, and maintain it there until the stipulated 60 minute 		timepoint is reached. Keep in mind the physical capabilities of the actuators when
	designing the ramp so that the setpoints are actually achievable in the
	length of time designated by the timepoint.


	RAMPLEN.TXT
	1-3 digit integer followed by a semicolon. Specifies the length (number of lines)
	in the RAMPxx.TXT file.

	
	RAMPPOS.TXT
	1-3 digit integer followed by a semicolon. Specifies which line of the ramp the 
	program is currently on. For a new experiment the user should set this to 1. The
	program overwrites the value as it progresses through the ramp. This allows the
	ramp to start from where it left off in the case of an Arduino reset.

 
	 


	Program-created files -

	LOGxx.TXT
	Where xx is a 1-2 digit integer matching the one in TANKID.TXT. This file is created
	and modified by the program. It logs the values of temperature (degC), dissolved 
	oxygen (mg/L), and pH, as well as the on/off status of the ramp and actuator (0/1), 
	at regular intervals specified in the program code (variable sdInterval, default is
	5 minutes).

	Format:

	[date/time],[temp],[do],[pH],[ramp],[chiller],[heater],[N2],[CO2]


	

	

	
	
	

	