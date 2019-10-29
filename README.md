# Tank controller public repository

This repository accompanies the manuscript:

Low NHN, Ng CA, Micheli F. A low-cost control system for multi-stressor climate change experiments.


**Folders and files in repository:**

The *TankController* folder contains EAGLE schematic files for the tank controller shield and a parts list in the *TankControllerSchematics* subfolder and the relevant Arduino program code in the *TankControllerCode* subfolder. The *SensorCalibrationCode* subfolder contains a simple Arduino program that can be used with the tank controller units to calibrate sensors, as described in Appendix 2 of the manuscript. The *SDCardFiles* subfolder contains the list of files that should be on the microSD card used with the tank controller, and includes a readme with descriptions and instructions for all the files.

The *MasterArduino* folder contains general schematic diagrams for an implementation of the tank controllers in an experimental aquaria system, including an ethernet-connected 'Master' Arduino board that consolidates data from multiple tanks and sends them to online datastreams via the [ThingSpeak](http://thingspeak.com) platform. The *UABC_MasterCode* subfolder contains the Arduino program code for running such a Master board.
