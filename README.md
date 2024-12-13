# FALL-E_public
A public repository of the code to run the robotic rollator FALL-E.

This code runs on a Teensy 4.1 in an Arduino environment. 
FALLE_main.ino is the basis for the code to control the rollator during experiments when someone is using FALL-E. This code was designed to collect data locally on the Teensy SD card that will be integrated with data collected using a Qualisys Motion Capture system. 
load_cells_calibration.ino is run to calibrate the load cells. The legs of FALL-E are removed individually for calibration.
speed_test.ino is used for data collection during experiments when FALL-E is running on its own, without a person.

For questions about the code contact cyunis@usc.edu
