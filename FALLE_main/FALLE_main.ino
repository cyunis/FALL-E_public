#include "feedback_audio.h"
#include "loadCells.h"
#include "sdMemCard.h"
#include "buttonBox.h"
#include <util/atomic.h>
#include <iostream>
#include <cmath>
#include <ODriveUART.h>
#include <ODriveEnums.h>
#include <SoftwareSerial.h>


//UPDATE THESE - experiment variables
float SSWS = 1.07; //in m/s, converted in setup()
double perturb_size [6]; //make array in setup()
bool feedback_on = 1; //1 is on
int feedback_condition = 2; //0 is 5%, 1 is 10%, 2 is 20%
float body_weight = 69.2; //kg
//double pilot_conditions [40] = {0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3}; //for practice
double pilot_conditions [55] = {1,2,2,3,3,0,2,3,3,3,3,3,2,0,0,3,2,3,1,2,3,0,3,1,3,3,2,0,2,2,0,1,2,0,2,3,0,0,3,0,0,2,1,1,2,0,2,1,1,0,0,2,0,1,1};
float randomStepTime; 
float step_times [5] = {300, 430, 560, 390, 420};

// perturbation trajectory, setpoint, direction and count variables
float SSWS_start; //start buffer //REMOVE?
float SSWS_end; //end time 
float perturb_duration = 500; //in ms, can be 0.5 or 0.25 
unsigned int perturb_num = 0;
bool perturbation_played = 0;
int dir [2];
int velocity_direction;
float revs_speed = 0;
double R_traj;
double L_traj;

// loop timing variables
unsigned long counter;
unsigned long load_time;
unsigned long t_from_last_command;
long prevT;

// weight + button + switch variables
float target_weights [3]; //kg
const int debounceDelay = 100; // Adjust debounce delay as needed
float switch_value;
int switch_pin = 41;

// logging variables
int log_condition = 10;
double log_velocity = 0;
int log_frequency = 10000;
unsigned int buffer_size = 250000; //check malloc open space and sdMemCard.h

// odrive globals + setup
int baudrate = 115200;
HardwareSerial& odrive_serialR = Serial4;
ODriveUART odriveR(odrive_serialR);
HardwareSerial& odrive_serialL = Serial1;
ODriveUART odriveL(odrive_serialL);

// state machine variables
enum SwitchState {
  SWITCH_OFF = 0,
  SWITCH_TURN_ON = 1,
  SWITCH_ON = 2,
  SWITCH_TURN_OFF = 3,
};
SwitchState switch_current_state = SWITCH_OFF;
SwitchState switch_next_state = SWITCH_OFF;
enum PerturbState {
  WAITING = 0,
  SYNCING = 1,
  COUNTDOWN_BEGIN = 2,
  TURN_ON = 3,
  PERTURBATION = 4,
  TURN_OFF = 5,
  COUNTDOWN_END = 6
};
PerturbState current_state = WAITING;
PerturbState next_state = WAITING;
PerturbState experiment_phase = TURN_OFF; //repurpose to check for feedback state or not?

// more variables for load cells, buttons and data logging are in the h files


void setup() {
  //Initialize the serial ports
  odrive_serialR.begin(baudrate);
  odrive_serialL.begin(baudrate);
  Serial.begin(baudrate); // must match baudrate

  //set up the switch
  pinMode(switch_pin, INPUT_PULLUP);

  //Initialize tasks
  SSWS = speedConversion(SSWS);
  Serial.print("SSWS in rev/s: ");
  Serial.println(SSWS);
  SSWS_start = 2000; 
  SSWS_end = 4500; 

  sdMemCard::sdMemCardBegin();
  sdMemCard::openLog(); //should be replaced with filename GUI
  feedback_audio::setupAudio();
  loadCells::LCsetup();
  Serial.print("Initial weight: ");
  Serial.println(loadCells::loadCellsInfo.walker_weight);

  //set up odrives
  Serial.println("Waiting for odriveR...");
  while (odriveR.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }
  Serial.println("found odriveR");
  Serial.println("Waiting for odriveL...");
  while (odriveL.getState() == AXIS_STATE_UNDEFINED) {
    delay(100);
  }
  Serial.println("found odriveL");
  Serial.println("Enabling closed loop control...");
  while (odriveR.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odriveR.clearErrors();
    odriveR.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  Serial.println("odriveR running!");
  while (odriveL.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odriveL.clearErrors();
    odriveL.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }
  Serial.println("odriveL running!");

  //calculate speeds
  perturb_size[0] = SSWS*0.5; //forward
  perturb_size[1] = 0; //no change
  perturb_size[2] = -SSWS*0.5; //backward
  perturb_size[3] = -SSWS*1; //backward

  //calculate target weights
  target_weights[0] = body_weight * 0.05;
  target_weights[1] = body_weight * 0.1;
  target_weights[2] = body_weight * 0.2;

  Serial.println("Done with setup");

  //save time for completing setup()
  prevT = micros();
  load_time = millis();
}


void loop() {
  //use counter to tell time in ms since loop started
  counter = millis() - load_time;
  unsigned long t = (counter - t_from_last_command);
  velocity_direction = dir[0] - dir[1];
  switch_value = digitalRead(switch_pin);
  sdMemCard::chooseData(log_frequency, log_velocity, R_traj, L_traj, odriveR.getVelocity(), odriveL.getVelocity(), 
                          loadCells::loadCellsInfo.weightA, loadCells::loadCellsInfo.weightB, loadCells::loadCellsInfo.weightC, loadCells::loadCellsInfo.weightD,
                          buttonBox::buttonInfo.syncBool, buttonBox::buttonInfo.startBool, perturb_num, log_condition);

  ///////////////// SWITCH STATE MACHINE CODE ///////////////////
  // SWITCH_TURN_OFF STATE - check if the switch was flipped off first
  if (switch_current_state == SWITCH_TURN_OFF) { 
    Serial.println("SWITCH_TURN_OFF - 3");
    switch_next_state = SWITCH_OFF;
  }
  // SWITCH_TURN_ON STATE
  if (switch_current_state == SWITCH_TURN_ON) {
    Serial.println("SWITCH_TURN_ON - 1");
    switch_next_state = SWITCH_ON;
  } 
  // SWITCH_ON STATE
  if (switch_current_state == SWITCH_ON) {
    if (switch_value > 0) { //maybe replace >0 with debouncing logic
      Serial.println("SWITCH_ON - 2");
      switch_next_state = SWITCH_TURN_OFF;
    }
  }
  // SWITCH_OFF STATE
  if (switch_current_state == SWITCH_OFF) {
    if (switch_value == 0) {
      Serial.println("SWITCH_OFF - 0");
      switch_next_state = SWITCH_TURN_ON;
    }
  }

  ///////////// PERTURBATION STATE MACHINE CODE /////////////////
  // TURN_OFF STATE 
  if (current_state == TURN_OFF) {
    Serial.println("TURN_OFF - 5");
    next_state = COUNTDOWN_END;
  }
  
  // TURN_ON STATE
  if (current_state == TURN_ON) {
    Serial.println("TURN ON - 3");
    next_state = PERTURBATION;
    t_from_last_command = counter;
  } 

  // WAITING STATE
  else if (current_state == WAITING) {
    log_frequency = 100000;

    //check if sync button is pressed
    buttonBox::readPin(5000); //same duration as data logging, sync pin hardcoded in cpp
    if (buttonPressed(buttonBox::buttonInfo.syncBool)) {
      Serial.println("WAITING STATE - 0");
      next_state = SYNCING;
      t_from_last_command = counter;
    }
  }

  // SYNCING STATE
  if (current_state == SYNCING) {
    log_frequency = 1000; 
    Serial.println(buttonBox::buttonInfo.syncBool);

    //read load cells 
    float weight = loadCells::readScale(1000);
    Serial.print("weight: ");
    Serial.println(weight);
    
    //check if white button is pressed
    buttonBox::readPin(5000);
    if (buttonPressed(buttonBox::buttonInfo.startBool)) {
      sdMemCard::writeLog();
      sdMemCard::sdCardInfo.logBuf[0] = '\0';  // Clear the buffer without using free()
      sdMemCard::openLog();
      Serial.println("SYNCING STATE - 1");
      experiment_phase = TURN_OFF;
      next_state = COUNTDOWN_BEGIN;
      t_from_last_command = counter;
    }  

    if (sdMemCard::isBufferFull(sdMemCard::sdCardInfo.logBuf, buffer_size)) {
      sdMemCard::writeLog();
      sdMemCard::sdCardInfo.logBuf[0] = '\0';  // Clear the buffer without using free()
      sdMemCard::openLog();
    }
  }

  // COUNTDOWN_BEGIN STATE
  if (current_state == COUNTDOWN_BEGIN) {
    log_frequency = 10000;
    log_condition = 10;
    log_velocity = 0; 
    
    if (sdMemCard::isBufferFull(sdMemCard::sdCardInfo.logBuf, buffer_size)) {
      sdMemCard::writeLog();
      sdMemCard::sdCardInfo.logBuf[0] = '\0';  // Clear the buffer without using free()
      sdMemCard::openLog(); //takes ~0.06-0.07 sec
    }

    //delay before transitioning states for audio to play
    if (experiment_phase == TURN_ON) {
      if (t > 4000) { 
        sdMemCard::writeLog();
        sdMemCard::sdCardInfo.logBuf[0] = '\0';  // Clear the buffer without using free()
        sdMemCard::openLog();
        perturb_num += 1;
        if (pilot_conditions[perturb_num] == 1) {
          log_condition = 1;
        }
        Serial.print("perturb_num is ");
        Serial.println(perturb_num);
        int rand_idx = sizeof(step_times) / sizeof(step_times[0]); //REMOVE?
        rand_idx = random(0, rand_idx); //REMOVE?
        randomStepTime = step_times[rand_idx]; //REMOVE?
        Serial.print("random step time: "); //REMOVE?
        Serial.println(randomStepTime); //REMOVE?
        Serial.println("COUNTDOWN_BEGIN - 2");
        next_state = experiment_phase;
        t_from_last_command = counter;
      }
    }

    //read buttons
    buttonBox::readPin(5000);

    //read load cells 
    float weight = loadCells::readScale(1000); //duration of 1000 usec between readings is 1000 hz frequency

    //check for feedback
    if (feedback_on) { 
      feedback_audio::audio_instructions(1000000 - durationScaling(weight), loadCells::loadCellsInfo.summed_load, target_weights[feedback_condition]);
    }

    //check which practice or experiment is running from sync press pattern
    if (buttonHeld(buttonBox::buttonInfo.syncBool, 3)) {
      next_state = WAITING;
      Serial.println("COUNTDOWN_BEGIN - 2");
      t_from_last_command = counter;
    }
    else if (buttonPressed(buttonBox::buttonInfo.syncBool)) {
      experiment_phase = TURN_ON;
      t_from_last_command = counter;
    }
  }

  // PERTURBATION STATE
  if (current_state == PERTURBATION) {
    log_frequency = 10000;
    int condition = pilot_conditions[perturb_num];
    log_condition = condition;
    R_traj = perturb_size[condition]+SSWS;
    L_traj = -(perturb_size[condition]+SSWS);
    log_velocity = velocity_direction * R_traj;
    
//      velocity = calculate_trajectory(t, condition, randomStepTime); //when is t calculated?
//      Serial.print(" t is: ");
//      Serial.println(t);
//      Serial.print(" velocity is: ");
//      Serial.println(odriveR.getVelocity()*(M_PI*6.5*0.0254)
    
    odriveR.setVelocity(R_traj);
    odriveL.setVelocity(L_traj);

    //read load cells 
    float weight = loadCells::readScale(12500); //duration of 10000 usec between readings is 100 hz frequency
    float variable_gain = 2*weight+70; //40 for vel_gain, 20 kg max expected applied weight
    odriveR.setParameter("axis0.controller.config.vel_gain",String(variable_gain));
    odriveR.setParameter("axis0.controller.config.vel_integrator_gain",String(variable_gain/3));
    odriveL.setParameter("axis0.controller.config.vel_gain",String(variable_gain));
    odriveL.setParameter("axis0.controller.config.vel_integrator_gain",String(variable_gain/3));
    

    //read buttons
    buttonBox::readPin(5000);

    //check for feedback
    if (feedback_on) {
      feedback_audio::audio_instructions(1000000 - durationScaling(weight), loadCells::loadCellsInfo.summed_load, target_weights[feedback_condition]);
    }

    //clear the log first thing before more data is logged
    if (sdMemCard::isBufferFull(sdMemCard::sdCardInfo.logBuf, buffer_size)) {
      sdMemCard::writeLog();
      sdMemCard::sdCardInfo.logBuf[0] = '\0';  // Clear the buffer without using free()
      sdMemCard::openLog(); //takes ~0.06-0.07 sec
      Serial.println("write to log");
    }

    if (t>perturb_duration || condition == 1) {
      sdMemCard::writeLog();
      sdMemCard::sdCardInfo.logBuf[0] = '\0';  // Clear the buffer without using free()
      sdMemCard::openLog();
      experiment_phase = TURN_OFF; //reassign so start phase waits
      Serial.println("PERTURBATION - 4");
      next_state = TURN_OFF;
      t_from_last_command = counter;
    }
  }

  // COUNTDOWN_END STATE
  if (current_state == COUNTDOWN_END) {
    //read load cells 
    float weight = loadCells::readScale(1000); //duration of 1000 usec between readings is 1000 hz frequency
    
    //check for feedback
    if (feedback_on) {
      feedback_audio::audio_instructions(1000000 - durationScaling(weight), loadCells::loadCellsInfo.summed_load, target_weights[feedback_condition]);
    }

    if (t > SSWS_end || buttonPressed(buttonBox::buttonInfo.startBool)) {
      sdMemCard::writeLog();
      sdMemCard::sdCardInfo.logBuf[0] = '\0';  // Clear the buffer without using free()
      sdMemCard::openLog();
      Serial.println("COUNTDOWN_END - 6");
      next_state = WAITING;
    }
  }

  //////////// WARNING!!!!!!!!!!! ///////////////
  //always TURN OFF MOTOR if switch state commands it//
  if (switch_current_state == SWITCH_OFF) {
    Serial.println("Set motor to AXIS_STATE_IDLE");
    odriveR.clearErrors();
    odriveR.setState(ODriveAxisState::AXIS_STATE_IDLE); // can be print statement to avoid breaking motor in falle main testing (can call print vs set state with bool as test mode / real mode at top)
    odriveL.clearErrors();
    odriveL.setState(ODriveAxisState::AXIS_STATE_IDLE);
    R_traj = 0;
    L_traj = 0;
  }
  else if (current_state == TURN_ON && switch_current_state == SWITCH_ON) {
    Serial.println("Set motor to AXIS_STATE_CLOSED_LOOP_CONTROL");
    odriveR.clearErrors();
    odriveR.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odriveL.clearErrors();
    odriveL.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  else if (current_state == TURN_OFF) {
    Serial.println("Set motor to AXIS_STATE_IDLE");
    odriveR.clearErrors();
    odriveR.setState(ODriveAxisState::AXIS_STATE_IDLE);
    odriveL.clearErrors();
    odriveL.setState(ODriveAxisState::AXIS_STATE_IDLE);
    R_traj = 0;
    L_traj = 0;
  }
  //make condition to check if teensy disconnected - switch to idle mode
  
  // UPDATE STATE 
  if (switch_current_state != switch_next_state) {
    Serial.print("moving to switch state: ");
    Serial.println(switch_next_state);
  }
  if (current_state != next_state) {
    Serial.print("odriveR State : ");
    Serial.println(odriveR.getState());
    Serial.print("moving to perturb state: ");
    Serial.println(next_state);
  }
  switch_current_state = switch_next_state;
  current_state = next_state; //change states
}


//////////////////////// FUNCTIONS ////////////////////////
float speedConversion(float ms) {
  //convert speed from m/s to rev/s (odrive default units)
  revs_speed = ms/(M_PI*6.5*0.0254); //m/s * (1 rev/2*pi*r meters) - where r*2 is 6.5 in * 0.0254 m/in
  return revs_speed;
}


bool buttonPressed(bool input_button) {
  static bool previousState = 1;
  static unsigned long debounceTime = 0;
  bool currentState = input_button;
  if (currentState != previousState) {
    debounceTime = millis();
  }
  if ((millis() - debounceTime) > debounceDelay) {
    if (currentState == 0) {
      previousState = currentState;
      return true;
    }
  }
  previousState = currentState;
  return false;
}


bool buttonHeld(bool input_button, float hold_duration) {
  static bool button_held = false;
  static unsigned long start_time = 0;
  if (input_button == 0) {
    if (!button_held) {
      start_time = millis();  // Start time of button hold
      button_held = true;
    }
    if (millis() - start_time >= hold_duration * 1000) {
      return true;  // Button held for the specified duration
    }
  } else {
    button_held = false;
  }
  return false;  // Button not held for the specified duration
}


int durationScaling(float weight) {
  int error = abs(weight - feedback_audio::fbinfo.weight_threshold);
  error = map(error, 0, 10, 0, 500000); //can only do ints with map
  return error;
}
