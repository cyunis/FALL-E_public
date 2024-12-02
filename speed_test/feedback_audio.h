#ifndef FEEDBACK_AUDIO_H
#define FEEDBACK_AUDIO_H

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>


namespace feedback_audio {
  struct FEEDBACKINFO {
    //audio setup
    AudioPlaySdWav           playSdWav1;
    AudioOutputI2S           i2s1;
    AudioConnection          patchCord1; //(playSdWav1, 0, i2s1, 0); //need 2 for stereo sound
    AudioConnection          patchCord2; //(AudioPlaySdWav, 1, i2s1, 1);
    AudioControlSGTL5000     sgtl5000_1;
    
    //Set desired weight threshold and error bounds
    float weight_threshold;
    float threshold_error = 2.5; //kg
    float volume_error;

    unsigned long previousTime;  // Variable to store the previous time
    unsigned long interval_delay;  // Time interval in milliseconds

    //constructor (for the FEEDBACKINFO "class" which is a struct but structs are basically classes)
    // : notation with class - everything after the colon is initialized
    FEEDBACKINFO() : patchCord1(playSdWav1, 0, i2s1, 0), patchCord2(playSdWav1, 1, i2s1, 1) {}
  };

  extern FEEDBACKINFO fbinfo;
  
  void setupAudio();
  void audio_instructions(unsigned long durationAudio, float weight, float target);
  float volumeScaling(float weight);

} //namespace feedback_audio

#endif
