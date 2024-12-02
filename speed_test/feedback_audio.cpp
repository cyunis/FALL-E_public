#include "feedback_audio.h"
#include <elapsedMillis.h>
#include "loadCells.h"

// Use these with the Teensy Audio Shield
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14

namespace feedback_audio{
  
FEEDBACKINFO fbinfo;

void setupAudio() {
  // Audio shield initialization
  AudioMemory(8);
  fbinfo.sgtl5000_1.enable();
  fbinfo.sgtl5000_1.volume(0.5);
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!SD.begin(SDCARD_CS_PIN)) {
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
}


void audio_instructions(unsigned long durationAudio, float weight, float target) {
   //timer check
   static unsigned long chrono = micros();
   if (micros() - chrono < durationAudio) return;
   chrono = micros();
  
  static bool playing = false;
  static elapsedMillis timer;

  fbinfo.weight_threshold = loadCells::loadCellsInfo.walker_weight + target; //kg

  if (fbinfo.playSdWav1.isPlaying()) {
    playing = true;
  } else if (!fbinfo.playSdWav1.isPlaying() && !playing) {
    if (weight > fbinfo.weight_threshold + fbinfo.threshold_error) {
      fbinfo.playSdWav1.play("high_beep_halfsec.wav");
      fbinfo.volume_error = volumeScaling(weight);
      fbinfo.sgtl5000_1.volume(fbinfo.volume_error);
      timer = 0;
      playing = true;
    }
    if (weight < fbinfo.weight_threshold - fbinfo.threshold_error) {
      fbinfo.playSdWav1.play("low_beep_halfsec.wav");
      fbinfo.volume_error = volumeScaling(weight);
      fbinfo.sgtl5000_1.volume(fbinfo.volume_error);
      timer = 0;
      playing = true;
    }
  }

  if (playing && (timer >= fbinfo.volume_error || timer > durationAudio/1000)) {
    fbinfo.playSdWav1.stop();
    playing = false;
  }
}


float volumeScaling(float weight) {
  float error = abs(weight - fbinfo.weight_threshold);
  error = map(error, 0, 20, 5, 8); //can only do ints with map
  error = error/10; //to get float
  return error;
}

}
