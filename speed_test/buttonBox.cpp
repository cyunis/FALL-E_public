#include "buttonBox.h"
#include <arduino.h>

namespace buttonBox {

BUTTONINFO buttonInfo;

void readPin(unsigned long duration) {
  //timer check
  static unsigned long chrono = micros();
  if (micros() - chrono < duration) return;
  chrono = micros();
//  Serial.print("measuring sync value at this number of seconds: ");
//  Serial.println(chrono/1000000.0);

  //check the sync
  pinMode(5, INPUT_PULLUP);  // enable internal pull-up, set to pin being read by teensy
  pinMode(35, INPUT_PULLUP); 
  buttonInfo.syncBool = 0;
  buttonInfo.startBool = 0;
  int syncPinValue = digitalRead(5);
  int startPinValue = digitalRead(35);
  //delay(10); // quick and dirty debounce filter
  if (buttonInfo.syncBool != syncPinValue) {
    buttonInfo.syncBool = syncPinValue;
  }
  if (buttonInfo.startBool != startPinValue) {
    buttonInfo.startBool = startPinValue;
  }
}

}
