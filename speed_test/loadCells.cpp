#include "loadCells.h" // header in local directory
#include <HX711.h>//https://github.com/bogde/HX711

//be careful of delays!!!!

namespace loadCells{

LOADCELLSINFO loadCellsInfo;

HX711 scaleA;
HX711 scaleB;
HX711 scaleC;
HX711 scaleD;

void LCsetup() {
  //Init Serial USB
  Serial.begin(9600);
  Serial.println(F("Initialize Load Cells Calibration - Remove weight from walker!!"));
  scaleA.begin(loadCellsInfo.doutPinA, loadCellsInfo.sckPinA);
  scaleB.begin(loadCellsInfo.doutPinB, loadCellsInfo.sckPinB);
  scaleC.begin(loadCellsInfo.doutPinC, loadCellsInfo.sckPinC);
  scaleD.begin(loadCellsInfo.doutPinD, loadCellsInfo.sckPinD);
  
  scaleA.tare(); //pink
  scaleB.tare(); //green
  scaleC.tare(); //yellow
  scaleD.tare(); //orange
  
  long b_A = -322; //scaleA.get_units(20); //no weight on scale to get intercept
  long b_B = 17; //scaleB.get_units(20);
  long b_C = -2; //scaleC.get_units(20);
  long b_D = 1; //scaleD.get_units(20);

  long m_A = 19098; //(y_A-b_A)/loadCellsInfo.inputWeight; 
  long m_B = 51846; //(y_B-b_B)/loadCellsInfo.inputWeight; 
  long m_C = 52342; //(y_C-b_C)/loadCellsInfo.inputWeight; 
  long m_D = 53342; //(y_D-b_D)/loadCellsInfo.inputWeight; 
  
  scaleA.set_scale(m_A);
  scaleA.set_offset(b_A);
  scaleB.set_scale(m_B);
  scaleB.set_offset(b_B);
  scaleC.set_scale(m_C);
  scaleC.set_offset(b_C);
  scaleD.set_scale(m_D);
  scaleD.set_offset(b_D);

  loadCellsInfo.weightA = scaleA.get_units(loadCellsInfo.numberOfReadings)+loadCellsInfo.offsetA;
  loadCellsInfo.weightB = scaleB.get_units(loadCellsInfo.numberOfReadings)+loadCellsInfo.offsetB;
  loadCellsInfo.weightC = scaleC.get_units(loadCellsInfo.numberOfReadings)+loadCellsInfo.offsetC;
  loadCellsInfo.weightD = scaleD.get_units(loadCellsInfo.numberOfReadings)+loadCellsInfo.offsetD;
  loadCellsInfo.walker_weight = loadCellsInfo.weightA + loadCellsInfo.weightB + loadCellsInfo.weightC + loadCellsInfo.weightD;
} 

float readScale(unsigned long duration) { /* function readScale */
  //timer check
  static unsigned long chrono = micros();
  if (micros() - chrono < duration) return 0;
  chrono = micros();
  
  //read scale
  bool A = scaleA.is_ready();
  bool B = scaleB.is_ready();
  bool C = scaleC.is_ready();
  bool D = scaleD.is_ready();
  
//  if (scaleB.is_ready()) {
  if (A && B && C && D) {
   loadCellsInfo.weightA = scaleA.get_units(loadCellsInfo.numberOfReadings)+loadCellsInfo.offsetA;
   loadCellsInfo.weightB = scaleB.get_units(loadCellsInfo.numberOfReadings)+loadCellsInfo.offsetB;
   loadCellsInfo.weightC = scaleC.get_units(loadCellsInfo.numberOfReadings)+loadCellsInfo.offsetC;
   loadCellsInfo.weightD = scaleD.get_units(loadCellsInfo.numberOfReadings)+loadCellsInfo.offsetD;

   loadCellsInfo.summed_load = loadCellsInfo.weightA + loadCellsInfo.weightB + loadCellsInfo.weightC + loadCellsInfo.weightD;
//   Serial.print("summed load value: ");
//   Serial.println(loadCellsInfo.summed_load);

   return loadCellsInfo.summed_load;

  } else {
//   Serial.println("HX711 not found.");
    return loadCellsInfo.summed_load;
//     loadCellsInfo.weightA = 10;
//     loadCellsInfo.weightB = 10;
//     loadCellsInfo.weightC = 10;
//     loadCellsInfo.weightD = 10;
//     loadCellsInfo.summed_load = loadCellsInfo.weightA + loadCellsInfo.weightB + loadCellsInfo.weightC + loadCellsInfo.weightD;
//     Serial.print("summed load value: ");
//     Serial.println(loadCellsInfo.summed_load);
  }
} //return something from readScale NO VOID

} //namespace loadCells
