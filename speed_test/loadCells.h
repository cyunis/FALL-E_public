#ifndef LOADCELLS_H //prevents multiple inclusions in cpp file
#define LOADCELLS_H

namespace loadCells{

  struct LOADCELLSINFO {
    
    const int numberOfReadings = 1;
    
    //pink load cell
    const int doutPinA = 33; //green wire
    const int sckPinA = 34; //blue wire
    const float offsetA = -1.27; //kg
    //green load cell
    const int doutPinB = 28; //green wire
    const int sckPinB = 29; //blue wire
    const float offsetB = -1; //kg
    //yellow load cell
    const int doutPinC = 26; //green wire
    const int sckPinC = 27; //blue wire
    const float offsetC = -1.92; //kg
    //orange load cell
    const int doutPinD = 36; //green wire
    const int sckPinD = 37; //blue wire
    const float offsetD = -0.84; //kg
    
    //Variables
    float weightA = 0;
    float initWeightA = 0;
    float weightB = 0;
    float initWeightB = 0;
    float weightC = 0;
    float initWeightC = 0;
    float weightD = 0;
    float initWeightD = 0;

    float summed_load;
    float walker_weight;

  };

  extern LOADCELLSINFO loadCellsInfo;
  
  void LCsetup();
  
  float readScale(unsigned long duration);

}// namespace loadCells

#endif //LOADCELLS_H
