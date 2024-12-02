#ifndef __SDCARD_H
#define __SDCARD_H

#include <Arduino.h>
#include <stdlib.h>

namespace sdMemCard {

  struct SDCARDINFO {
    bool sdExists;  // true when SD card is inserted
    bool sdValid;   // move to SD
    bool sdFileIsOpen;
    char *logBuf = (char*)malloc(500000);
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int millisecond;
    double trajectoryData;
    double r_traj;
    double l_traj;
    double r_out;
    double l_out;
    float weightAData;
    float weightBData;
    float weightCData;
    float weightDData;
    bool syncData;
    bool startData;
    int trial_num;
    int traj_running;
    String logFileName;
    static int fileNumber;
  };

  extern SDCARDINFO sdCardInfo;

  void writeLog();
  void sdMemCardBegin();
  void dumpDebugLog();
  void chooseData(unsigned long duration, double trajectoryData, double r_traj, double l_traj, double r_out, double l_out, float weightAData, float weightBData, float weightCData, float weightDData, bool syncData, bool startData, int trial_num, int traj_running);
  void openLog();
  void closeLog();
  bool isBufferFull(char* bufferMalloc, unsigned int bufferSizeMalloc);
  

}  // namespace sdMemCard

#endif
