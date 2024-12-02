#include "sdMemCard.h"
#include <SdFat.h>
#include <arduino.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

//const int chipSelect = 4;
//bool ok;

#define SD_FAT_TYPE 3

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ( 50 )

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#  define SD_CONFIG SdioConfig( FIFO_SDIO )
#elif ENABLE_DEDICATED_SPI
#  define SD_CONFIG SdSpiConfig( SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK )
#else  // HAS_SDIO_CLASS
#  define SD_CONFIG SdSpiConfig( SD_CS_PIN, SHARED_SPI, SPI_CLOCK )
#endif  // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat sd;
File configFile;
File logFile
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 configFile;
File32 logFile;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile configFile;
ExFile logFile;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile logFile;
#else  // SD_FAT_TYPE
#  error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

    namespace sdMemCard {

  SDCARDINFO sdCardInfo;

  unsigned long duration_log;

  int sdMemCard::SDCARDINFO::fileNumber = 1;

  void getTimeStamp() {
    // read rtc (64bit, number of 32,768 kHz crystal periods)
    uint64_t periods;
    uint32_t hi1 = SNVS_HPRTCMR, lo1 = SNVS_HPRTCLR;
    while( true ) {
      uint32_t hi2 = SNVS_HPRTCMR, lo2 = SNVS_HPRTCLR;
      if( lo1 == lo2 && hi1 == hi2 ) {
        periods = (uint64_t)hi2 << 32 | lo2;
        break;
      }
      hi1 = hi2;
      lo1 = lo2;
    }

    // calculate seconds and milliseconds
    uint32_t ms = ( 1000 * ( periods % 32768 ) ) / 32768;
    time_t sec  = periods / 32768;

    tm t                   = *gmtime( &sec );  // calculate calendar data
    sdCardInfo.second      = t.tm_sec;
    sdCardInfo.minute      = t.tm_min;
    sdCardInfo.hour        = t.tm_hour;
    sdCardInfo.day         = t.tm_mday;
    sdCardInfo.month       = t.tm_mon + 1;
    sdCardInfo.year        = t.tm_year + 1900;
    sdCardInfo.millisecond = int( ms );

  }

  void dumpDebugLog() {
    if( sdCardInfo.sdFileIsOpen ) {
      logFile.close();
      sdCardInfo.sdFileIsOpen = false;
    }
    if( sdCardInfo.sdValid ) {
      if( sd.exists( sdCardInfo.logFileName ) ) {
        logFile                 = sd.open( sdCardInfo.logFileName, FILE_READ );
        sdCardInfo.sdFileIsOpen = true;
        Serial.print( "logFile dump start >>>\n" );
        while( logFile.available() ) {
          uint8_t ch = logFile.read();
          Serial.write( ch );
        }
        logFile.close();
        sdCardInfo.sdFileIsOpen = false;
        Serial.print( "\n<<< end logFile dump\n" );
      }
      else {
        sdCardInfo.sdFileIsOpen = false;
        Serial.print( "SD Card removed or full?" );
        sdCardInfo.sdValid = false;
      }
    }
    else {
      Serial.print( "sdCardInfo.sdValid is not valid\n" );
      sdCardInfo.sdFileIsOpen = false;
    }
  }


  void openLog() {
    
    
    if( sdCardInfo.sdValid ) {
//      if( sd.exists( sdCardInfo.logFileName ) ) {
        logFile = sd.open( sdCardInfo.logFileName, FILE_WRITE );
        sdCardInfo.logBuf[0] = '\0';
        sdCardInfo.sdFileIsOpen = true;
//      }
//      else {
//        sdCardInfo.sdFileIsOpen = false;
//        Serial.print( "SD Card removed or full?" );
//        sdCardInfo.sdValid = false;
//      }
    }
    else {
      Serial.print( "sdCardInfo.sdValid is not valid\n" );
      sdCardInfo.sdFileIsOpen = false;
    }
  }

  void closeLog() {
    if( sdCardInfo.sdFileIsOpen ) {
      logFile.close();
      
      sdCardInfo.fileNumber++;

      sdCardInfo.logFileName = "logFile" + String(sdCardInfo.fileNumber) + ".txt";
    }
    
    sdCardInfo.sdFileIsOpen = false;
    
//    //freeing the buffers - do i need to free the pointers?
//    free(sdCardInfo.logBuf);
  }

  void writeLog() {
    if( sdCardInfo.sdValid ) {
        while (sd.card()->isBusy()) {
        }
        size_t bytesw = logFile.write(sdCardInfo.logBuf, strlen(sdCardInfo.logBuf));
        sd.errorPrint(&Serial);
        Serial.print("number of logfile bytes ");
        Serial.println(bytesw);
        Serial.print("is sd busy? ");
        Serial.println(sd.isBusy());
        Serial.print("getWriteError ");
        Serial.println(logFile.getWriteError());
        while (sd.card()->isBusy()) {
        }
        closeLog();
      }
      else {
        Serial.printf( "%s Error - SD File is not open\n", sdCardInfo.logBuf );
      }
    }
  


  // -----------------------------------------------------------------------------------------------
  // ------------------------------ FILE HELPERS ---------------------------------------------------
  // -----------------------------------------------------------------------------------------------

  bool isBufferFull(char* bufferMalloc, unsigned int bufferSizeMalloc) {
    return strlen(bufferMalloc) >= bufferSizeMalloc;
  }

  void sdMemCardBegin() {
    // sdCardInfo.debugLogOnSerial4 = true;
    sdCardInfo.sdExists     = true;
    sdCardInfo.sdValid      = true;
    sdCardInfo.sdFileIsOpen = false;
    Serial.print( "Opening SD card..." );
//    ok = SD.sdfs.begin(SdioConfig(FIFO_SDIO));
    if( !sd.begin (SD_CONFIG) ) {
      Serial.print( "opening SD failed!\n" );
      sdCardInfo.sdExists = false;
      sdCardInfo.sdValid  = false;
      return;
    }
    else {
      Serial.print( "completed\n" );
      // Serial.printf( "Checking logFile.txt...\n" );
      int filecount = 1;
      while (true) {
        String lastFileName = ("logFile" + String(filecount) + ".txt"); 
        if( !sd.exists( lastFileName ) ) {
          sdCardInfo.logFileName = lastFileName;
          break;
        }
        filecount++;
      }
      Serial.print("filecount starts at: ");
      Serial.println(sdCardInfo.logFileName);
      
      if( sd.exists( sdCardInfo.logFileName ) ) {     
        writeLog();
      }
      else {
        Serial.printf( sdCardInfo.logBuf, "File "+ sdCardInfo.logFileName +" does not exist. Attempting to create (new)" );
        logFile = sd.open( sdCardInfo.logFileName, FILE_WRITE | O_APPEND );
        logFile.print( "year,month,day,hour,minute,second,velocity,right_trajectory,left_trajectory,r_out,l_out,loadcell_1,loadcell_2,loadcell_3,loadcell_4,sync_pin,start_pin,perturb_num,condition" );
        logFile.close();
        delay( 500 );
        if( sd.exists( sdCardInfo.logFileName ) ) {
          sprintf( sdCardInfo.logBuf + strlen(sdCardInfo.logBuf), "[SD Card] Created empty file " );
          writeLog();
        }
        else {
          Serial.print( "_________FILE CREATE FAILED, CHECK WRITE PROTECT?________\n" );
          sdCardInfo.sdValid = false;
        }
      }
    }
  }

  void chooseData(unsigned long duration, double trajectory_in,  double r_traj_in, double l_traj_in, double r_out_in, double l_out_in, float weightA_in, float weightB_in, float weightC_in, float weightD_in, bool sync_in, bool start_in, int trial_in, int traj_in) {
    //timer check
    static unsigned long chrono = micros();
    if (micros() - chrono < duration) return;
    chrono = micros();
//    Serial.print("chooseData duration lasts this number of microseconds: ");
//    Serial.println(chrono);///1000000.0);

    getTimeStamp();
    
    //save data from other scripts to the sdMemCard namespace
    sdCardInfo.trajectoryData = trajectory_in;
    sdCardInfo.r_traj = r_traj_in;
    sdCardInfo.l_traj = l_traj_in;
    sdCardInfo.r_out = r_out_in;
    sdCardInfo.l_out = l_out_in;
    sdCardInfo.weightAData = weightA_in;
    sdCardInfo.weightBData = weightB_in;
    sdCardInfo.weightCData = weightC_in;
    sdCardInfo.weightDData = weightD_in;
    sdCardInfo.syncData = sync_in;
    sdCardInfo.startData = start_in;
    sdCardInfo.trial_num = trial_in;
    sdCardInfo.traj_running = traj_in;

    sprintf( sdCardInfo.logBuf + strlen(sdCardInfo.logBuf), "\n%04d,%02d,%02d,%02d,%02d,%02d.%03d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%i,%i", sdCardInfo.year, sdCardInfo.month, sdCardInfo.day, sdCardInfo.hour,
             sdCardInfo.minute, sdCardInfo.second, sdCardInfo.millisecond,
    sdCardInfo.trajectoryData, sdCardInfo.r_traj, sdCardInfo.l_traj, sdCardInfo.r_out, sdCardInfo.l_out,
    sdCardInfo.weightAData, sdCardInfo.weightBData, sdCardInfo.weightCData, sdCardInfo.weightDData, 
    sdCardInfo.syncData, sdCardInfo.startData, sdCardInfo.trial_num, sdCardInfo.traj_running);
  }
  
}  // namespace sdMemCard
