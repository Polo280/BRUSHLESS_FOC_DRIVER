#include "BLDC_Logger.h"

void BLDC_Logger::init()
{
    SD_detected = true;
    if(!SD.begin(BUILTIN_SDCARD)){
        SD_detected = false;
        return;
    }

    if(SD_detected){
        if (SD.exists(this->logger_file_name)) {
            SD.remove(this->logger_file_name);
        }

        SD_log_file = SD.open(this->logger_file_name, FILE_WRITE);

        SD_log_file.println("time(ms),Raw Throtttle,VBat,CurrentA,CurrentB,CurrentC");
        SD_log_file.flush();
    }
}


void BLDC_Logger::logMotorDataSD()
{ 
    if(!SD_detected){
        return;
    }

    SD_log_file.print(data.timestamp);
    SD_log_file.print(',');
    SD_log_file.print(data.raw_throttle);
    SD_log_file.print(',');
    SD_log_file.print(data.VBat);
    SD_log_file.print(',');
    SD_log_file.print(data.currentA, 3);
    SD_log_file.print(',');
    SD_log_file.print(data.currentB, 3);
    SD_log_file.print(',');
    SD_log_file.println(data.currentC, 3);

    sampleID++;

    /* small fix: flush after N samples */
    if(sampleID >= kSamplesToFlushSD){
        SD_log_file.flush();
        sampleID = 0;
    }
}
