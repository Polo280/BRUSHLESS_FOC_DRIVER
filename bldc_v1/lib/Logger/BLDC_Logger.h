#pragma once

#include <Arduino.h>
#include <SD.h>

// Flush to SD every N samples 
constexpr uint8_t kSamplesToFlushSD{200};

typedef struct {
    uint32_t timestamp;  // Milliseconds
    uint16_t raw_throttle;
    uint16_t VBat;       // mV
    float currentA;      // A
    float currentB;      // A
    float currentC;      // A
} BLDC_Logger_Data;


class BLDC_Logger {

private:
    const char* logger_file_name = "testing1.csv";
    File SD_log_file;
    bool SD_detected = false;
    uint8_t sampleID = 0;

public:
    BLDC_Logger_Data data;

    BLDC_Logger() = default;
    ~BLDC_Logger();

    void init();

    void logMotorDataSD();
};
