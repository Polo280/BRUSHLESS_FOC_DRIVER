#include <Arduino.h>
#include "Bldc.h"
#include "Trap.h"
#include "Telemetry_Manager.h"

#ifdef FOC_CONTROL
  Foc bldc;
#else
  Trap bldc;
#endif

TelemetryManager telem_manager;

Bldc::foc_all_fast_t foc_fast;
Bldc::foc_status_t   foc_status;

uint32_t time_aux = 0;

void setup()
{
    bldc.driverInit();

    //////////////////////////
    telem_manager.begin(Serial1);

    /* bind pointers */
    telem_manager.allFast = &bldc.foc_fast_data;

}

void loop()
{
    bldc.run();
    telem_manager.process();

    if(millis() - time_aux >= 100){
      logger->logMotorDataSD();
      time_aux = millis();
    }
}