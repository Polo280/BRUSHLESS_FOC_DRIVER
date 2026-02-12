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

/* ---- fake data storage ---- */
Bldc::foc_all_fast_t fake_fast;
Bldc::foc_status_t   fake_status;

uint16_t fake_vbus;
int32_t  fake_ibus;
int32_t  fake_rpm;
int16_t  fake_iq;
int16_t  fake_id;

void setup()
{
    bldc.driverInit();

    //////////////////////////
    telem_manager.begin(Serial1);

    /* bind pointers */
    telem_manager.allFast = &fake_fast;
    telem_manager.status  = &fake_status;

    telem_manager.vbus_mV = &fake_vbus;
    telem_manager.ibus_mA = &fake_ibus;
    telem_manager.rpm     = &fake_rpm;
    telem_manager.iq_mA   = &fake_iq;
    telem_manager.id_mA   = &fake_id;

    /* initial fake values */
    fake_fast.vbus_mV = 48000;
    fake_fast.ibus_mA = 1200;
    fake_fast.rpm     = 543;
    fake_fast.fault_flags = 0;

    fake_status.state = 1;
    fake_status.fault_flags = 0;

    fake_vbus = 48000;
    fake_ibus = 1200;
    fake_rpm  = 543;
    fake_iq   = 150;
    fake_id   = -20;
}

void loop()
{
    bldc.run();
    telem_manager.process();

    /* animate values so you see changes */
    static uint32_t t = 0;
    if (millis() - t > 200)
    {
        t = millis();
        fake_rpm += 10;
        fake_fast.rpm = fake_rpm;
    }
}
