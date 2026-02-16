#pragma once
#include <Arduino.h>
#include "Bldc.h"

/* ---------- protocol constants ---------- */

#define FOC_UART_SOF   0xAA

/* ---------- command IDs ---------- */

typedef enum
{
    FOC_CMD_GET_STATUS          = 0x01,
    FOC_CMD_GET_BUS_VOLTAGE     = 0x02,
    FOC_CMD_GET_BUS_CURRENT     = 0x03,
    FOC_CMD_GET_SPEED           = 0x04,
    FOC_CMD_GET_POWER           = 0x05,
    FOC_CMD_GET_ALL_FAST        = 0x06,
    FOC_CMD_GET_IQ              = 0x07,
    FOC_CMD_GET_ID              = 0x08,
    FOC_CMD_GET_PROTOCOL_VER    = 0x09,
    FOC_CMD_SET_IQ_PI_GAINS     = 0x20,
    FOC_CMD_SET_ID_PI_GAINS     = 0x21
} foc_uart_cmd_t;


class TelemetryManager
{
public:
    void begin(HardwareSerial &port);

    /* call this from loop() */
    void process();

    /* ----- data providers (you bind your real variables) ----- */

    Bldc::foc_all_fast_t* allFast;
    Bldc::foc_status_t*   status;

    uint16_t* vbus_mV;
    int32_t*  ibus_mA;
    int32_t*  rpm;
    int16_t*  iq_mA;
    int16_t*  id_mA;

    /* setters (optional – only used by SET commands) */
    bool (*setIqPiGains)(int32_t kp, int32_t ki);
    bool (*setIdPiGains)(int32_t kp, int32_t ki);

private:
    HardwareSerial *serial;

    enum RxState
    {
        WAIT_SOF,
        WAIT_LEN,
        WAIT_BODY,
        WAIT_CRC
    };

    RxState state = WAIT_SOF;

    uint8_t len;
    uint8_t index;
    uint8_t buf[64];
    uint8_t crc_rx;

    void handleFrame(uint8_t *frame, uint8_t len);
    void sendReply(uint8_t cmd, const uint8_t *payload, uint8_t payload_len);

    static uint8_t crc8_atm(const uint8_t *data, uint8_t len);
};