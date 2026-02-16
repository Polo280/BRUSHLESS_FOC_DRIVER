#include "Telemetry_Manager.h"

void TelemetryManager::begin(HardwareSerial &port)
{
    serial = &port;

    allFast = nullptr;
    status  = nullptr;

    vbus_mV = nullptr;
    ibus_mA = nullptr;
    rpm     = nullptr;
    iq_mA   = nullptr;
    id_mA   = nullptr;

    setIqPiGains = nullptr;
    setIdPiGains = nullptr;
}


uint8_t TelemetryManager::crc8_atm(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}


void TelemetryManager::process()
{
    while (serial->available())
    {
        uint8_t b = serial->read();

        switch (state)
        {
        case WAIT_SOF:
            if (b == FOC_UART_SOF)
                state = WAIT_LEN;
            break;

        case WAIT_LEN:
            len = b;
            if (len == 0 || len > sizeof(buf))
            {
                state = WAIT_SOF;
            }
            else
            {
                index = 0;
                state = WAIT_BODY;
            }
            break;

        case WAIT_BODY:
            buf[index++] = b;
            if (index == len)
                state = WAIT_CRC;
            break;

        case WAIT_CRC:
            crc_rx = b;

            {
                uint8_t tmp[65];
                tmp[0] = len;
                memcpy(&tmp[1], buf, len);

                uint8_t crc = crc8_atm(tmp, len + 1);

                if (crc == crc_rx)
                {
                    handleFrame(buf, len);
                }
            }

            state = WAIT_SOF;
            break;
        }
    }
}


void TelemetryManager::sendReply(uint8_t cmd,
                                 const uint8_t *payload,
                                 uint8_t payload_len)
{
    uint8_t frame[64];

    uint8_t l = 1 + payload_len;

    frame[0] = FOC_UART_SOF;
    frame[1] = l;
    frame[2] = cmd;

    if (payload_len && payload)
        memcpy(&frame[3], payload, payload_len);

    uint8_t crc = crc8_atm(&frame[1], l + 1);

    frame[3 + payload_len] = crc;

    serial->write(frame, 4 + payload_len);
}


void TelemetryManager::handleFrame(uint8_t *frame, uint8_t len)
{
    uint8_t cmd = frame[0];
    uint8_t *pl = &frame[1];
    uint8_t pl_len = len - 1;

    switch ((foc_uart_cmd_t)cmd)
    {
    case FOC_CMD_GET_ALL_FAST:
        if (allFast){
            sendReply(cmd, (uint8_t*)allFast, 14);
        }
        break;

    case FOC_CMD_GET_STATUS:
        if (status){
            sendReply(cmd, (uint8_t*)status,  3);
        }
        break;

    case FOC_CMD_GET_BUS_VOLTAGE:
        if (vbus_mV)
            sendReply(cmd, (uint8_t*)vbus_mV, 2);
        break;

    case FOC_CMD_GET_BUS_CURRENT:
        if (ibus_mA)
            sendReply(cmd, (uint8_t*)ibus_mA, 4);
        break;

    case FOC_CMD_GET_SPEED:
        if (rpm)
            sendReply(cmd, (uint8_t*)rpm, 4);
        break;

    case FOC_CMD_GET_IQ:
        if (iq_mA)
            sendReply(cmd, (uint8_t*)iq_mA, 2);
        break;

    case FOC_CMD_GET_ID:
        if (id_mA)
            sendReply(cmd, (uint8_t*)id_mA, 2);
        break;

    case FOC_CMD_SET_IQ_PI_GAINS:
        if (pl_len == 8 && setIqPiGains)
        {
            int32_t kp, ki;
            memcpy(&kp, &pl[0], 4);
            memcpy(&ki, &pl[4], 4);

            uint8_t result = setIqPiGains(kp, ki) ? 1 : 0;
            sendReply(cmd, &result, 1);
        }
        break;

    case FOC_CMD_SET_ID_PI_GAINS:
        if (pl_len == 8 && setIdPiGains)
        {
            int32_t kp, ki;
            memcpy(&kp, &pl[0], 4);
            memcpy(&ki, &pl[4], 4);

            uint8_t result = setIdPiGains(kp, ki) ? 1 : 0;
            sendReply(cmd, &result, 1);
        }
        break;

    default:
        /* silently ignore unknown commands */
        break;
    }
}