#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY_IBUS)

#include "common/utils.h"

#include "drivers/serial.h"

#include "io/serial.h"

#include "scheduler/scheduler.h"

#include "telemetry/ibus2.h"
#include "telemetry/telemetry.h"
#include "telemetry/sensors.h"
#include "build/debug.h"

#include "rx/rx.h"

#define IBUS2_TASK_PERIOD_US (1000)

#define IBUS2_UART_MODE     (MODE_RXTX)
#define IBUS2_BAUDRATE      (1500000)

#define IBUS2_FRAME_LEN     (21)
#define IBUS2_CRC_SIZE      (1)
#define IBUS2_FIRST_FRAME_MIN_LEN (4)
#define IBUS2_FIRST_FRAME_MAX_LEN (37)

#define IBUS2_PACKET_TYPE_COMMAND  (1)
#define IBUS2_PACKET_TYPE_RESPONSE (2)

#define IBUS2_CMD_GET_VALUE  (2)
#define IBUS2_CMD_GET_PARAM  (3)
#define IBUS2_CMD_SET_PARAM  (4)
#define IBUS2_CMD_GET_TYPE   (1)

#define IBUS2_PARAM_ADVANCED_COMMANDS (0x0001)
#define IBUS2_PARAM_DEVICE_INFO       (0x000B)

#define IBUS2_VID  (0x01)
#define IBUS2_PID  (0x03)

#define IBUS2_PACK_TYPE_NORMAL (0)

// Telemetry adapter sensor types (subset)
#define IBUS2_SENSOR_VOLTAGE (1)
#define IBUS2_SENSOR_CURRENT (2)

// Data units
#define IBUS2_UNIT_V (1)
#define IBUS2_UNIT_A (2)

// Data format
#define IBUS2_FMT_1DP (1)

// Device type (appendix suggests digital servo; use as generic)
#define IBUS2_DEVICE_TYPE_DIGITAL_SERVO (0xF8)


static serialPort_t *ibus2SerialPort = NULL;
static const serialPortConfig_t *ibus2SerialPortConfig;

static uint16_t outboundBytesToIgnoreOnRxCount = 0;
static bool ibus2TelemetryEnabled = false;
static portSharing_e ibus2PortSharing;

static uint8_t ibus2ReceiveBuffer[IBUS2_FRAME_LEN] = { 0 };

typedef enum {
    IBUS2_HEADER_UNKNOWN = 0,
    IBUS2_HEADER_LSB,
    IBUS2_HEADER_MSB
} ibus2HeaderLayout_e;

typedef enum {
    IBUS2_CRC_UNKNOWN = 0,
    IBUS2_CRC_MSB,
    IBUS2_CRC_LSB
} ibus2CrcMode_e;

static ibus2HeaderLayout_e ibus2HeaderLayout = IBUS2_HEADER_UNKNOWN;
static ibus2CrcMode_e ibus2CrcMode = IBUS2_CRC_UNKNOWN;

uint32_t ibus2DbgFramesSeen = 0;
uint32_t ibus2DbgBytesSeen = 0;
uint32_t ibus2DbgNonZeroBytes = 0;
uint32_t ibus2DbgCrcOk = 0;
uint32_t ibus2DbgCmdGetType = 0;
uint32_t ibus2DbgTxCount = 0;
uint32_t ibus2DbgCmdGetValue = 0;
uint32_t ibus2DbgCmdGetParam = 0;
uint32_t ibus2DbgCmdSetParam = 0;
uint32_t ibus2DbgCmdOther = 0;
uint32_t ibus2DbgSecondFrameSeen = 0;
uint32_t ibus2DbgSecondFrameCrcFail = 0;
uint8_t ibus2DbgLastFrame[IBUS2_FRAME_LEN] = { 0 };
uint8_t ibus2DbgLastTxFrame[IBUS2_FRAME_LEN] = { 0 };
uint8_t ibus2DbgLastHeaderRaw = 0;
uint8_t ibus2DbgLastPacketType = 0;
uint8_t ibus2DbgLastCommandCode = 0;

typedef struct {
    uint8_t buf[IBUS2_FIRST_FRAME_MAX_LEN];
    uint8_t idx;
    uint8_t len;
} ibus2FirstFrameParser_t;

typedef struct {
    uint8_t buf[IBUS2_FRAME_LEN];
    uint8_t idx;
} ibus2SecondFrameParser_t;

typedef enum {
    IBUS2_PARSE_SYNC_TO_FIRST_FRAME = 0,
    IBUS2_PARSE_READ_FIRST_FRAME,
    IBUS2_PARSE_WAIT_SECOND_FRAME,
    IBUS2_PARSE_READ_SECOND_FRAME
} ibus2ParseState_e;

static ibus2FirstFrameParser_t ibus2FirstFrameParser = { 0 };
static ibus2SecondFrameParser_t ibus2SecondFrameParser = { 0 };
static ibus2ParseState_e ibus2ParseState = IBUS2_PARSE_SYNC_TO_FIRST_FRAME;
static uint8_t ibus2CommandWindow[IBUS2_FRAME_LEN] = { 0 };
static uint8_t ibus2CommandWindowCount = 0;

static void handleCommandFrame(const uint8_t *frame);


static void pushOntoTail(uint8_t *buffer, size_t bufferLength, uint8_t value)
{
    memmove(buffer, buffer + 1, bufferLength - 1);
    buffer[bufferLength - 1] = value;
}

static void resetIbus2Parsers(void)
{
    ibus2FirstFrameParser.idx = 0;
    ibus2FirstFrameParser.len = 0;
    ibus2SecondFrameParser.idx = 0;
    memset(ibus2CommandWindow, 0, sizeof(ibus2CommandWindow));
    ibus2CommandWindowCount = 0;
    ibus2ParseState = IBUS2_PARSE_SYNC_TO_FIRST_FRAME;
    ibus2HeaderLayout = IBUS2_HEADER_UNKNOWN;
    ibus2CrcMode = IBUS2_CRC_UNKNOWN;
}


static void bitWriterWrite(uint8_t *buffer, size_t bitOffset, uint32_t value, size_t bitCount)
{
    for (size_t i = 0; i < bitCount; i++) {
        const size_t bitIndex = bitOffset + i;
        const size_t byteIndex = bitIndex / 8;
        const size_t bitInByte = 7 - (bitIndex % 8);
        const uint8_t bit = (value >> (bitCount - 1 - i)) & 0x01;
        if (bit) {
            buffer[byteIndex] |= (uint8_t)(1U << bitInByte);
        } else {
            buffer[byteIndex] &= (uint8_t)~(1U << bitInByte);
        }
    }
}


static void packValuePayload(uint8_t value[14],
                             uint8_t packType,
                             uint8_t packLength,
                             uint8_t packIndex,
                             uint8_t type1, uint8_t fmt1, uint8_t unit1, int16_t data1,
                             uint8_t type2, uint8_t fmt2, uint8_t unit2, int16_t data2,
                             uint8_t type3, uint8_t fmt3, uint8_t unit3, int16_t data3)
{
    memset(value, 0, 14);

    size_t bit = 0;
    // 16-bit header: PackType(4) + PackLength(6) + PackCurIndex(6)
    bitWriterWrite(value, bit, packType & 0x0F, 4); bit += 4;
    bitWriterWrite(value, bit, packLength & 0x3F, 6); bit += 6;
    bitWriterWrite(value, bit, packIndex & 0x3F, 6); bit += 6;

    // Each data item: Type(6) + Format(5) + Unit(5) + Value(16) = 32 bits
    bitWriterWrite(value, bit, type1 & 0x3F, 6); bit += 6;
    bitWriterWrite(value, bit, fmt1 & 0x1F, 5); bit += 5;
    bitWriterWrite(value, bit, unit1 & 0x1F, 5); bit += 5;
    bitWriterWrite(value, bit, (uint16_t)data1, 16); bit += 16;

    bitWriterWrite(value, bit, type2 & 0x3F, 6); bit += 6;
    bitWriterWrite(value, bit, fmt2 & 0x1F, 5); bit += 5;
    bitWriterWrite(value, bit, unit2 & 0x1F, 5); bit += 5;
    bitWriterWrite(value, bit, (uint16_t)data2, 16); bit += 16;

    bitWriterWrite(value, bit, type3 & 0x3F, 6); bit += 6;
    bitWriterWrite(value, bit, fmt3 & 0x1F, 5); bit += 5;
    bitWriterWrite(value, bit, unit3 & 0x1F, 5); bit += 5;
    bitWriterWrite(value, bit, (uint16_t)data3, 16); bit += 16;
}


static uint8_t ibus2Crc8Msb(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x25); // poly 0x125 (x^8 + x^5 + x^2 + 1)
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint8_t ibus2Crc8Lsb(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x01) {
                crc = (uint8_t)((crc >> 1) ^ 0xA4); // reflected poly of 0x25
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static uint8_t ibus2CrcCompute(const uint8_t *data, size_t len)
{
    if (ibus2CrcMode == IBUS2_CRC_LSB) {
        return ibus2Crc8Lsb(data, len);
    }
    return ibus2Crc8Msb(data, len);
}

static bool ibus2CrcOk(const uint8_t *frame, size_t frameLen)
{
    if (frameLen < 2) {
        return false;
    }

    const uint8_t expected = frame[frameLen - 1];
    const uint8_t msb = ibus2Crc8Msb(frame, frameLen - IBUS2_CRC_SIZE);
    if (msb == expected) {
        ibus2CrcMode = IBUS2_CRC_MSB;
        return true;
    }
    const uint8_t lsb = ibus2Crc8Lsb(frame, frameLen - IBUS2_CRC_SIZE);
    if (lsb == expected) {
        ibus2CrcMode = IBUS2_CRC_LSB;
        return true;
    }
    return false;
}


static uint16_t getParamType(const uint8_t *frame)
{
    return (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);
}


static void buildResponseHeader(uint8_t *frame, uint8_t commandCode)
{
    if (ibus2HeaderLayout == IBUS2_HEADER_MSB) {
        frame[0] = (uint8_t)((IBUS2_PACKET_TYPE_RESPONSE << 6) | (commandCode & 0x3F));
    } else {
        frame[0] = (uint8_t)((commandCode << 2) | IBUS2_PACKET_TYPE_RESPONSE);
    }
}


static void sendResponse(const uint8_t *frame, size_t len)
{
    if (!ibus2SerialPort) {
        return;
    }
    serialWriteBuf(ibus2SerialPort, frame, len);
    if (len == IBUS2_FRAME_LEN) {
        memcpy(ibus2DbgLastTxFrame, frame, IBUS2_FRAME_LEN);
    }
    outboundBytesToIgnoreOnRxCount += len;
    ibus2DbgTxCount++;
}


static void respondGetParam(const uint8_t *request)
{
    uint8_t response[IBUS2_FRAME_LEN] = { 0 };
    const uint16_t paramType = getParamType(request);

    buildResponseHeader(response, IBUS2_CMD_GET_PARAM);
    response[1] = (uint8_t)(paramType & 0xFF);
    response[2] = (uint8_t)(paramType >> 8);

    // Minimal implementation: report no supported advanced commands and no device info.
    // Extend this to return real device capabilities.
    response[3] = 0;

    response[IBUS2_FRAME_LEN - 1] = ibus2CrcCompute(response, IBUS2_FRAME_LEN - IBUS2_CRC_SIZE);
    sendResponse(response, IBUS2_FRAME_LEN);
}

static void respondGetType(void)
{
    uint8_t response[IBUS2_FRAME_LEN] = { 0 };

    buildResponseHeader(response, IBUS2_CMD_GET_TYPE);
    response[1] = IBUS2_DEVICE_TYPE_DIGITAL_SERVO;
    response[2] = 16; // ValueLength
    // Flags: ChannelsTypes, Failsafe, ReceiverInternalSensors, Reserved
    response[3] = 0;

    response[IBUS2_FRAME_LEN - 1] = ibus2CrcCompute(response, IBUS2_FRAME_LEN - IBUS2_CRC_SIZE);
    sendResponse(response, IBUS2_FRAME_LEN);
}


static void respondSetParam(const uint8_t *request)
{
    uint8_t response[IBUS2_FRAME_LEN] = { 0 };
    const uint16_t paramType = getParamType(request);

    buildResponseHeader(response, IBUS2_CMD_SET_PARAM);
    response[1] = (uint8_t)(paramType & 0xFF);
    response[2] = (uint8_t)(paramType >> 8);

    // ParamLength = 0 indicates parameter not supported (per spec).
    response[3] = 0;

    response[IBUS2_FRAME_LEN - 1] = ibus2CrcCompute(response, IBUS2_FRAME_LEN - IBUS2_CRC_SIZE);
    sendResponse(response, IBUS2_FRAME_LEN);
}


static void respondGetValue(void)
{
    uint8_t response[IBUS2_FRAME_LEN] = { 0 };
    uint8_t value[14];

    const int16_t vbat = (int16_t)telemetrySensorValue(TELEM_BATTERY_VOLTAGE); // 0.1V
    const int16_t ibat = (int16_t)telemetrySensorValue(TELEM_BATTERY_CURRENT); // 0.1A

    buildResponseHeader(response, IBUS2_CMD_GET_VALUE);
    packValuePayload(
        value,
        IBUS2_PACK_TYPE_NORMAL,
        1, // total packets
        1, // packet index
        IBUS2_SENSOR_VOLTAGE, IBUS2_FMT_1DP, IBUS2_UNIT_V, vbat,
        IBUS2_SENSOR_CURRENT, IBUS2_FMT_1DP, IBUS2_UNIT_A, ibat,
        0, 0, 0, 0
    );
    memcpy(&response[1], value, sizeof(value));
    response[15] = IBUS2_VID;
    response[16] = IBUS2_PID;

    response[IBUS2_FRAME_LEN - 1] = ibus2CrcCompute(response, IBUS2_FRAME_LEN - IBUS2_CRC_SIZE);
    sendResponse(response, IBUS2_FRAME_LEN);
}

static bool decodeHeader(uint8_t raw, uint8_t *packetType, uint8_t *commandCode)
{
    if (ibus2HeaderLayout == IBUS2_HEADER_MSB) {
        *packetType = (raw >> 6) & 0x03;
        *commandCode = raw & 0x3F;
        return true;
    }
    if (ibus2HeaderLayout == IBUS2_HEADER_LSB) {
        *packetType = raw & 0x03;
        *commandCode = raw >> 2;
        return true;
    }

    // Unknown layout: try LSB first
    uint8_t pt = raw & 0x03;
    uint8_t cc = raw >> 2;
    if (pt == IBUS2_PACKET_TYPE_COMMAND) {
        ibus2HeaderLayout = IBUS2_HEADER_LSB;
        *packetType = pt;
        *commandCode = cc;
        return true;
    }

    // Try MSB
    pt = (raw >> 6) & 0x03;
    cc = raw & 0x3F;
    if (pt == IBUS2_PACKET_TYPE_COMMAND) {
        ibus2HeaderLayout = IBUS2_HEADER_MSB;
        *packetType = pt;
        *commandCode = cc;
        return true;
    }

    *packetType = raw & 0x03;
    *commandCode = raw >> 2;
    return true;
}

static bool decodeHeaderWithLayout(uint8_t raw, ibus2HeaderLayout_e layout, uint8_t *packetType, uint8_t *commandCode)
{
    if (layout == IBUS2_HEADER_LSB) {
        *packetType = raw & 0x03;
        *commandCode = raw >> 2;
        return true;
    }

    if (layout == IBUS2_HEADER_MSB) {
        *packetType = (raw >> 6) & 0x03;
        *commandCode = raw & 0x3F;
        return true;
    }

    return false;
}

static bool isCommandPayloadSane(const uint8_t *frame, uint8_t commandCode)
{
    UNUSED(frame);
    UNUSED(commandCode);
    return true;
}

static bool isCommandCodeSupported(uint8_t commandCode)
{
    return commandCode <= 0x3F;
}

static uint8_t commandCodeScore(uint8_t commandCode)
{
    if (commandCode >= IBUS2_CMD_GET_TYPE && commandCode <= IBUS2_CMD_SET_PARAM) {
        return 4;
    }
    if (commandCode <= 16) {
        return 3;
    }
    if (commandCode <= 31) {
        return 2;
    }
    return 1;
}

static bool isResponseWorthyCommand(uint8_t commandCode)
{
    switch (commandCode) {
    case IBUS2_CMD_GET_TYPE:
    case IBUS2_CMD_GET_VALUE:
    case IBUS2_CMD_GET_PARAM:
    case IBUS2_CMD_SET_PARAM:
        return true;
    default:
        return false;
    }
}

static void respondUnknownCommand(const uint8_t *request, uint8_t commandCode)
{
    uint8_t response[IBUS2_FRAME_LEN] = { 0 };

    buildResponseHeader(response, commandCode);
    // Echo request payload bytes for compatibility with vendor-specific commands.
    memcpy(&response[1], &request[1], IBUS2_FRAME_LEN - 2);

    response[IBUS2_FRAME_LEN - 1] = ibus2CrcCompute(response, IBUS2_FRAME_LEN - IBUS2_CRC_SIZE);
    sendResponse(response, IBUS2_FRAME_LEN);
}

static bool isPacketTypeWithLayout(uint8_t raw, ibus2HeaderLayout_e layout, uint8_t packetType)
{
    uint8_t pt = 0;
    uint8_t cc = 0;
    if (!decodeHeaderWithLayout(raw, layout, &pt, &cc)) {
        return false;
    }
    UNUSED(cc);
    return pt == packetType;
}

static bool isPotentialFirstFrameHeader(uint8_t raw)
{
    if (ibus2HeaderLayout == IBUS2_HEADER_LSB || ibus2HeaderLayout == IBUS2_HEADER_MSB) {
        return isPacketTypeWithLayout(raw, ibus2HeaderLayout, 0);
    }

    return ((raw & 0x03) == 0) || (((raw >> 6) & 0x03) == 0);
}

static bool detectCommandHeader(uint8_t raw, ibus2HeaderLayout_e *layout)
{
    uint8_t pt = 0;
    uint8_t cc = 0;

    if (ibus2HeaderLayout == IBUS2_HEADER_LSB || ibus2HeaderLayout == IBUS2_HEADER_MSB) {
        decodeHeaderWithLayout(raw, ibus2HeaderLayout, &pt, &cc);
        if (pt == IBUS2_PACKET_TYPE_COMMAND) {
            *layout = ibus2HeaderLayout;
            return true;
        }
    }

    decodeHeaderWithLayout(raw, IBUS2_HEADER_LSB, &pt, &cc);
    if (pt == IBUS2_PACKET_TYPE_COMMAND) {
        *layout = IBUS2_HEADER_LSB;
        return true;
    }

    decodeHeaderWithLayout(raw, IBUS2_HEADER_MSB, &pt, &cc);
    if (pt == IBUS2_PACKET_TYPE_COMMAND) {
        *layout = IBUS2_HEADER_MSB;
        return true;
    }

    return false;
}

static void startFirstFrameParse(uint8_t firstByte)
{
    ibus2FirstFrameParser.idx = 0;
    ibus2FirstFrameParser.len = 0;
    ibus2FirstFrameParser.buf[ibus2FirstFrameParser.idx++] = firstByte;
    ibus2ParseState = IBUS2_PARSE_READ_FIRST_FRAME;
}

static bool handleIbus2CommandFrameIfValid(const uint8_t *frame)
{
    ibus2DbgSecondFrameSeen++;

    if (!ibus2CrcOk(frame, IBUS2_FRAME_LEN)) {
        ibus2DbgSecondFrameCrcFail++;
        return false;
    }

    uint8_t packetType = 0;
    uint8_t commandCode = 0;
    decodeHeader(frame[0], &packetType, &commandCode);
    if (packetType != IBUS2_PACKET_TYPE_COMMAND ||
        !isCommandCodeSupported(commandCode) ||
        !isCommandPayloadSane(frame, commandCode)) {
        return false;
    }

    ibus2DbgCrcOk++;
    memcpy(ibus2DbgLastFrame, frame, sizeof(ibus2DbgLastFrame));
    ibus2DbgLastHeaderRaw = frame[0];
    {
        ibus2DbgLastPacketType = packetType;
        ibus2DbgLastCommandCode = commandCode;
    }
    handleCommandFrame(frame);

    return true;
}

static bool parseStreamFrameByte(uint8_t c)
{
    switch (ibus2ParseState) {
    case IBUS2_PARSE_SYNC_TO_FIRST_FRAME:
        if (isPotentialFirstFrameHeader(c)) {
            startFirstFrameParse(c);
        }
        return false;

    case IBUS2_PARSE_READ_FIRST_FRAME:
        ibus2FirstFrameParser.buf[ibus2FirstFrameParser.idx++] = c;

        if (ibus2FirstFrameParser.idx == 2) {
            const uint8_t len = ibus2FirstFrameParser.buf[1];
            if (len < IBUS2_FIRST_FRAME_MIN_LEN || len > IBUS2_FIRST_FRAME_MAX_LEN) {
                ibus2ParseState = IBUS2_PARSE_SYNC_TO_FIRST_FRAME;
                if (isPotentialFirstFrameHeader(c)) {
                    startFirstFrameParse(c);
                }
                return false;
            }
            ibus2FirstFrameParser.len = len;
        }

        if (ibus2FirstFrameParser.len && ibus2FirstFrameParser.idx >= ibus2FirstFrameParser.len) {
            if (ibus2CrcOk(ibus2FirstFrameParser.buf, ibus2FirstFrameParser.len)) {
                const uint8_t header = ibus2FirstFrameParser.buf[0];
                const bool lsbType0 = ((header & 0x03) == 0);
                const bool msbType0 = ((((header >> 6) & 0x03) == 0));

                ibus2DbgFramesSeen++;
                if (lsbType0 && !msbType0) {
                    ibus2HeaderLayout = IBUS2_HEADER_LSB;
                } else if (msbType0 && !lsbType0) {
                    ibus2HeaderLayout = IBUS2_HEADER_MSB;
                }

                ibus2ParseState = IBUS2_PARSE_WAIT_SECOND_FRAME;
            } else {
                ibus2ParseState = IBUS2_PARSE_SYNC_TO_FIRST_FRAME;
            }
        }
        return false;

    case IBUS2_PARSE_WAIT_SECOND_FRAME:
    {
        ibus2HeaderLayout_e layout = IBUS2_HEADER_UNKNOWN;
        if (detectCommandHeader(c, &layout)) {
            ibus2HeaderLayout = layout;
            ibus2SecondFrameParser.idx = 0;
            ibus2SecondFrameParser.buf[ibus2SecondFrameParser.idx++] = c;
            ibus2ParseState = IBUS2_PARSE_READ_SECOND_FRAME;
        } else if (isPotentialFirstFrameHeader(c)) {
            startFirstFrameParse(c);
        } else {
            ibus2ParseState = IBUS2_PARSE_SYNC_TO_FIRST_FRAME;
        }
        return false;
    }

    case IBUS2_PARSE_READ_SECOND_FRAME:
        ibus2SecondFrameParser.buf[ibus2SecondFrameParser.idx++] = c;
        if (ibus2SecondFrameParser.idx >= IBUS2_FRAME_LEN) {
            ibus2ParseState = IBUS2_PARSE_SYNC_TO_FIRST_FRAME;
            return handleIbus2CommandFrameIfValid(ibus2SecondFrameParser.buf);
        }
        return false;
    }

    return false;
}

static bool isLikelyCommandFrame(const uint8_t *frame, ibus2HeaderLayout_e *layout, ibus2CrcMode_e *crcMode)
{
    const uint8_t expected = frame[IBUS2_FRAME_LEN - 1];
    const uint8_t msb = ibus2Crc8Msb(frame, IBUS2_FRAME_LEN - IBUS2_CRC_SIZE);
    const uint8_t lsb = ibus2Crc8Lsb(frame, IBUS2_FRAME_LEN - IBUS2_CRC_SIZE);
    const bool msbOk = (msb == expected);
    const bool lsbOk = (lsb == expected);

    if (!msbOk && !lsbOk) {
        return false;
    }

    const ibus2HeaderLayout_e candidates[2] = { IBUS2_HEADER_LSB, IBUS2_HEADER_MSB };
    ibus2HeaderLayout_e bestLayout = IBUS2_HEADER_UNKNOWN;
    uint8_t bestScore = 0;
    uint8_t bestCommand = 0xFF;

    for (int i = 0; i < 2; i++) {
        uint8_t packetType = 0;
        uint8_t commandCode = 0;
        decodeHeaderWithLayout(frame[0], candidates[i], &packetType, &commandCode);
        if (packetType != IBUS2_PACKET_TYPE_COMMAND ||
            !isCommandCodeSupported(commandCode) ||
            !isCommandPayloadSane(frame, commandCode)) {
            continue;
        }

        uint8_t score = commandCodeScore(commandCode);
        if (ibus2HeaderLayout == candidates[i]) {
            score += 1;
        }

        if (bestLayout == IBUS2_HEADER_UNKNOWN ||
            score > bestScore ||
            (score == bestScore && commandCode < bestCommand)) {
            bestLayout = candidates[i];
            bestScore = score;
            bestCommand = commandCode;
        }
    }

    if (bestLayout != IBUS2_HEADER_UNKNOWN) {
        *layout = bestLayout;
        *crcMode = msbOk ? IBUS2_CRC_MSB : IBUS2_CRC_LSB;
        return true;
    }

    return false;
}

static bool parseRollingCommandByte(uint8_t c)
{
    pushOntoTail(ibus2CommandWindow, IBUS2_FRAME_LEN, c);
    if (ibus2CommandWindowCount < IBUS2_FRAME_LEN) {
        ibus2CommandWindowCount++;
        return false;
    }

    ibus2HeaderLayout_e layout = IBUS2_HEADER_UNKNOWN;
    ibus2CrcMode_e crcMode = IBUS2_CRC_UNKNOWN;
    if (!isLikelyCommandFrame(ibus2CommandWindow, &layout, &crcMode)) {
        return false;
    }

    ibus2HeaderLayout = layout;
    ibus2CrcMode = crcMode;
    return handleIbus2CommandFrameIfValid(ibus2CommandWindow);
}


static void handleCommandFrame(const uint8_t *frame)
{
    uint8_t packetType = 0;
    uint8_t commandCode = 0;
    decodeHeader(frame[0], &packetType, &commandCode);

    if (packetType != IBUS2_PACKET_TYPE_COMMAND) {
        return;
    }

    if (commandCode == IBUS2_CMD_GET_TYPE) {
        ibus2DbgCmdGetType++;
    } else if (commandCode == IBUS2_CMD_GET_VALUE) {
        ibus2DbgCmdGetValue++;
    } else if (commandCode == IBUS2_CMD_GET_PARAM) {
        ibus2DbgCmdGetParam++;
    } else if (commandCode == IBUS2_CMD_SET_PARAM) {
        ibus2DbgCmdSetParam++;
    } else {
        ibus2DbgCmdOther++;
    }

    if (isResponseWorthyCommand(commandCode)) {
        switch (commandCode) {
        case IBUS2_CMD_GET_TYPE:
            respondGetType();
            break;
        case IBUS2_CMD_GET_VALUE:
            respondGetValue();
            break;
        case IBUS2_CMD_GET_PARAM:
            respondGetParam(frame);
            break;
        case IBUS2_CMD_SET_PARAM:
            respondSetParam(frame);
            break;
        default:
            break;
        }
    } else {
        respondUnknownCommand(frame, commandCode);
    }
}


void initIbus2Telemetry(void)
{
    ibus2SerialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_IBUS);
    ibus2PortSharing = determinePortSharing(ibus2SerialPortConfig, FUNCTION_TELEMETRY_IBUS);
    ibus2TelemetryEnabled = false;
    resetIbus2Parsers();
}


void handleIbus2Telemetry(void)
{
    if (!ibus2TelemetryEnabled) {
        return;
    }

    if (ibus2SerialPort == telemetrySharedPort &&
        rxRuntimeState.serialrxProvider == SERIALRX_IBUS2) {
        // Shared port: RX driver feeds bytes via ibus2ProcessRxByte().
        return;
    }

    while (serialRxBytesWaiting(ibus2SerialPort) > 0) {
        uint8_t c = serialRead(ibus2SerialPort);
        ibus2ProcessRxByte(c);
    }
}


bool checkIbus2TelemetryState(void)
{
    if (rxRuntimeState.serialrxProvider == SERIALRX_IBUS2 &&
        telemetrySharedPort != NULL) {
        return false;
    }

    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(ibus2PortSharing);

    if (newTelemetryEnabledValue == ibus2TelemetryEnabled) {
        return false;
    }

    if (newTelemetryEnabledValue) {
        rescheduleTask(TASK_TELEMETRY, IBUS2_TASK_PERIOD_US);
        configureIbus2TelemetryPort();
    } else {
        freeIbus2TelemetryPort();
    }

    return true;
}


void configureIbus2TelemetryPort(void)
{
    if (!ibus2SerialPortConfig) {
        return;
    }

    if (isSerialPortShared(ibus2SerialPortConfig, FUNCTION_RX_SERIAL, FUNCTION_TELEMETRY_IBUS)) {
        // RX driver will open and feed telemetry.
        return;
    }

    ibus2SerialPort = openSerialPort(
        ibus2SerialPortConfig->identifier,
        FUNCTION_TELEMETRY_IBUS,
        NULL,
        NULL,
        IBUS2_BAUDRATE,
        IBUS2_UART_MODE,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_BIDIR |
            (telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            (telemetryConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP)
        );

    if (!ibus2SerialPort) {
        return;
    }

    ibus2TelemetryEnabled = true;
    outboundBytesToIgnoreOnRxCount = 0;
    resetIbus2Parsers();
}


void freeIbus2TelemetryPort(void)
{
    closeSerialPort(ibus2SerialPort);
    ibus2SerialPort = NULL;
    ibus2TelemetryEnabled = false;
    resetIbus2Parsers();
}


void initSharedIbus2Telemetry(serialPort_t *port)
{
    ibus2SerialPort = port;
    ibus2TelemetryEnabled = true;
    outboundBytesToIgnoreOnRxCount = 0;
    resetIbus2Parsers();
}


void ibus2ProcessRxByte(uint8_t c)
{
    if (outboundBytesToIgnoreOnRxCount) {
        outboundBytesToIgnoreOnRxCount--;
        return;
    }

    ibus2DbgBytesSeen++;
    if (c != 0) {
        ibus2DbgNonZeroBytes++;
    }

    // Sliding buffer for last bytes (debug only)
    pushOntoTail(ibus2ReceiveBuffer, IBUS2_FRAME_LEN, c);

    const bool handledByStreamParser = parseStreamFrameByte(c);
    if (!handledByStreamParser) {
        parseRollingCommandByte(c);
    }

    DEBUG_SET(DEBUG_TTA, 0, ibus2DbgBytesSeen);
    DEBUG_SET(DEBUG_TTA, 1, ibus2DbgNonZeroBytes);
    DEBUG_SET(DEBUG_TTA, 2, ibus2DbgCrcOk);
    DEBUG_SET(DEBUG_TTA, 3, ibus2DbgTxCount);
}

#endif
