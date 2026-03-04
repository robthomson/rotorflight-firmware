#include "platform.h"

#ifdef USE_SERIALRX_IBUS

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "pg/serial.h"

#include "rx/ibus2.h"
#include "rx/rx.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif
#if defined(USE_TELEMETRY_IBUS)
#include "telemetry/ibus2.h"
#endif

#define IBUS2_BAUDRATE 1500000
#define IBUS2_FIRST_FRAME_MIN_LEN 4
#define IBUS2_FIRST_FRAME_MAX_LEN 37
#define IBUS2_BASE_CHANNEL_COUNT 18
#define IBUS2_SECOND_FRAME_LENGTH 21
#define IBUS2_DEFAULT_FIRST_FRAME_HEADER 0x00
#define IBUS2_DEFAULT_FIRST_FRAME_LENGTH 31
#define IBUS2_DEFAULT_FIRST_FRAME_ADDRESS 0x07

typedef enum {
    IBUS2_HEADER_UNKNOWN = 0,
    IBUS2_HEADER_LSB,
    IBUS2_HEADER_MSB,
} ibus2HeaderLayout_e;

typedef enum {
    IBUS2_CRC_UNKNOWN = 0,
    IBUS2_CRC_MSB,
    IBUS2_CRC_LSB,
} ibus2CrcMode_e;

static uint32_t ibus2ChannelData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static bool ibus2FrameDone = false;
static bool ibus2FrameFailsafe = false;
static bool ibus2FrameSyncLost = false;
static timeUs_t ibus2LastFrameTimeUs = 0;
static bool ibus2FeedSharedTelemetry = false;
static uint8_t ibus2ExpectedFirstFrameLength = 0;
static uint8_t ibus2ExpectedFirstFrameHeader = 0xFF;
static uint8_t ibus2ExpectedFirstFrameAddress = 0xFF;

static ibus2HeaderLayout_e ibus2HeaderLayout = IBUS2_HEADER_UNKNOWN;
static ibus2CrcMode_e ibus2CrcMode = IBUS2_CRC_UNKNOWN;

uint32_t ibus2RxDbgBytes = 0;
uint32_t ibus2RxDbgFirstFramesSeen = 0;
uint32_t ibus2RxDbgFirstFramesCrcOk = 0;
uint32_t ibus2RxDbgFirstFramesCrcFail = 0;
uint32_t ibus2RxDbgSecondFramesSkipped = 0;
uint32_t ibus2RxDbgSubtype0Seen = 0;
uint32_t ibus2RxDbgSubtype1Seen = 0;
uint32_t ibus2RxDbgSubtype2Seen = 0;
uint32_t ibus2RxDbgDecodeOk = 0;
uint32_t ibus2RxDbgDecodeFail = 0;
uint8_t ibus2RxDbgLastHeader = 0;
uint8_t ibus2RxDbgLastLength = 0;
uint8_t ibus2RxDbgLastSubtype = 0;
uint8_t ibus2RxDbgLastFirstFrame[IBUS2_FIRST_FRAME_MAX_LEN] = { 0 };
uint16_t ibus2RxDbgLastChannels[4] = { 0 };

static uint8_t ibus2Crc8Msb(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x25);
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
                crc = (uint8_t)((crc >> 1) ^ 0xA4);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static bool ibus2CrcOk(const uint8_t *frame, size_t frameLen)
{
    if (frameLen < 2) {
        return false;
    }

    const uint8_t expected = frame[frameLen - 1];

    if (ibus2CrcMode == IBUS2_CRC_LSB) {
        return ibus2Crc8Lsb(frame, frameLen - 1) == expected;
    }

    if (ibus2CrcMode == IBUS2_CRC_MSB) {
        return ibus2Crc8Msb(frame, frameLen - 1) == expected;
    }

    // Before first lock, try both CRC modes and keep whichever validates first.
    const uint8_t lsb = ibus2Crc8Lsb(frame, frameLen - 1);
    if (lsb == expected) {
        ibus2CrcMode = IBUS2_CRC_LSB;
        return true;
    }

    const uint8_t msb = ibus2Crc8Msb(frame, frameLen - 1);
    if (msb == expected) {
        ibus2CrcMode = IBUS2_CRC_MSB;
        return true;
    }

    return false;
}

static void decodeFirstHeader(uint8_t raw, ibus2HeaderLayout_e layout, uint8_t *packetType, uint8_t *packetSubtype, bool *syncLost, bool *failsafe)
{
    if (layout == IBUS2_HEADER_MSB) {
        *packetType = (raw >> 6) & 0x03;
        *packetSubtype = (raw >> 2) & 0x0F;
        *syncLost = (raw & 0x02) != 0;
        *failsafe = (raw & 0x01) != 0;
    } else {
        *packetType = raw & 0x03;
        *packetSubtype = (raw >> 2) & 0x0F;
        *syncLost = (raw & 0x40) != 0;
        *failsafe = (raw & 0x80) != 0;
    }
}

static bool decodeAfhds3PackedChannels(const uint8_t *payload, size_t payloadLen, uint32_t *channelsOut)
{
    if (payloadLen < 28) {
        return false;
    }

    for (uint8_t i = 0; i < 14; i++) {
        const uint8_t low = payload[2 * i];
        const uint8_t highNibble = payload[(2 * i) + 1] & 0x0F;
        channelsOut[i] = (uint32_t)low | ((uint32_t)highNibble << 8);
    }

    channelsOut[14] = (uint32_t)((payload[1] & 0xF0) >> 4) | (uint32_t)(payload[3] & 0xF0) | ((uint32_t)(payload[5] & 0xF0) << 4);
    channelsOut[15] = (uint32_t)((payload[7] & 0xF0) >> 4) | (uint32_t)(payload[9] & 0xF0) | ((uint32_t)(payload[11] & 0xF0) << 4);
    channelsOut[16] = (uint32_t)((payload[13] & 0xF0) >> 4) | (uint32_t)(payload[15] & 0xF0) | ((uint32_t)(payload[17] & 0xF0) << 4);
    channelsOut[17] = (uint32_t)((payload[19] & 0xF0) >> 4) | (uint32_t)(payload[21] & 0xF0) | ((uint32_t)(payload[23] & 0xF0) << 4);

    uint8_t plausible = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (channelsOut[i] >= 750 && channelsOut[i] <= 2250) {
            plausible++;
        }
    }

    return plausible >= 3;
}

static uint32_t extractBitsLE(const uint8_t *data, size_t bitOffset, uint8_t bitCount, size_t dataBytes)
{
    uint32_t value = 0;

    for (uint8_t bit = 0; bit < bitCount; bit++) {
        const size_t index = bitOffset + bit;
        const size_t byteIndex = index >> 3;
        if (byteIndex >= dataBytes) {
            break;
        }
        const uint8_t bitInByte = index & 7;
        const uint8_t bitValue = (data[byteIndex] >> bitInByte) & 0x01;
        value |= ((uint32_t)bitValue << bit);
    }

    return value;
}

static bool decodePacked12Channels(const uint8_t *payload, size_t payloadLen, uint32_t *channelsOut)
{
    // 27 payload bytes == 216 bits == 18 channels * 12 bits.
    const size_t maxChannels = payloadLen >= 27 ? IBUS2_BASE_CHANNEL_COUNT : (payloadLen * 8U) / 12U;
    if (maxChannels < 4) {
        return false;
    }

    for (size_t i = 0; i < maxChannels && i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        const uint32_t raw = extractBitsLE(payload, i * 12U, 12, payloadLen);
        channelsOut[i] = 750 + (raw * 1500U) / 4095U;
    }

    return true;
}

static bool decodeAndStoreChannels(const uint8_t *payload, size_t payloadLen)
{
    uint32_t decoded[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    for (uint8_t i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        decoded[i] = ibus2ChannelData[i];
    }

    bool ok = decodeAfhds3PackedChannels(payload, payloadLen, decoded);
    if (!ok && payloadLen >= 24) {
        ok = decodePacked12Channels(payload, payloadLen, decoded);
    }
    if (!ok) {
        return false;
    }

    for (uint8_t i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        ibus2ChannelData[i] = decoded[i];
    }

    for (uint8_t i = 0; i < 4; i++) {
        ibus2RxDbgLastChannels[i] = (uint16_t)decoded[i];
    }

    return true;
}

static void processFirstFrame(const uint8_t *frame, size_t frameLen, timeUs_t nowUs)
{
    ibus2RxDbgFirstFramesSeen++;
    ibus2RxDbgLastHeader = frame[0];
    ibus2RxDbgLastLength = (uint8_t)frameLen;
    memset(ibus2RxDbgLastFirstFrame, 0, sizeof(ibus2RxDbgLastFirstFrame));
    const size_t copyLen = (frameLen < sizeof(ibus2RxDbgLastFirstFrame)) ? frameLen : sizeof(ibus2RxDbgLastFirstFrame);
    memcpy(ibus2RxDbgLastFirstFrame, frame, copyLen);

    if (frameLen < IBUS2_FIRST_FRAME_MIN_LEN) {
        return;
    }

    const uint8_t packetTypeLsb = frame[0] & 0x03;
    const uint8_t packetTypeMsb = (frame[0] >> 6) & 0x03;

    if (ibus2HeaderLayout == IBUS2_HEADER_UNKNOWN) {
        if (packetTypeLsb == 0 && packetTypeMsb != 0) {
            ibus2HeaderLayout = IBUS2_HEADER_LSB;
        } else if (packetTypeMsb == 0 && packetTypeLsb != 0) {
            ibus2HeaderLayout = IBUS2_HEADER_MSB;
        } else {
            ibus2HeaderLayout = IBUS2_HEADER_LSB;
        }
    }

    uint8_t packetType = 0;
    uint8_t packetSubtype = 0;
    bool syncLost = false;
    bool failsafe = false;
    decodeFirstHeader(frame[0], ibus2HeaderLayout, &packetType, &packetSubtype, &syncLost, &failsafe);
    if (packetType != 0) {
        return;
    }

    const uint8_t payloadStart = 3;
    if (frameLen <= payloadStart + 1) {
        return;
    }
    const size_t payloadLen = frameLen - payloadStart - 1;
    const uint8_t *payload = &frame[payloadStart];

    if (packetSubtype == 0) {
        ibus2RxDbgSubtype0Seen++;
        if (decodeAndStoreChannels(payload, payloadLen)) {
            ibus2RxDbgDecodeOk++;
            ibus2FrameDone = true;
            ibus2FrameFailsafe = failsafe;
            ibus2FrameSyncLost = syncLost;
            ibus2LastFrameTimeUs = nowUs;
        } else {
            ibus2RxDbgDecodeFail++;
        }
    } else if (packetSubtype == 1) {
        ibus2RxDbgSubtype1Seen++;
    } else if (packetSubtype == 2) {
        ibus2RxDbgSubtype2Seen++;
    }
    ibus2RxDbgLastSubtype = packetSubtype;
}

static bool ibus2HeaderMatchesLock(uint8_t byte)
{
    uint8_t packetType = 0;
    uint8_t packetSubtype = 0;
    bool syncLost = false;
    bool failsafe = false;

    decodeFirstHeader(byte, IBUS2_HEADER_LSB, &packetType, &packetSubtype, &syncLost, &failsafe);
    UNUSED(syncLost);
    UNUSED(failsafe);

    // Control channels are transported in subtype 0 frames.
    if (packetType != 0 || packetSubtype != 0) {
        return false;
    }

    if (ibus2ExpectedFirstFrameHeader == 0xFF) {
        return true;
    }

    const uint8_t expectedFixedBits = ibus2ExpectedFirstFrameHeader & 0x3F;
    return (byte & 0x3F) == expectedFixedBits;
}

static void ibus2ProcessByte(uint8_t byte)
{
    static uint8_t rollingBuf[IBUS2_FIRST_FRAME_MAX_LEN];
    static uint8_t rollingWritePos = 0;
    static uint8_t rollingCount = 0;
    static uint8_t bytesToSkipAfterFirstFrame = 0;
    static uint8_t candidateFrame[IBUS2_FIRST_FRAME_MAX_LEN];

    ibus2RxDbgBytes++;
#if defined(USE_TELEMETRY_IBUS)
    if (ibus2FeedSharedTelemetry) {
        ibus2ProcessRxByte(byte);
    }
#endif

    if (bytesToSkipAfterFirstFrame > 0) {
        bytesToSkipAfterFirstFrame--;
        ibus2RxDbgSecondFramesSkipped++;
        return;
    }

    const uint8_t frameLength = ibus2ExpectedFirstFrameLength;
    if (frameLength < IBUS2_FIRST_FRAME_MIN_LEN || frameLength > IBUS2_FIRST_FRAME_MAX_LEN) {
        return;
    }

    rollingBuf[rollingWritePos] = byte;
    rollingWritePos = (rollingWritePos + 1U) % IBUS2_FIRST_FRAME_MAX_LEN;
    if (rollingCount < IBUS2_FIRST_FRAME_MAX_LEN) {
        rollingCount++;
    }

    if (rollingCount < frameLength) {
        return;
    }

    const uint8_t windowStart = (rollingWritePos + IBUS2_FIRST_FRAME_MAX_LEN - frameLength) % IBUS2_FIRST_FRAME_MAX_LEN;
    const uint8_t header = rollingBuf[windowStart];
    if (!ibus2HeaderMatchesLock(header)) {
        return;
    }

    const uint8_t lenByte = rollingBuf[(windowStart + 1U) % IBUS2_FIRST_FRAME_MAX_LEN];
    if (lenByte != frameLength) {
        return;
    }

    const uint8_t address = rollingBuf[(windowStart + 2U) % IBUS2_FIRST_FRAME_MAX_LEN];
    const bool validAddressFlags = (address & 0xC0) == 0;
    const bool addressMatchesLock = (ibus2ExpectedFirstFrameAddress == 0xFF) || (address == ibus2ExpectedFirstFrameAddress);
    if (!validAddressFlags || !addressMatchesLock) {
        return;
    }

    for (uint8_t i = 0; i < frameLength; i++) {
        candidateFrame[i] = rollingBuf[(windowStart + i) % IBUS2_FIRST_FRAME_MAX_LEN];
    }

    if (ibus2CrcOk(candidateFrame, frameLength)) {
        const timeUs_t nowUs = microsISR();
        ibus2RxDbgFirstFramesCrcOk++;
        ibus2ExpectedFirstFrameLength = frameLength;
        ibus2ExpectedFirstFrameHeader = candidateFrame[0];
        ibus2ExpectedFirstFrameAddress = candidateFrame[2];
        processFirstFrame(candidateFrame, frameLength, nowUs);

        // Keep skip counter wired in for debug continuity, but do not skip in RX control parsing path.
        bytesToSkipAfterFirstFrame = 0;
    } else {
        ibus2RxDbgFirstFramesCrcFail++;
    }
}

// Receive ISR callback
static void ibus2DataReceive(uint16_t c, void *data)
{
    UNUSED(data);
    ibus2ProcessByte((uint8_t)c);
}

static uint8_t ibus2FrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    uint8_t frameStatus = RX_FRAME_PENDING;

    if (!ibus2FrameDone) {
        return frameStatus;
    }

    ibus2FrameDone = false;

    if (ibus2FrameFailsafe) {
        frameStatus = RX_FRAME_FAILSAFE;
    } else if (ibus2FrameSyncLost) {
        frameStatus = RX_FRAME_DROPPED;
    } else {
        frameStatus = RX_FRAME_COMPLETE;
    }

    rxRuntimeState->lastRcFrameTimeUs = ibus2LastFrameTimeUs;
    return frameStatus;
}

static float ibus2ReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return ibus2ChannelData[chan];
}


bool ibus2Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeState->channelCount = IBUS2_BASE_CHANNEL_COUNT;
    rxRuntimeState->rxRefreshRate = 20000;
    rxRuntimeState->rcReadRawFn = ibus2ReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ibus2FrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    ibus2HeaderLayout = IBUS2_HEADER_LSB;
    ibus2CrcMode = IBUS2_CRC_LSB;
    ibus2FrameDone = false;
    ibus2FrameFailsafe = false;
    ibus2FrameSyncLost = false;
    ibus2LastFrameTimeUs = 0;
    ibus2FeedSharedTelemetry = false;
    ibus2ExpectedFirstFrameLength = IBUS2_DEFAULT_FIRST_FRAME_LENGTH;
    ibus2ExpectedFirstFrameHeader = IBUS2_DEFAULT_FIRST_FRAME_HEADER;
    ibus2ExpectedFirstFrameAddress = IBUS2_DEFAULT_FIRST_FRAME_ADDRESS;

    ibus2RxDbgBytes = 0;
    ibus2RxDbgFirstFramesSeen = 0;
    ibus2RxDbgFirstFramesCrcOk = 0;
    ibus2RxDbgFirstFramesCrcFail = 0;
    ibus2RxDbgSecondFramesSkipped = 0;
    ibus2RxDbgSubtype0Seen = 0;
    ibus2RxDbgSubtype1Seen = 0;
    ibus2RxDbgSubtype2Seen = 0;
    ibus2RxDbgDecodeOk = 0;
    ibus2RxDbgDecodeFail = 0;
    ibus2RxDbgLastHeader = 0;
    ibus2RxDbgLastLength = 0;
    ibus2RxDbgLastSubtype = 0;
    memset(ibus2RxDbgLastFirstFrame, 0, sizeof(ibus2RxDbgLastFirstFrame));
    memset(ibus2RxDbgLastChannels, 0, sizeof(ibus2RxDbgLastChannels));

    for (uint8_t i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        ibus2ChannelData[i] = 1500;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef USE_TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig, rxRuntimeState->serialrxProvider);
#else
    bool portShared = false;
#endif

    serialPort_t *ibusPort = openSerialPort(
        portConfig->identifier,
        FUNCTION_RX_SERIAL,
        ibus2DataReceive,
        NULL,
        IBUS2_BAUDRATE,
        portShared ? MODE_RXTX : MODE_RX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
            (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            (rxConfig->halfDuplex || portShared ? SERIAL_BIDIR : SERIAL_UNIDIR) |
            (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP)
    );

#ifdef USE_TELEMETRY
    if (portShared && ibusPort) {
        telemetrySharedPort = ibusPort;
#if defined(USE_TELEMETRY_IBUS)
        initSharedIbus2Telemetry(ibusPort);
        ibus2FeedSharedTelemetry = true;
#endif
    }
#endif

    return ibusPort != NULL;
}

#endif
