/*
 * FlySky i-BUS2 receiver protocol (channels + command/response)
 *
 * Control:
 *   - Host (receiver) sends PacketType=0 channel frames.
 * Telemetry/Config:
 *   - Host may send PacketType=1 command frames.
 *   - Device (flight controller) must ONLY transmit PacketType=2 responses
 *     immediately after receiving a PacketType=1 frame.
 *
 * This implementation focuses on:
 *   - Robust framing + CRC verification
 *   - A minimal command/response implementation so the bus stays healthy
 *   - A pragmatic initial channel unpacker that treats the 32 'Channels' bytes
 *     as 16 little-endian signed 16-bit values (range ~[-16384,16384]).
 *
 * IMPORTANT:
 *   FlySky i-BUS2 supports compressed channel payloads (PacketSubtype=0)
 *   and a separate 'channel data type' packet (PacketSubtype=1) that
 *   describes how to decompress. The official spec refers to
 *   SES_UnpackChannels(), but does not include the algorithm.
 *
 *   If your receiver emits compressed data that is NOT 16x int16, you'll
 *   need to replace ibus2UnpackChannels() with the real SES unpack.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_IBUS2

#include "common/time.h"

#include "config/feature.h"

#include "drivers/serial.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/ibus2.h"

// ---- i-BUS2 constants (per spec) ----

#define IBUS2_BAUDRATE 1500000

// PacketType values are stored in the lowest 2 bits of the first byte.
#define IBUS2_PACKETTYPE_CHANNELS 0
#define IBUS2_PACKETTYPE_COMMAND  1
#define IBUS2_PACKETTYPE_RESPONSE 2

// CommandCode is the upper 6 bits of the first byte for PacketType=1/2.
#define IBUS2_CMD_GET_TYPE   1
#define IBUS2_CMD_GET_VALUE  2
#define IBUS2_CMD_GET_PARAM  3
#define IBUS2_CMD_SET_PARAM  4

// Spec appendix lists digital servo type as 0xF8.
#define IBUS2_DEVICE_TYPE_DIGITAL_SERVO 0xF8

// Fixed lengths (command + response). Channels frame is variable.
#define IBUS2_FIXED_FRAME_LEN 21

// CRC8 polynomial described as 0x125 in the spec. For CRC8 implementation
// we use the low 8 bits (0x25) with an implicit x^8 term.
#define IBUS2_CRC8_POLY 0x25

// Channels payload in the spec is 32 bytes.
#define IBUS2_CHANNEL_BYTES 32
#define IBUS2_MAX_CHANNELS 16

// ---- State ----

static uint16_t ibus2ChannelData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static volatile bool ibus2FrameDone = false;
static volatile bool ibus2Failsafe = false;
static volatile timeUs_t ibus2FrameTimeUs = 0;

// command/response
static volatile bool ibus2ResponsePending = false;
static uint8_t ibus2LastCommand[IBUS2_FIXED_FRAME_LEN];

// RX framing buffer
static uint8_t rxBuf[64];
static uint8_t rxPos = 0;
static uint8_t rxTargetLen = 0;
static uint32_t lastByteTimeUs = 0;

static serialPort_t *ibus2Port = NULL;

// ---- CRC8 ----

static uint8_t ibus2Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ IBUS2_CRC8_POLY);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static bool ibus2VerifyCrc(const uint8_t *frame, uint8_t len)
{
    if (len < 2) {
        return false;
    }
    const uint8_t expected = frame[len - 1];
    const uint8_t actual = ibus2Crc8(frame, (uint8_t)(len - 1));
    return expected == actual;
}

// ---- Channels ----

static void ibus2UnpackChannelsPragmatic(const uint8_t channels[IBUS2_CHANNEL_BYTES])
{
    // Pragmatic initial implementation:
    // Treat channels as 16 little-endian signed values, scale to PWM-ish.
    // Output range from SES_UnpackChannels is documented as -16384..16384.
    for (uint8_t i = 0; i < IBUS2_MAX_CHANNELS; i++) {
        const uint16_t lo = channels[i * 2 + 0];
        const uint16_t hi = channels[i * 2 + 1];
        int16_t v = (int16_t)((hi << 8) | lo);

        // Special cases from spec appendix:
        //  -32768: keep last output
        //  -32767: no output
        if (v == (int16_t)0x8000) {
            // keep last
            continue;
        }
        if (v == (int16_t)0x8001) {
            // no output -> set to mid
            ibus2ChannelData[i] = RX_PWM_PULSE_MID;
            continue;
        }

        // Scale (-16384..16384) into (1000..2000) us approx
        // constrain to safe range.
        int32_t scaled = (int32_t)RX_PWM_PULSE_MID + ((int32_t)v * (int32_t)PWM_RANGE) / (2 * 16384);
        if (scaled < RX_PWM_PULSE_MIN) scaled = RX_PWM_PULSE_MIN;
        if (scaled > RX_PWM_PULSE_MAX) scaled = RX_PWM_PULSE_MAX;
        ibus2ChannelData[i] = (uint16_t)scaled;
    }
}

// ---- Response helpers ----

static void ibus2SendResponseFrame(const uint8_t response[IBUS2_FIXED_FRAME_LEN])
{
    if (!ibus2Port) {
        return;
    }

    // Half-duplex: switch to TX just for the response.
    serialSetMode(ibus2Port, MODE_TX);
    for (uint8_t i = 0; i < IBUS2_FIXED_FRAME_LEN; i++) {
        serialWrite(ibus2Port, response[i]);
    }
    waitForSerialPortToFinishTransmitting(ibus2Port);
    serialSetMode(ibus2Port, MODE_RX);
}

static void ibus2BuildAndSendResponse(const uint8_t command[IBUS2_FIXED_FRAME_LEN])
{
    const uint8_t cmdByte = command[0];
    const uint8_t packetType = (uint8_t)(cmdByte & 0x03);
    const uint8_t commandCode = (uint8_t)(cmdByte >> 2);

    if (packetType != IBUS2_PACKETTYPE_COMMAND) {
        return;
    }

    uint8_t resp[IBUS2_FIXED_FRAME_LEN];
    memset(resp, 0, sizeof(resp));

    switch (commandCode) {
    case IBUS2_CMD_GET_TYPE: {
        // Response format (spec):
        //  PacketType=2, CommandCode=1
        //  Type
        //  ValueLength (1..16)
        //  flags: ChannelsTypes, Failsafe, ReceiverInternalSensors
        //  Reserved...
        resp[0] = (uint8_t)((IBUS2_CMD_GET_TYPE << 2) | IBUS2_PACKETTYPE_RESPONSE);
        resp[1] = IBUS2_DEVICE_TYPE_DIGITAL_SERVO;
        resp[2] = 16; // ValueLength (max recommended)
        // flags byte: bit0 ChannelsTypes, bit1 Failsafe, bit2 ReceiverInternalSensors
        // We request failsafe data (bit1) so the host may send it when updated.
        resp[3] = (1 << 1);
        break;
    }
    case IBUS2_CMD_GET_VALUE: {
        // Minimal placeholder telemetry response.
        // Value[14] is application-specific. We leave it zero-filled for now.
        // Many FlySky integrations use VID/PID to identify the device.
        resp[0] = (uint8_t)((IBUS2_CMD_GET_VALUE << 2) | IBUS2_PACKETTYPE_RESPONSE);
        // Value[14] => resp[1..14]
        // VID/PID => resp[15..16]
        resp[15] = 0x01; // VID (FlySky)
        resp[16] = 0x03; // PID (commonly used by FlySky telemetry adapter)
        break;
    }
    case IBUS2_CMD_GET_PARAM: {
        // Echo ParamType and return ParamLength=0 (unsupported) by default.
        resp[0] = (uint8_t)((IBUS2_CMD_GET_PARAM << 2) | IBUS2_PACKETTYPE_RESPONSE);
        resp[1] = command[1];
        resp[2] = command[2];
        resp[3] = 0; // ParamLength
        break;
    }
    case IBUS2_CMD_SET_PARAM: {
        // Echo ParamType and return ParamLength=0 (unsupported).
        resp[0] = (uint8_t)((IBUS2_CMD_SET_PARAM << 2) | IBUS2_PACKETTYPE_RESPONSE);
        resp[1] = command[1];
        resp[2] = command[2];
        resp[3] = 0; // ParamLength == 0 => not supported
        break;
    }
    default:
        // Unknown command: ignore.
        return;
    }

    resp[IBUS2_FIXED_FRAME_LEN - 1] = ibus2Crc8(resp, (uint8_t)(IBUS2_FIXED_FRAME_LEN - 1));
    ibus2SendResponseFrame(resp);
}

// ---- RX byte stream ----

static void ibus2ResetRxState(void)
{
    rxPos = 0;
    rxTargetLen = 0;
}

static void ibus2ProcessFrame(const uint8_t *frame, uint8_t len)
{
    if (!ibus2VerifyCrc(frame, len)) {
        return;
    }

    const uint8_t b0 = frame[0];
    const uint8_t packetType = (uint8_t)(b0 & 0x03);

    if (packetType == IBUS2_PACKETTYPE_CHANNELS) {
        // Channels frame layout:
        //  b0: PacketType/Subtype/flags
        //  b1: Length
        //  b2: Address bits...
        //  b3..: Channels payload
        //  last: CRC8
        //
        // We expect at least: b0,b1,b2 + 32 channel bytes + crc.
        // Some receivers may send shorter frames; we ignore if too short.
        if (len < (uint8_t)(3 + IBUS2_CHANNEL_BYTES + 1)) {
            return;
        }

        const bool failsafeTriggered = (b0 & 0x80) != 0;
        ibus2Failsafe = failsafeTriggered;

        const uint8_t *channels = &frame[3];
        ibus2UnpackChannelsPragmatic(channels);

        ibus2FrameTimeUs = micros();
        ibus2FrameDone = true;
        return;
    }

    if (packetType == IBUS2_PACKETTYPE_COMMAND) {
        if (len != IBUS2_FIXED_FRAME_LEN) {
            return;
        }
        memcpy(ibus2LastCommand, frame, IBUS2_FIXED_FRAME_LEN);
        ibus2ResponsePending = true;
        return;
    }
}

static void ibus2DataReceive(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);

    const uint32_t now = micros();

    // If there is a gap, reset framing.
    if (rxPos && (now - lastByteTimeUs) > 2000) { // conservative gap reset
        ibus2ResetRxState();
    }
    lastByteTimeUs = now;

    const uint8_t b = (uint8_t)c;

    if (rxPos == 0) {
        rxBuf[0] = b;
        rxPos = 1;
        rxTargetLen = 0;
        return;
    }

    // Determine expected length as early as possible.
    if (rxPos == 1) {
        rxBuf[1] = b;
        rxPos = 2;

        const uint8_t packetType = (uint8_t)(rxBuf[0] & 0x03);
        if (packetType == IBUS2_PACKETTYPE_CHANNELS) {
            // Variable-length channels frame: 'Length' is byte1.
            // Spec: Length is the actual length of data in this frame, max 37.
            // We interpret this as bytes excluding CRC. Total bytes = Length + 1.
            const uint8_t declaredLen = rxBuf[1];
            if (declaredLen < 4 || declaredLen > 37) {
                ibus2ResetRxState();
                return;
            }
            rxTargetLen = (uint8_t)(declaredLen + 1);
        } else if (packetType == IBUS2_PACKETTYPE_COMMAND || packetType == IBUS2_PACKETTYPE_RESPONSE) {
            rxTargetLen = IBUS2_FIXED_FRAME_LEN;
        } else {
            ibus2ResetRxState();
            return;
        }
        return;
    }

    // Collect remaining bytes.
    if (rxPos < sizeof(rxBuf)) {
        rxBuf[rxPos++] = b;
    } else {
        ibus2ResetRxState();
        return;
    }

    if (rxTargetLen && rxPos == rxTargetLen) {
        ibus2ProcessFrame(rxBuf, rxTargetLen);
        ibus2ResetRxState();
    }
}

// ---- RX interface ----

static float ibus2ReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return (chan < MAX_SUPPORTED_RC_CHANNEL_COUNT) ? ibus2ChannelData[chan] : RX_PWM_PULSE_MID;
}

static uint8_t ibus2FrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    // Send any pending response *only* after we have received the command.
    if (ibus2ResponsePending) {
        ibus2ResponsePending = false;
        ibus2BuildAndSendResponse(ibus2LastCommand);
    }

    if (!ibus2FrameDone) {
        return RX_FRAME_PENDING;
    }
    ibus2FrameDone = false;

    return RX_FRAME_COMPLETE | (ibus2Failsafe ? RX_FRAME_FAILSAFE : 0);
}

static timeUs_t ibus2FrameTimeUsFn(void)
{
    return ibus2FrameTimeUs;
}

bool ibus2Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeState->channelCount = IBUS2_MAX_CHANNELS;
    rxRuntimeState->rxRefreshRate = 11000; // conservative initial guess; depends on receiver

    rxRuntimeState->rcReadRawFn = ibus2ReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ibus2FrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = ibus2FrameTimeUsFn;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    ibus2ResetRxState();
    ibus2FrameDone = false;
    ibus2Failsafe = false;
    ibus2ResponsePending = false;

    // i-BUS2 is half-duplex by design.
    const portOptions_e options =
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
        (rxConfig->serialrx_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
        SERIAL_BIDIR |
        (rxConfig->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP);

    ibus2Port = openSerialPort(
        portConfig->identifier,
        FUNCTION_RX_SERIAL,
        ibus2DataReceive,
        NULL,
        IBUS2_BAUDRATE,
        MODE_RX,
        options);

    return ibus2Port != NULL;
}

#endif // USE_SERIALRX_IBUS2
