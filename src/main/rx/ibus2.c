#include "platform.h"

#ifdef USE_SERIALRX_IBUS

#include <stdbool.h>
#include <stdint.h>

#include "drivers/serial.h"

#include "io/serial.h"

#include "pg/serial.h"

#include "rx/rx.h"

#include "config/feature.h"

#include "telemetry/ibus2.h"
#include "telemetry/telemetry.h"

#define IBUS2_BAUDRATE 1500000

static uint32_t ibus2ChannelData[MAX_SUPPORTED_RC_CHANNEL_COUNT];


static void ibus2DataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    // Feed IBUS2 telemetry handler (command frames) from the shared RX port.
    ibus2ProcessRxByte((uint8_t)c);

    // TODO: Implement IBUS2 channel frame parsing (PacketType=0).
}


static uint8_t ibus2FrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);
    // TODO: Implement proper IBUS2 channel parsing and frame status.
    return RX_FRAME_PENDING;
}


static float ibus2ReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    UNUSED(rxRuntimeState);
    return ibus2ChannelData[chan];
}


bool ibus2Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    rxRuntimeState->channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT;
    rxRuntimeState->rxRefreshRate = 20000;
    rxRuntimeState->rcReadRawFn = ibus2ReadRawRC;
    rxRuntimeState->rcFrameStatusFn = ibus2FrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;

    for (uint8_t i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        ibus2ChannelData[i] = 1500;
    }

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

    bool portShared = false;
#ifdef USE_TELEMETRY
    if (featureIsEnabled(FEATURE_TELEMETRY)) {
        portShared = true;
    }
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

#if defined(USE_TELEMETRY) && defined(USE_TELEMETRY_IBUS)
    if (portShared) {
        telemetrySharedPort = ibusPort;
        initSharedIbus2Telemetry(ibusPort);
    }
#endif

    return ibusPort != NULL;
}

#endif
