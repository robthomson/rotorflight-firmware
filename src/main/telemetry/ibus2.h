#pragma once

#include <stdint.h>

#include "drivers/serial.h"

void initIbus2Telemetry(void);

void handleIbus2Telemetry(void);
bool checkIbus2TelemetryState(void);

void configureIbus2TelemetryPort(void);
void freeIbus2TelemetryPort(void);

void initSharedIbus2Telemetry(serialPort_t *port);
void ibus2ProcessRxByte(uint8_t c);
