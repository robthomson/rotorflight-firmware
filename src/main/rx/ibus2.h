/*
 * FlySky i-BUS2 receiver protocol (channels + command/response)
 *
 * Notes:
 *  - UART 1.5 Mbit/s, 8N1, half-duplex.
 *  - Host (receiver) sends:
 *      Frame 1: channels (PacketType=0, variable length up to 37 bytes)
 *      Frame 2: command  (PacketType=1, fixed 21 bytes, optional)
 *    Device (flight controller) must ONLY transmit a response frame
 *    after receiving a command frame.
 *
 * Protocol reference: IBU2 UART Protocol (v2.x).
 */

#pragma once

#include "pg/rx.h"
#include "rx/rx.h"

bool ibus2Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState);
