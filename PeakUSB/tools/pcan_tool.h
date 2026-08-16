/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Small helpers shared by the pcan_* command line tools. Header-only so the
 * tools stay single-translation-unit examples of using the library.
 */
#ifndef PCAN_TOOL_H
#define PCAN_TOOL_H

#include "PeakUSB.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 1 Mbit/s arbitration, 2 Mbit/s data, 80 MHz clock. */
#define PCAN_TOOL_DEFAULT_FD                                            \
    "f_clock=80000000,nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,"    \
    "data_brp=4,data_tseg1=7,data_tseg2=2,data_sjw=1"

/* Bit rates PCAN-Basic has a canned SJA1000 BTR0BTR1 value for. */
static inline TPCANBaudrate pcan_tool_baud(unsigned long bps)
{
    switch (bps) {
    case 1000000: return PCAN_BAUD_1M;
    case 800000:  return PCAN_BAUD_800K;
    case 500000:  return PCAN_BAUD_500K;
    case 250000:  return PCAN_BAUD_250K;
    case 125000:  return PCAN_BAUD_125K;
    case 100000:  return PCAN_BAUD_100K;
    case 95000:   return PCAN_BAUD_95K;
    case 83000:   return PCAN_BAUD_83K;
    case 50000:   return PCAN_BAUD_50K;
    case 47000:   return PCAN_BAUD_47K;
    case 33000:   return PCAN_BAUD_33K;
    case 20000:   return PCAN_BAUD_20K;
    case 10000:   return PCAN_BAUD_10K;
    case 5000:    return PCAN_BAUD_5K;
    default:      return 0;
    }
}

/* Open a channel from a command line argument:
 *   NULL            CAN FD at 1 M / 2 M, falling back to classic 500 kbit/s
 *                   on adapters that do not support CAN FD
 *   "f_clock=..."   CAN FD bit-timing string, passed through verbatim
 *   "500000"        classic CAN at that bit rate
 *   "0x001C"        classic CAN with a raw SJA1000 BTR0BTR1 value
 * On success *fd_mode says which of CAN_Read/CAN_ReadFD to use. */
static inline TPCANStatus pcan_tool_open(TPCANHandle h, const char *rate, int *fd_mode)
{
    TPCANStatus st;

    *fd_mode = 0;
    if (rate && strchr(rate, '=')) {
        *fd_mode = 1;
        return CAN_InitializeFD(h, (TPCANBitrateFD)rate);
    }
    if (rate) {
        unsigned long v = strtoul(rate, NULL, 0);
        TPCANBaudrate b = pcan_tool_baud(v);
        if (!b && v <= 0xFFFF)
            b = (TPCANBaudrate)v;      /* raw BTR0BTR1 */
        if (!b) {
            fprintf(stderr, "unsupported bit rate '%s'\n", rate);
            return PCAN_ERROR_ILLPARAMVAL;
        }
        return CAN_Initialize(h, b, 0, 0, 0);
    }

    *fd_mode = 1;
    st = CAN_InitializeFD(h, (TPCANBitrateFD)PCAN_TOOL_DEFAULT_FD);
    if (st == PCAN_ERROR_ILLOPERATION) {
        fprintf(stderr, "adapter is CAN 2.0 only, using 500 kbit/s\n");
        *fd_mode = 0;
        st = CAN_Initialize(h, PCAN_BAUD_500K, 0, 0, 0);
    }
    return st;
}

/* Smallest CAN FD DLC that carries len bytes (the payload is zero padded). */
static inline unsigned pcan_tool_dlc(unsigned len)
{
    if (len <= 8)  return len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;
}

/* Payload length of a received message, from its DLC and message type. */
static inline unsigned pcan_tool_len(TPCANMessageType type, unsigned dlc)
{
    static const unsigned fd_len[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
    };
    if (type & PCAN_MESSAGE_FD)
        return fd_len[dlc & 0x0F];
    return dlc > 8 ? 8 : dlc;
}

#endif /* PCAN_TOOL_H */
