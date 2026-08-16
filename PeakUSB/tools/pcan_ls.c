/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * List every PCAN USB channel the library can see, with the handle that
 * pcan_recv / pcan_send and PCAN-Basic applications use.
 */
#include "PeakUSB.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    TPCANChannelInformation info[16];
    DWORD count = 0, i;
    char err[256];

    memset(info, 0, sizeof(info));
    if (CAN_GetValue(PCAN_NONEBUS, PCAN_ATTACHED_CHANNELS_COUNT,
                     &count, sizeof(count))) {
        CAN_GetErrorText(PCAN_ERROR_NODRIVER, 0x09, err);
        fprintf(stderr, "cannot enumerate (%s)\n", err);
        return 1;
    }
    printf("PCAN USB channels: %u\n", count);
    if (!count)
        return 0;
    if (count > sizeof(info) / sizeof(info[0]))
        count = sizeof(info) / sizeof(info[0]);
    if (CAN_GetValue(PCAN_NONEBUS, PCAN_ATTACHED_CHANNELS, info,
                     (DWORD)(count * sizeof(info[0])))) {
        fprintf(stderr, "PCAN_ATTACHED_CHANNELS failed\n");
        return 1;
    }
    for (i = 0; i < count; i++)
        printf("  handle=0x%02X  %-18s  CAN%u  id=%u  %s  %s\n",
               info[i].channel_handle,
               info[i].device_name,
               info[i].controller_number + 1,
               info[i].device_id,
               (info[i].device_features & FEATURE_FD_CAPABLE) ? "FD     " : "classic",
               info[i].channel_condition == PCAN_CHANNEL_OCCUPIED ? "in use"
                                                                  : "available");
    return 0;
}
