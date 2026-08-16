/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * List every CAN channel of every attached Kvaser USB adapter.
 * The printed handle is what kvaser_recv / kvaser_send expect.
 */
#include "KvaserUSB.h"

#include <stdio.h>

int main(void)
{
    kv_channel_info ch[KV_MAX_CHANNELS];
    int n, i;

    n = kv_scan(ch, KV_MAX_CHANNELS);
    if (n < 0) {
        fprintf(stderr, "scan failed: %s\n", kv_strerror(n));
        return 1;
    }
    if (n == 0) {
        printf("No Kvaser USB adapters found (VID 0x0BFD).\n");
        return 0;
    }
    printf("%d channel(s):\n", n);
    for (i = 0; i < n; i++)
        printf("  handle=0x%08X  %-32s  pid=0x%04X  CAN%u/%u  %s\n",
               ch[i].handle, ch[i].name, ch[i].pid,
               ch[i].channel + 1, ch[i].channel_count,
               ch[i].fd_capable ? "FD" : "classic");
    return 0;
}
