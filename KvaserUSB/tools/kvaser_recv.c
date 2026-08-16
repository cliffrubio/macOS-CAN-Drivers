/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * Print every frame received on one channel until interrupted.
 */
#include "KvaserUSB.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop;

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s <handle> [bitrate] [data-bitrate]\n"
            "  handle        as printed by kvaser_ls, e.g. 0x00140000\n"
            "  bitrate       arbitration bit rate in bit/s (default 500000)\n"
            "  data-bitrate  FD data bit rate in bit/s; opens the channel in\n"
            "                CAN FD mode (FD-capable adapters only)\n",
            argv0);
}

int main(int argc, char **argv)
{
    kv_open_opts opts;
    kv_msg msg;
    uint32_t handle;
    int err, i;

    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }
    memset(&opts, 0, sizeof(opts));
    opts.bitrate = 500000;
    handle = (uint32_t)strtoul(argv[1], NULL, 0);
    if (argc >= 3)
        opts.bitrate = atoi(argv[2]);
    if (argc >= 4) {
        opts.data_bitrate = atoi(argv[3]);
        opts.can_fd = 1;
    }

    signal(SIGINT, on_sigint);

    err = kv_open(handle, &opts);
    if (err) {
        fprintf(stderr, "open 0x%08X: %s\n", handle, kv_strerror(err));
        return 1;
    }
    printf("listening on 0x%08X at %d bit/s%s (Ctrl-C to stop)\n",
           handle, opts.bitrate, opts.can_fd ? " CAN FD" : "");

    while (!g_stop) {
        err = kv_read(handle, &msg);
        if (err == KV_ERR_EMPTY) {
            usleep(1000);
            continue;
        }
        if (err) {
            fprintf(stderr, "read: %s\n", kv_strerror(err));
            break;
        }
        printf("%10llu  %s%s%s%s  %08X  [%2u]",
               (unsigned long long)msg.ts_us,
               (msg.flags & KV_FLAG_EXTENDED) ? "X" : "S",
               (msg.flags & KV_FLAG_FD)       ? "F" : " ",
               (msg.flags & KV_FLAG_BRS)      ? "B" : " ",
               (msg.flags & KV_FLAG_ERROR)    ? "E" : " ",
               msg.id, msg.len);
        for (i = 0; i < msg.len; i++)
            printf(" %02X", msg.data[i]);
        putchar('\n');
        fflush(stdout);
    }
    kv_close(handle);
    return 0;
}
