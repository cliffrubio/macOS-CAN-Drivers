/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * Transmit one classic CAN frame.
 *   kvaser_send 0x00140000 123 11 22 33
 */
#include "KvaserUSB.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    kv_open_opts opts;
    kv_msg msg;
    uint32_t handle;
    int err, i;

    if (argc < 3) {
        fprintf(stderr,
                "usage: %s <handle> <id-hex> [byte-hex ...]\n"
                "  handle  as printed by kvaser_ls\n"
                "  id-hex  identifier; > 0x7FF is sent as a 29-bit id\n",
                argv[0]);
        return 1;
    }

    memset(&opts, 0, sizeof(opts));
    opts.bitrate = 500000;
    handle = (uint32_t)strtoul(argv[1], NULL, 0);

    memset(&msg, 0, sizeof(msg));
    msg.id = (uint32_t)strtoul(argv[2], NULL, 16);
    if (msg.id > 0x7FF)
        msg.flags |= KV_FLAG_EXTENDED;
    for (i = 3; i < argc && msg.len < 8; i++)
        msg.data[msg.len++] = (uint8_t)strtoul(argv[i], NULL, 16);
    msg.dlc = msg.len;

    err = kv_open(handle, &opts);
    if (err) {
        fprintf(stderr, "open 0x%08X: %s\n", handle, kv_strerror(err));
        return 1;
    }
    err = kv_write(handle, &msg);
    if (err)
        fprintf(stderr, "write: %s\n", kv_strerror(err));
    else
        printf("sent %03X [%u]\n", msg.id, msg.len);
    /* Let the transmit URB reach the adapter before we go bus-off. */
    usleep(20000);
    kv_close(handle);
    return err ? 1 : 0;
}
