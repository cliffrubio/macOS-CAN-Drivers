/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Print every frame received on one PCAN channel until interrupted.
 *
 *   pcan_recv                       CAN1, 1 Mbit/s + 2 Mbit/s data (FD)
 *   pcan_recv 0x52                  CAN2 of a Pro / Pro FD
 *   pcan_recv 0x51 500000           classic CAN at 500 kbit/s
 *   pcan_recv 0x51 "f_clock=..."    explicit CAN FD bit-timing string
 */
#include "PeakUSB.h"

#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pcan_tool.h"

static volatile sig_atomic_t g_stop;

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

int main(int argc, char **argv)
{
    TPCANHandle h = PCAN_USBBUS1;
    const char *rate = NULL;
    TPCANStatus st;
    DWORD last_status = 0;
    int got_status = 0, fd = -1, fd_mode;
    char err[256];

    if (argc > 1 && strcmp(argv[1], "-h") == 0) {
        fprintf(stderr, "usage: %s [handle] [bitrate|fd-bitrate-string]\n", argv[0]);
        return 1;
    }
    if (argc > 1)
        h = (TPCANHandle)strtoul(argv[1], NULL, 0);
    if (argc > 2)
        rate = argv[2];

    signal(SIGINT, on_sigint);

    st = pcan_tool_open(h, rate, &fd_mode);
    if (st != PCAN_ERROR_OK) {
        CAN_GetErrorText(st, 0x09, err);
        fprintf(stderr, "open handle 0x%02X failed: %s (0x%x)\n", h, err, st);
        return 1;
    }
    CAN_GetValue(h, PCAN_RECEIVE_EVENT, &fd, sizeof(fd));
    fprintf(stderr,
            "listening on handle 0x%02X (%s, event fd %d), Ctrl-C to stop\n"
            "the channel is ACTIVE, so it acknowledges frames on the bus\n",
            h, fd_mode ? "CAN FD" : "classic CAN", fd);

    while (!g_stop) {
        struct pollfd p = { .fd = fd, .events = POLLIN, .revents = 0 };
        if (poll(&p, 1, 1000) < 0)
            break;

        for (;;) {
            TPCANMsgFD m;
            TPCANTimestampFD ts = 0;
            unsigned i, n;

            if (fd_mode) {
                st = CAN_ReadFD(h, &m, &ts);
            } else {
                TPCANMsg c;
                TPCANTimestamp cts;
                memset(&m, 0, sizeof(m));
                st = CAN_Read(h, &c, &cts);
                if (!(st & PCAN_ERROR_QRCVEMPTY)) {
                    m.ID = c.ID;
                    m.MSGTYPE = c.MSGTYPE;
                    m.DLC = c.LEN;
                    memcpy(m.DATA, c.DATA, sizeof(c.DATA));
                    ts = ((TPCANTimestampFD)cts.millis_overflow << 32) * 1000u +
                         (TPCANTimestampFD)cts.millis * 1000u + cts.micros;
                }
            }
            if (st & PCAN_ERROR_QRCVEMPTY)
                break;
            /* Any other failure means there is no frame in the buffer, so
             * carrying on would print uninitialised stack forever. */
            if (st != PCAN_ERROR_OK && !(st & PCAN_ERROR_ANYBUSERR)) {
                CAN_GetErrorText(st, 0x09, err);
                fprintf(stderr, "read failed: %s (0x%x)\n", err, st);
                g_stop = 1;
                break;
            }

            if (m.MSGTYPE & PCAN_MESSAGE_STATUS) {
                if (got_status && m.ID == last_status)
                    continue;
                last_status = m.ID;
                got_status = 1;
                fprintf(stderr, "bus status 0x%x%s%s%s%s\n", m.ID,
                        (m.ID & PCAN_ERROR_BUSPASSIVE) ? " BUSPASSIVE" : "",
                        (m.ID & PCAN_ERROR_BUSOFF)     ? " BUSOFF" : "",
                        (m.ID & PCAN_ERROR_BUSWARNING) ? " WARNING" : "",
                        (m.ID & PCAN_ERROR_BUSLIGHT)   ? " LIGHT" : "");
                continue;
            }

            n = pcan_tool_len(m.MSGTYPE, m.DLC);
            printf("%12llu  %s%s%s  %08X  [%2u]",
                   (unsigned long long)ts,
                   (m.MSGTYPE & PCAN_MESSAGE_EXTENDED) ? "X" : "S",
                   (m.MSGTYPE & PCAN_MESSAGE_FD)       ? "F" : " ",
                   (m.MSGTYPE & PCAN_MESSAGE_BRS)      ? "B" : " ",
                   m.ID, n);
            if (!(m.MSGTYPE & PCAN_MESSAGE_RTR))
                for (i = 0; i < n; i++)
                    printf(" %02X", m.DATA[i]);
            putchar('\n');
            fflush(stdout);
        }
    }
    CAN_Uninitialize(h);
    return 0;
}
