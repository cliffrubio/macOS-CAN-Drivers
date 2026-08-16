/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Transmit one frame.
 *
 *   pcan_send 123 11 22 33            CAN1, default CAN FD bit rate
 *   pcan_send 0x52 123 11 22 33       CAN2 of a Pro / Pro FD
 *   pcan_send -b 500000 123 11 22     classic CAN at 500 kbit/s
 */
#include "PeakUSB.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "pcan_tool.h"

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [-b bitrate] [handle] <id-hex> [byte-hex ...]\n"
            "  handle   0x51..0x58, as printed by pcan_ls (default 0x51)\n"
            "  bitrate  500000, 0x001C, or a full f_clock=... FD string\n",
            argv0);
}

int main(int argc, char **argv)
{
    TPCANHandle h = PCAN_USBBUS1;
    const char *rate = NULL;
    TPCANStatus st;
    TPCANMsgFD m;
    char err[256];
    int i, arg = 1, fd_mode;

    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        usage(argv[0]);
        return 1;
    }
    if (arg < argc && strcmp(argv[arg], "-b") == 0) {
        /* Without its value, -b must not fall through and be parsed as the
         * identifier: that would transmit an unintended frame. */
        if (arg + 1 >= argc) {
            fprintf(stderr, "-b needs a bit rate\n");
            usage(argv[0]);
            return 1;
        }
        rate = argv[arg + 1];
        arg += 2;
    }
    if (arg < argc && strncmp(argv[arg], "0x", 2) == 0 &&
        strtoul(argv[arg], NULL, 0) <= PCAN_USBBUS8 &&
        strtoul(argv[arg], NULL, 0) >= PCAN_USBBUS1) {
        h = (TPCANHandle)strtoul(argv[arg], NULL, 0);
        arg++;
    }
    if (arg >= argc) {
        usage(argv[0]);
        return 1;
    }

    st = pcan_tool_open(h, rate, &fd_mode);
    if (st != PCAN_ERROR_OK) {
        CAN_GetErrorText(st, 0x09, err);
        fprintf(stderr, "open handle 0x%02X failed: %s (0x%x)\n", h, err, st);
        return 1;
    }

    memset(&m, 0, sizeof(m));
    {
        char *endp;
        unsigned long id = strtoul(argv[arg], &endp, 16);
        if (endp == argv[arg] || *endp || id > 0x1FFFFFFFul) {
            fprintf(stderr, "bad identifier '%s' (hex, max 1FFFFFFF)\n", argv[arg]);
            CAN_Uninitialize(h);
            return 1;
        }
        m.ID = (DWORD)id;
        arg++;
    }
    if (m.ID > 0x7FF)
        m.MSGTYPE |= PCAN_MESSAGE_EXTENDED;
    if (fd_mode)
        m.MSGTYPE |= PCAN_MESSAGE_FD | PCAN_MESSAGE_BRS;
    for (i = 0; arg < argc && i < 64; arg++, i++)
        m.DATA[i] = (BYTE)strtoul(argv[arg], NULL, 16);

    if (fd_mode) {
        m.DLC = (BYTE)pcan_tool_dlc(i);
        st = CAN_WriteFD(h, &m);
    } else {
        TPCANMsg c;
        memset(&c, 0, sizeof(c));
        c.ID = m.ID;
        c.MSGTYPE = m.MSGTYPE & ~(PCAN_MESSAGE_FD | PCAN_MESSAGE_BRS);
        c.LEN = (BYTE)(i > 8 ? 8 : i);
        memcpy(c.DATA, m.DATA, c.LEN);
        m.DLC = c.LEN;
        st = CAN_Write(h, &c);
    }
    if (st != PCAN_ERROR_OK) {
        CAN_GetErrorText(st, 0x09, err);
        fprintf(stderr, "write failed: %s (0x%x)\n", err, st);
        CAN_Uninitialize(h);
        return 1;
    }
    printf("sent id=0x%X dlc=%u on handle 0x%02X\n", m.ID, m.DLC, h);
    /* Let the transmit URB reach the adapter before we close the channel. */
    usleep(20000);
    CAN_Uninitialize(h);
    return 0;
}
