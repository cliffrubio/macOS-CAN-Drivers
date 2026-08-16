/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "pcan_priv.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* PEAK documents blank space as permitted around the separators, and its own
 * example strings contain it, so each token is trimmed before matching. A
 * value must also be a plain non-negative number: strtoul would otherwise
 * accept "-1" and hand back 0xFFFFFFFF. */
static int kv_u32(const char *s, const char *key, uint32_t *out)
{
    char pat[64];
    const char *p;
    size_t klen;

    snprintf(pat, sizeof(pat), "%s=", key);
    klen = strlen(pat);
    p = s;
    while (p && *p) {
        const char *tok = p;
        while (*tok && isspace((unsigned char)*tok))
            tok++;
        if (strncasecmp(tok, pat, klen) == 0) {
            const char *v = tok + klen;
            char *endp;
            unsigned long val;
            while (*v && isspace((unsigned char)*v))
                v++;
            if (!isdigit((unsigned char)*v))
                return 0;
            val = strtoul(v, &endp, 0);
            if (endp == v || val > 0xFFFFFFFFul)
                return 0;
            *out = (uint32_t)val;
            return 1;
        }
        p = strchr(p, ',');
        if (p)
            p++;
    }
    return 0;
}

/* Ranges the uCAN registers can actually encode. Values beyond these would be
 * silently truncated by the register packing and the channel would come up at
 * a bit rate nobody asked for, reported as success. */
static int timing_in_range(uint32_t brp, uint32_t tseg1, uint32_t tseg2,
                           uint32_t sjw)
{
    return brp >= 1 && brp <= 1024 &&
           tseg1 >= 1 && tseg1 <= 256 &&
           tseg2 >= 1 && tseg2 <= 128 &&
           sjw >= 1 && sjw <= 128 && sjw <= tseg2;
}

int pcan_parse_bitrate_fd(const char *s, struct pcan_bittiming *bt)
{
    uint32_t clock = 0, clock_mhz = 0;
    uint32_t nom_brp = 0, nom_tseg1 = 0, nom_tseg2 = 0, nom_sjw = 1;
    uint32_t data_brp = 0, data_tseg1 = 0, data_tseg2 = 0, data_sjw = 1;

    if (!s || !bt)
        return -1;
    memset(bt, 0, sizeof(*bt));

    kv_u32(s, "f_clock", &clock);
    kv_u32(s, "f_clock_mhz", &clock_mhz);
    if (!clock && clock_mhz)
        clock = clock_mhz * 1000000u;
    if (!clock)
        clock = 80000000u;
    /* Only these domains exist in the hardware. Anything else used to be
     * quietly rounded to 80 MHz, which halved or doubled the resulting bit
     * rate while still reporting success. */
    if (clock != 80000000u && clock != 60000000u && clock != 40000000u &&
        clock != 30000000u && clock != 24000000u && clock != 20000000u)
        return -1;

    if (!kv_u32(s, "nom_brp", &nom_brp) ||
        !kv_u32(s, "nom_tseg1", &nom_tseg1) ||
        !kv_u32(s, "nom_tseg2", &nom_tseg2))
        return -1;
    kv_u32(s, "nom_sjw", &nom_sjw);
    if (!timing_in_range(nom_brp, nom_tseg1, nom_tseg2, nom_sjw))
        return -1;

    bt->clock_hz = clock;
    bt->brp = (uint16_t)nom_brp;
    bt->tseg1 = (uint16_t)nom_tseg1;
    bt->tseg2 = (uint16_t)nom_tseg2;
    bt->sjw = (uint16_t)nom_sjw;

    if (kv_u32(s, "data_brp", &data_brp) &&
        kv_u32(s, "data_tseg1", &data_tseg1) &&
        kv_u32(s, "data_tseg2", &data_tseg2)) {
        kv_u32(s, "data_sjw", &data_sjw);
        if (!timing_in_range(data_brp, data_tseg1, data_tseg2, data_sjw))
            return -1;
        bt->fd = 1;
        bt->dbrp = (uint16_t)data_brp;
        bt->dtseg1 = (uint16_t)data_tseg1;
        bt->dtseg2 = (uint16_t)data_tseg2;
        bt->dsjw = (uint16_t)data_sjw;
    }
    return 0;
}

/* SJA1000 BTR0BTR1 at 16 MHz -> uCAN timings at 80 MHz (same bit time). */
int pcan_btr_from_sja1000(uint16_t btr0btr1, struct pcan_bittiming *bt)
{
    uint8_t btr0 = (uint8_t)(btr0btr1 >> 8);
    uint8_t btr1 = (uint8_t)(btr0btr1 & 0xff);
    uint16_t brp = (uint16_t)((btr0 & 0x3f) + 1);
    uint16_t sjw = (uint16_t)(((btr0 >> 6) & 0x03) + 1);
    uint16_t tseg1 = (uint16_t)((btr1 & 0x0f) + 1);
    uint16_t tseg2 = (uint16_t)(((btr1 >> 4) & 0x07) + 1);

    memset(bt, 0, sizeof(*bt));
    bt->clock_hz = 80000000u;
    bt->brp = (uint16_t)(brp * 10u); /* 80 MHz / (16 MHz / 2) */
    bt->tseg1 = tseg1;
    bt->tseg2 = tseg2;
    bt->sjw = sjw;
    bt->tsam = (btr1 >> 7) & 1;
    bt->fd = 0;
    return 0;
}
