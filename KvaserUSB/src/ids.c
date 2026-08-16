/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
 *
 * Supported adapters. Product IDs, channel counts and the protocol family
 * each device speaks, taken from linuxcan leaf/, usbcanII/ and mhydra/.
 * Channel counts here are a fallback: the real count is read from the card
 * once it has been opened.
 */
#include "kvaser_priv.h"

/* Vendor is always 0x0BFD (Kvaser AB). */
static const struct kv_product k_products[] = {
    /* The USBcan II generation speaks the "helios" command set, not "filo":
     * the records share command numbers but not layouts. See helios.c. */
    { 2,   KV_FAMILY_HELIOS, 2, 0, "Kvaser USBcan" },
    { 3,   KV_FAMILY_HELIOS, 2, 0, "Kvaser VCI-2" },
    { 4,   KV_FAMILY_HELIOS, 2, 0, "Kvaser USBcan II" },
    { 5,   KV_FAMILY_HELIOS, 2, 0, "Kvaser Memorator" },

    /* Leaf / Filo */
    { 10,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf prototype" },
    { 11,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Light" },
    { 12,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Professional HS" },
    { 14,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf SemiPro HS" },
    { 15,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Professional LS" },
    { 16,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Professional SWC" },
    { 18,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf SemiPro LS" },
    { 19,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf SemiPro SWC" },
    { 22,  KV_FAMILY_LEAF,  2, 0, "Kvaser Memorator II prototype" },
    { 23,  KV_FAMILY_LEAF,  2, 0, "Kvaser Memorator II HS/HS" },
    { 24,  KV_FAMILY_LEAF,  2, 0, "Kvaser USBcan Professional HS/HS" },
    { 25,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Light GI" },
    { 26,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Professional HS OBD-II" },
    { 27,  KV_FAMILY_LEAF,  2, 0, "Kvaser Memorator Professional HS/LS" },
    { 28,  KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Light" },
    { 29,  KV_FAMILY_LEAF,  1, 0, "Kvaser BlackBird SemiPro" },
    { 32,  KV_FAMILY_LEAF,  1, 0, "Kvaser Memorator R SemiPro" },
    { 34,  KV_FAMILY_LEAF,  1, 0, "Kvaser OEM Mercury" },
    { 35,  KV_FAMILY_LEAF,  1, 0, "Kvaser OEM Leaf" },
    { 38,  KV_FAMILY_LEAF,  1, 0, "Key Driving Interface HS" },
    { 39,  KV_FAMILY_LEAF,  2, 0, "Kvaser USBcan R" },
    { 288, KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Light v2" },
    { 289, KV_FAMILY_LEAF,  1, 0, "Kvaser Mini PCI Express HS" },
    { 290, KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Light HS v2 OEM" },
    { 291, KV_FAMILY_LEAF,  2, 0, "Kvaser USBcan Light 2xHS" },
    { 292, KV_FAMILY_LEAF,  2, 0, "Kvaser Mini PCI Express 2xHS" },
    { 294, KV_FAMILY_LEAF,  2, 0, "Kvaser USBcan R v2" },
    { 295, KV_FAMILY_LEAF,  1, 0, "Kvaser Leaf Light R v2" },
    { 296, KV_FAMILY_LEAF,  1, 0, "Kvaser OEM ATI Leaf Light HS v2" },

    /* Hydra / mhydra (CAN FD) */
    { 256, KV_FAMILY_HYDRA, 1, 1, "Kvaser Eagle" },
    { 258, KV_FAMILY_HYDRA, 1, 1, "Kvaser BlackBird v2" },
    { 260, KV_FAMILY_HYDRA, 5, 1, "Kvaser Memorator Pro 5xHS" },
    { 261, KV_FAMILY_HYDRA, 5, 1, "Kvaser USBcan Pro 5xHS" },
    { 262, KV_FAMILY_HYDRA, 4, 0, "Kvaser USBcan Light 4xHS" },
    { 263, KV_FAMILY_HYDRA, 1, 1, "Kvaser Leaf Pro HS v2" },
    { 264, KV_FAMILY_HYDRA, 2, 1, "Kvaser USBcan Pro 2xHS v2" },
    { 265, KV_FAMILY_HYDRA, 2, 1, "Kvaser Memorator 2xHS v2" },
    { 266, KV_FAMILY_HYDRA, 2, 1, "Kvaser Memorator Pro 2xHS v2" },
    { 267, KV_FAMILY_HYDRA, 2, 1, "Kvaser Hybrid 2xCAN/LIN" },
    { 268, KV_FAMILY_HYDRA, 2, 1, "ATI USBcan Pro 2xHS v2" },
    { 269, KV_FAMILY_HYDRA, 2, 1, "ATI Memorator Pro 2xHS v2" },
    { 270, KV_FAMILY_HYDRA, 2, 1, "Kvaser Hybrid Pro 2xCAN/LIN" },
    { 271, KV_FAMILY_HYDRA, 1, 1, "Kvaser BlackBird Pro HS v2" },
    { 272, KV_FAMILY_HYDRA, 1, 1, "Kvaser Memorator Light HS v2" },
    { 273, KV_FAMILY_HYDRA, 1, 1, "Kvaser U100" },
    { 274, KV_FAMILY_HYDRA, 1, 1, "Kvaser U100P" },
    { 275, KV_FAMILY_HYDRA, 1, 1, "Kvaser U100S" },
    { 276, KV_FAMILY_HYDRA, 4, 1, "Kvaser USBcan Pro 4xHS" },
    { 277, KV_FAMILY_HYDRA, 1, 1, "Kvaser Hybrid CAN/LIN" },
    { 278, KV_FAMILY_HYDRA, 1, 1, "Kvaser Hybrid Pro CAN/LIN" },
    { 279, KV_FAMILY_HYDRA, 1, 1, "Kvaser Leaf v3" },
    { 280, KV_FAMILY_HYDRA, 4, 1, "Kvaser USBcan Pro 4xCAN Silent" },
    { 281, KV_FAMILY_HYDRA, 1, 1, "VINING 800" },
    { 282, KV_FAMILY_HYDRA, 5, 1, "Kvaser USBcan Pro 5xCAN" },
    { 283, KV_FAMILY_HYDRA, 1, 1, "Kvaser Mini PCIe 1xCAN" },
    { 284, KV_FAMILY_HYDRA, 1, 1, "Easyscan CAN" },
    { 285, KV_FAMILY_HYDRA, 1, 1, "CAN Logger" },
};

const struct kv_product *kv_lookup_pid(uint16_t pid)
{
    unsigned i;
    for (i = 0; i < sizeof(k_products) / sizeof(k_products[0]); i++)
        if (k_products[i].pid == pid)
            return &k_products[i];
    return NULL;
}
