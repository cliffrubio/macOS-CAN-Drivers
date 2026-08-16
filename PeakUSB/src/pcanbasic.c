/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 * PCAN-Basic compatible API for macOS (libPeakUSB.dylib)
 */
#include "pcan_priv.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define EXPORT __attribute__((visibility("default")))

/* The PCAN-Basic API is documented as callable from several threads. Every
 * entry point below therefore runs under one lock: the device list, the
 * handle map and each channel's state are all reachable from any handle, and
 * pcan_usb_refresh() may free and rebuild devices outright. The lock is held
 * across blocking USB I/O, which serialises calls but keeps them correct;
 * receive is unaffected because CAN_Read only drains an already-filled queue.
 * Ordering is api_lock -> g_lock; nothing ever takes them the other way. */
static pthread_mutex_t api_lock = PTHREAD_MUTEX_INITIALIZER;


static int is_usb_handle(TPCANHandle h)
{
    return (h >= PCAN_USBBUS1 && h <= PCAN_USBBUS8);
}

static TPCANStatus need_ch(TPCANHandle Channel, struct pcan_channel **out, int must_init)
{
    struct pcan_channel *ch;
    if (!is_usb_handle(Channel) && Channel != PCAN_NONEBUS)
        return PCAN_ERROR_ILLCLIENT;
    ch = pcan_usb_channel(Channel);
    if (!ch)
        return PCAN_ERROR_ILLHW;
    if (must_init && !ch->initialized)
        return PCAN_ERROR_INITIALIZE;
    *out = ch;
    return PCAN_ERROR_OK;
}

static TPCANStatus do_init(TPCANHandle Channel, int fd_mode)
{
    struct pcan_channel *ch;
    TPCANStatus st;

    st = need_ch(Channel, &ch, 0);
    if (st)
        return st;
    if (ch->initialized)
        return PCAN_ERROR_NETINUSE;
    if (pcan_usb_open_channel(ch) != 0)
        return PCAN_ERROR_HWINUSE;
    ch->fd_mode = fd_mode;
    if (pcan_usb_configure(ch) != 0) {
        pcan_usb_close_channel(ch);
        return PCAN_ERROR_ILLPARAMVAL;
    }
    if (pcan_usb_bus_on(ch) != 0) {
        pcan_usb_close_channel(ch);
        return PCAN_ERROR_RESOURCE;
    }
    ch->initialized = 1;
    pcan_log("initialized handle 0x%02x CAN%u %s", Channel, ch->can_idx + 1,
             fd_mode ? "FD" : "classic");
    return PCAN_ERROR_OK;
}

static TPCANStatus CAN_Initialize_locked(TPCANHandle Channel, TPCANBaudrate Btr0Btr1,
                                  TPCANType HwType, DWORD IOPort, WORD Interrupt)
{
    struct pcan_channel *ch;
    TPCANStatus st;
    (void)HwType;
    (void)IOPort;
    (void)Interrupt;

    pcan_usb_refresh();
    st = need_ch(Channel, &ch, 0);
    if (st)
        return st;
    /* Reject a busy channel before touching its stored configuration, or a
     * failed call leaves the channel reporting a bit rate it is not using. */
    if (ch->initialized)
        return PCAN_ERROR_NETINUSE;
    if (pcan_btr_from_sja1000(Btr0Btr1, &ch->bt) != 0)
        return PCAN_ERROR_ILLPARAMVAL;
    ch->btr0btr1 = Btr0Btr1;
    snprintf(ch->bitrate_fd, sizeof(ch->bitrate_fd),
             "f_clock=80000000,nom_brp=%u,nom_tseg1=%u,nom_tseg2=%u,nom_sjw=%u",
             ch->bt.brp, ch->bt.tseg1, ch->bt.tseg2, ch->bt.sjw);
    return do_init(Channel, 0);
}

static TPCANStatus CAN_InitializeFD_locked(TPCANHandle Channel, TPCANBitrateFD BitrateFD)
{
    struct pcan_channel *ch;
    TPCANStatus st;

    pcan_usb_refresh();
    st = need_ch(Channel, &ch, 0);
    if (st)
        return st;
    if (!BitrateFD)
        return PCAN_ERROR_ILLPARAMVAL;
    if (ch->dev->family != PCAN_FAM_FD)
        return PCAN_ERROR_ILLOPERATION;
    if (ch->initialized)
        return PCAN_ERROR_NETINUSE;
    if (pcan_parse_bitrate_fd(BitrateFD, &ch->bt) != 0)
        return PCAN_ERROR_ILLPARAMVAL;
    snprintf(ch->bitrate_fd, sizeof(ch->bitrate_fd), "%s", BitrateFD);
    return do_init(Channel, 1);
}

static TPCANStatus CAN_Uninitialize_locked(TPCANHandle Channel)
{
    int i;
    if (Channel == PCAN_NONEBUS) {
        pcan_usb_refresh();
        for (i = 0; i < pcan_usb_handle_count(); i++) {
            struct pcan_handle *m = pcan_usb_lookup(pcan_usb_handle_at(i));
            if (m && m->dev)
                pcan_usb_close_channel(&m->dev->ch[m->can_idx]);
        }
        return PCAN_ERROR_OK;
    }
    {
        struct pcan_channel *ch;
        TPCANStatus st = need_ch(Channel, &ch, 1);
        if (st)
            return st;
        pcan_usb_close_channel(ch);
        return PCAN_ERROR_OK;
    }
}

static TPCANStatus CAN_Reset_locked(TPCANHandle Channel)
{
    struct pcan_channel *ch;
    TPCANStatus st = need_ch(Channel, &ch, 1);
    if (st)
        return st;
    pcan_usb_reset_queue(ch);
    if (ch->status & PCAN_ERROR_BUSOFF) {
        pcan_usb_bus_off(ch);
        pcan_usb_bus_on(ch);
    }
    return PCAN_ERROR_OK | (ch->status & PCAN_ERROR_ANYBUSERR);
}

static TPCANStatus CAN_GetStatus_locked(TPCANHandle Channel)
{
    struct pcan_channel *ch;
    TPCANStatus st = need_ch(Channel, &ch, 1);
    if (st)
        return st;
    if (ch->busoff_autoreset && (ch->status & PCAN_ERROR_BUSOFF) && ch->initialized) {
        pcan_usb_bus_off(ch);
        pcan_usb_bus_on(ch);
    }
    return PCAN_ERROR_OK | (ch->status & PCAN_ERROR_ANYBUSERR) |
           (ch->status & PCAN_ERROR_QOVERRUN);
}

static TPCANStatus pop_msg(struct pcan_channel *ch, TPCANMsgFD *fd,
                           TPCANTimestampFD *tsfd)
{
    uint64_t ts = 0;
    if (!pcan_queue_pop(&ch->q, fd, &ts))
        return PCAN_ERROR_QRCVEMPTY | (ch->status & PCAN_ERROR_ANYBUSERR);
    if (tsfd)
        *tsfd = ts;
    pcan_event_clear_if_empty(ch);
    return PCAN_ERROR_OK | (ch->status & PCAN_ERROR_ANYBUSERR);
}

static TPCANStatus CAN_Read_locked(TPCANHandle Channel, TPCANMsg *MessageBuffer,
                            TPCANTimestamp *TimestampBuffer)
{
    struct pcan_channel *ch;
    TPCANMsgFD fd;
    TPCANTimestampFD ts = 0;
    TPCANStatus st;
    uint8_t len;

    st = need_ch(Channel, &ch, 1);
    if (st)
        return st;
    if (!MessageBuffer)
        return PCAN_ERROR_ILLPARAMVAL;
    if (ch->fd_mode)
        return PCAN_ERROR_ILLMODE;
    st = pop_msg(ch, &fd, &ts);
    if (st & PCAN_ERROR_QRCVEMPTY)
        return st;
    MessageBuffer->ID = fd.ID;
    MessageBuffer->MSGTYPE = (TPCANMessageType)(fd.MSGTYPE & ~PCAN_MESSAGE_FD);
    MessageBuffer->LEN = fd.DLC > 8 ? 8 : fd.DLC;
    len = MessageBuffer->LEN;
    memcpy(MessageBuffer->DATA, fd.DATA, len);
    if (TimestampBuffer) {
        TimestampBuffer->millis = (DWORD)(ts / 1000ull);
        TimestampBuffer->millis_overflow = (WORD)((ts / 1000ull) >> 32);
        TimestampBuffer->micros = (WORD)(ts % 1000ull);
    }
    return st;
}

static TPCANStatus CAN_ReadFD_locked(TPCANHandle Channel, TPCANMsgFD *MessageBuffer,
                              TPCANTimestampFD *TimestampBuffer)
{
    struct pcan_channel *ch;
    TPCANStatus st = need_ch(Channel, &ch, 1);
    if (st)
        return st;
    if (!MessageBuffer)
        return PCAN_ERROR_ILLPARAMVAL;
    if (!ch->fd_mode)
        return PCAN_ERROR_ILLMODE;
    return pop_msg(ch, MessageBuffer, TimestampBuffer);
}

static TPCANStatus CAN_Write_locked(TPCANHandle Channel, TPCANMsg *MessageBuffer)
{
    struct pcan_channel *ch;
    TPCANMsgFD fd;
    TPCANStatus st = need_ch(Channel, &ch, 1);
    if (st)
        return st;
    if (!MessageBuffer)
        return PCAN_ERROR_ILLPARAMVAL;
    if (ch->fd_mode)
        return PCAN_ERROR_ILLMODE;
    if (ch->status & PCAN_ERROR_BUSOFF)
        return PCAN_ERROR_BUSOFF;
    memset(&fd, 0, sizeof(fd));
    fd.ID = MessageBuffer->ID;
    fd.MSGTYPE = MessageBuffer->MSGTYPE;
    fd.DLC = MessageBuffer->LEN > 8 ? 8 : MessageBuffer->LEN;
    memcpy(fd.DATA, MessageBuffer->DATA, fd.DLC);
    if (pcan_usb_tx_throttle(ch, &fd))
        return PCAN_ERROR_XMTFULL | (ch->status & PCAN_ERROR_ANYBUSERR);
    if (pcan_usb_write_msg(ch, &fd) != 0)
        return PCAN_ERROR_RESOURCE | (ch->status & PCAN_ERROR_ANYBUSERR);
    return PCAN_ERROR_OK | (ch->status & PCAN_ERROR_ANYBUSERR);
}

static TPCANStatus CAN_WriteFD_locked(TPCANHandle Channel, TPCANMsgFD *MessageBuffer)
{
    struct pcan_channel *ch;
    TPCANStatus st = need_ch(Channel, &ch, 1);
    if (st)
        return st;
    if (!MessageBuffer)
        return PCAN_ERROR_ILLPARAMVAL;
    if (!ch->fd_mode)
        return PCAN_ERROR_ILLMODE;
    if (ch->status & PCAN_ERROR_BUSOFF)
        return PCAN_ERROR_BUSOFF;
    if (pcan_usb_tx_throttle(ch, MessageBuffer))
        return PCAN_ERROR_XMTFULL | (ch->status & PCAN_ERROR_ANYBUSERR);
    if (pcan_usb_write_msg(ch, MessageBuffer) != 0)
        return PCAN_ERROR_RESOURCE | (ch->status & PCAN_ERROR_ANYBUSERR);
    return PCAN_ERROR_OK | (ch->status & PCAN_ERROR_ANYBUSERR);
}

static TPCANStatus CAN_FilterMessages_locked(TPCANHandle Channel, DWORD FromID,
                                      DWORD ToID, TPCANMode Mode)
{
    struct pcan_channel *ch;
    TPCANStatus st = need_ch(Channel, &ch, 1);
    DWORD lo = FromID, hi = ToID;
    if (st)
        return st;
    if (lo > hi) {
        DWORD tmp = lo;
        lo = hi;
        hi = tmp;
    }
    if (ch->filter_mode != PCAN_FILTER_CUSTOM) {
        ch->filter_mode = PCAN_FILTER_CUSTOM;
        ch->filter_count = 0;
    }
    if (ch->filter_count >= PEAKUSB_MAX_FILTERS)
        return PCAN_ERROR_RESOURCE | (ch->status & PCAN_ERROR_ANYBUSERR);
    ch->filters[ch->filter_count].from = lo;
    ch->filters[ch->filter_count].to = hi;
    ch->filters[ch->filter_count].ext = (Mode & PCAN_MESSAGE_EXTENDED) ? 1 : 0;
    ch->filter_count++;
    return PCAN_ERROR_OK;
}

static void fill_info(struct pcan_handle *m, TPCANChannelInformation *info)
{
    struct pcan_channel *ch = &m->dev->ch[m->can_idx];
    memset(info, 0, sizeof(*info));
    info->channel_handle = m->handle;
    info->device_type = PCAN_USB;
    info->controller_number = (BYTE)m->can_idx;
    info->device_features = (m->dev->family == PCAN_FAM_FD) ? FEATURE_FD_CAPABLE : 0;
    snprintf(info->device_name, sizeof(info->device_name), "%s", m->dev->name);
    info->device_id = ch->device_id;
    info->channel_condition = ch->initialized ? PCAN_CHANNEL_OCCUPIED
                                              : PCAN_CHANNEL_AVAILABLE;
}

static TPCANStatus CAN_GetValue_locked(TPCANHandle Channel, TPCANParameter Parameter,
                                void *Buffer, DWORD BufferLength)
{
    struct pcan_channel *ch = NULL;
    struct pcan_handle *m;
    DWORD n;
    int i;

    if (!Buffer || BufferLength == 0)
        return PCAN_ERROR_ILLPARAMVAL;

    pcan_usb_refresh();

    switch (Parameter) {
    case PCAN_API_VERSION:
        snprintf((char *)Buffer, BufferLength, "%s", PEAKUSB_API_VERSION);
        return PCAN_ERROR_OK;

    /* Queried on PCAN_NONEBUS this reports the version of this library.
     * Asked about a channel it falls through to the per-channel switch below,
     * which reports that device's firmware and hardware revision. */
    case PCAN_CHANNEL_VERSION:
        if (Channel == PCAN_NONEBUS) {
            snprintf((char *)Buffer, BufferLength, "%s", PEAKUSB_LIB_VERSION);
            return PCAN_ERROR_OK;
        }
        break;

    case PCAN_ATTACHED_CHANNELS_COUNT:
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        n = (DWORD)pcan_usb_handle_count();
        memcpy(Buffer, &n, sizeof(n));
        return PCAN_ERROR_OK;

    case PCAN_ATTACHED_CHANNELS: {
        TPCANChannelInformation *tab = Buffer;
        DWORD maxn = BufferLength / sizeof(TPCANChannelInformation);
        DWORD count = (DWORD)pcan_usb_handle_count();
        if (maxn < count)
            return PCAN_ERROR_ILLPARAMVAL;
        for (i = 0; i < (int)count; i++) {
            m = pcan_usb_lookup(pcan_usb_handle_at(i));
            if (m)
                fill_info(m, &tab[i]);
        }
        return PCAN_ERROR_OK;
    }
    default:
        break;
    }

    if (Channel == PCAN_NONEBUS)
        return PCAN_ERROR_ILLHANDLE;

    m = pcan_usb_lookup(Channel);
    if (Parameter == PCAN_CHANNEL_CONDITION) {
        DWORD v = PCAN_CHANNEL_UNAVAILABLE;
        if (m && m->present)
            v = m->dev->ch[m->can_idx].initialized ? PCAN_CHANNEL_OCCUPIED
                                                   : PCAN_CHANNEL_AVAILABLE;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    if (!m)
        return PCAN_ERROR_ILLHW;

    ch = &m->dev->ch[m->can_idx];

    switch (Parameter) {
    case PCAN_DEVICE_ID: {
        DWORD v = ch->device_id;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_RECEIVE_EVENT: {
        int fd = ch->ev_r;
        if (!ch->initialized)
            return PCAN_ERROR_INITIALIZE;
        if (BufferLength >= sizeof(int))
            memcpy(Buffer, &fd, sizeof(fd));
        else
            return PCAN_ERROR_ILLPARAMVAL;
        return PCAN_ERROR_OK;
    }
    case PCAN_CHANNEL_VERSION:
        snprintf((char *)Buffer, BufferLength,
                 "%s, Firmware %u.%u.%u, Hardware %u",
                 m->dev->name, m->dev->fw[0], m->dev->fw[1], m->dev->fw[2],
                 m->dev->hw_ver);
        return PCAN_ERROR_OK;
    case PCAN_HARDWARE_NAME:
        snprintf((char *)Buffer, BufferLength, "%s", m->dev->name);
        return PCAN_ERROR_OK;
    case PCAN_FIRMWARE_VERSION:
        snprintf((char *)Buffer, BufferLength, "%u.%u.%u",
                 m->dev->fw[0], m->dev->fw[1], m->dev->fw[2]);
        return PCAN_ERROR_OK;
    case PCAN_CONTROLLER_NUMBER: {
        DWORD v = (DWORD)ch->can_idx;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_CHANNEL_FEATURES: {
        DWORD v = (m->dev->family == PCAN_FAM_FD) ? FEATURE_FD_CAPABLE : 0;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_LISTEN_ONLY: {
        DWORD v = ch->listen_only ? PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_RECEIVE_STATUS: {
        DWORD v = ch->receive_on ? PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_ALLOW_STATUS_FRAMES: {
        DWORD v = ch->allow_status ? PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_ALLOW_RTR_FRAMES: {
        DWORD v = ch->allow_rtr ? PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_ALLOW_ERROR_FRAMES: {
        DWORD v = ch->allow_error ? PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_BITRATE_INFO: {
        WORD v = ch->btr0btr1;
        if (BufferLength < sizeof(WORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_BITRATE_INFO_FD:
        snprintf((char *)Buffer, BufferLength, "%s", ch->bitrate_fd);
        return PCAN_ERROR_OK;
    case PCAN_ALLOW_ECHO_FRAMES: {
        DWORD v = ch->allow_echo ? PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_INTERFRAME_DELAY: {
        DWORD v = ch->interframe_us;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_BUSSPEED_NOMINAL: {
        DWORD v = pcan_usb_bitrate(ch, 0);
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_BUSSPEED_FD: {
        DWORD v;
        if (!ch->fd_mode)
            return PCAN_ERROR_ILLOPERATION;
        v = pcan_usb_bitrate(ch, 1);
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_IO_ANALOG_VALUE: {
        /* Analog input, PCAN-USB Chip only. The firmware answers on the
         * command endpoint, which this driver does not poll for replies, so
         * the value is unavailable rather than wrong. */
        if (m->dev->pid != PCAN_USBCHIP_PRODUCT_ID)
            return PCAN_ERROR_ILLOPERATION;
        return PCAN_ERROR_ILLPARAMTYPE;
    }
    case PCAN_BUSSPEED_XL: {
        DWORD v;
        if (ch->dev->family != PCAN_FAM_XL || !ch->xl_mode)
            return PCAN_ERROR_ILLOPERATION;
        v = pcan_usb_bitrate(ch, 1);
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_BITRATE_INFO_XL:
        if (ch->dev->family != PCAN_FAM_XL || !ch->xl_mode)
            return PCAN_ERROR_ILLOPERATION;
        snprintf((char *)Buffer, BufferLength, "%s", ch->bitrate_fd);
        return PCAN_ERROR_OK;
    case PCAN_DEVICE_GUID:
        /* PEAK has no GUID on the wire either; it derives one from the
         * device. Built here from the serial number and channel so the value
         * is stable for a given adapter, and distinct between channels. */
        if (!m->dev->serial)
            return PCAN_ERROR_ILLPARAMTYPE;
        snprintf((char *)Buffer, BufferLength,
                 "%08X-%04X-%04X-%04X-%08X%04X",
                 m->dev->serial,
                 (unsigned)(m->dev->pid & 0xFFFF),
                 (unsigned)(m->dev->hw_type & 0xFFFF),
                 (unsigned)(ch->can_idx & 0xFFFF),
                 m->dev->serial,
                 (unsigned)(m->dev->hw_ver & 0xFFFF));
        return PCAN_ERROR_OK;
    case PCAN_BITRATE_INFO_CC:
        /* Classic bit rate as a string, in the same form CAN_Initialize takes. */
        snprintf((char *)Buffer, BufferLength, "%u", pcan_usb_bitrate(ch, 0));
        return PCAN_ERROR_OK;
    case PCAN_DEVICE_PART_NUMBER:
        snprintf((char *)Buffer, BufferLength, "%s", m->dev->name);
        return PCAN_ERROR_OK;
    case PCAN_HARD_RESET_STATUS: {
        /* CAN_Reset always resets the controller, never only the queues. */
        DWORD v = PCAN_PARAMETER_ON;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_BITRATE_ADAPTING:
    case PCAN_5VOLTS_POWER: {
        /* Neither is supported by this driver; report them as off rather
         * than refusing, so a caller probing capabilities gets an answer. */
        DWORD v = PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_LOG_STATUS:
    case PCAN_TRACE_STATUS: {
        /* This driver has no trace-file machinery; PEAKUSB_DEBUG traces to
         * stderr instead. Report the feature as off rather than refusing, so
         * a caller probing for it gets a definite answer. */
        DWORD val = PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &val, sizeof(val));
        return PCAN_ERROR_OK;
    }
    case PCAN_LOG_LOCATION:
    case PCAN_TRACE_LOCATION:
        snprintf((char *)Buffer, BufferLength, "%s", "");
        return PCAN_ERROR_OK;
    case PCAN_MESSAGE_FILTER: {
        DWORD v = ch->filter_mode;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    case PCAN_BUSOFF_AUTORESET: {
        DWORD v = ch->busoff_autoreset ? PCAN_PARAMETER_ON : PCAN_PARAMETER_OFF;
        if (BufferLength < sizeof(DWORD))
            return PCAN_ERROR_ILLPARAMVAL;
        memcpy(Buffer, &v, sizeof(v));
        return PCAN_ERROR_OK;
    }
    default:
        return PCAN_ERROR_ILLPARAMTYPE;
    }
}

static TPCANStatus CAN_SetValue_locked(TPCANHandle Channel, TPCANParameter Parameter,
                                void *Buffer, DWORD BufferLength)
{
    struct pcan_channel *ch;
    TPCANStatus st;
    DWORD v = 0;

    /* A missing or empty buffer used to be accepted and applied as value 0,
     * which silently turned features off instead of reporting the mistake. */
    if (!Buffer || BufferLength == 0)
        return PCAN_ERROR_ILLPARAMVAL;

    st = need_ch(Channel, &ch, 0);
    if (st)
        return st;
    if (Buffer && BufferLength >= 1) {
        if (BufferLength >= 4)
            memcpy(&v, Buffer, 4);
        else
            v = *(uint8_t *)Buffer;
    }

    switch (Parameter) {
    case PCAN_LISTEN_ONLY: {
        int want = v ? 1 : 0;
        if (ch->listen_only == want)
            return PCAN_ERROR_OK;
        ch->listen_only = want;
        if (ch->initialized && ch->bus_on) {
            pcan_usb_bus_off(ch);
            pcan_usb_bus_on(ch);
        }
        return PCAN_ERROR_OK;
    }
    case PCAN_RECEIVE_STATUS:
        ch->receive_on = v ? 1 : 0;
        return PCAN_ERROR_OK;
    case PCAN_ALLOW_STATUS_FRAMES:
        ch->allow_status = v ? 1 : 0;
        return PCAN_ERROR_OK;
    case PCAN_ALLOW_RTR_FRAMES:
        ch->allow_rtr = v ? 1 : 0;
        return PCAN_ERROR_OK;
    case PCAN_ALLOW_ERROR_FRAMES:
        ch->allow_error = v ? 1 : 0;
        return PCAN_ERROR_OK;
    case PCAN_BUSOFF_AUTORESET:
        ch->busoff_autoreset = v ? 1 : 0;
        return PCAN_ERROR_OK;
    case PCAN_ALLOW_ECHO_FRAMES:
        ch->allow_echo = v ? 1 : 0;
        return PCAN_ERROR_OK;
    case PCAN_INTERFRAME_DELAY:
        /* Microseconds. Bounded so a stray value cannot stall transmit
         * indefinitely; one second between frames is already absurd. */
        if (v > 1000000u)
            return PCAN_ERROR_ILLPARAMVAL;
        ch->interframe_us = v;
        return PCAN_ERROR_OK;
    case PCAN_LOG_STATUS:
    case PCAN_TRACE_STATUS:
    case PCAN_LOG_CONFIGURE:
    case PCAN_TRACE_CONFIGURE:
    case PCAN_TRACE_SIZE:
        /* Turning tracing off is the state this driver is always in, so
         * accept that and refuse anything that would promise a trace file. */
        return v ? PCAN_ERROR_ILLOPERATION : PCAN_ERROR_OK;
    case PCAN_LOG_TEXT:
        /* A caller writing to the log gets it on stderr under PEAKUSB_DEBUG. */
        pcan_log("%s", Buffer ? (const char *)Buffer : "");
        return PCAN_ERROR_OK;
    case PCAN_IO_DIGITAL_CONFIGURATION:
        return pcan_usb_io_write(ch, CANFD_USB_CMD_DPIN_CFG_SET, v) == 0
               ? PCAN_ERROR_OK : PCAN_ERROR_ILLOPERATION;
    case PCAN_IO_DIGITAL_VALUE:
        return pcan_usb_io_write(ch, CANFD_USB_CMD_DPIN_VAL_SET, v) == 0
               ? PCAN_ERROR_OK : PCAN_ERROR_ILLOPERATION;
    case PCAN_IO_DIGITAL_SET:
        return pcan_usb_io_write(ch, CANFD_USB_CMD_DPIN_SET_HIGH, v) == 0
               ? PCAN_ERROR_OK : PCAN_ERROR_ILLOPERATION;
    case PCAN_IO_DIGITAL_CLEAR:
        return pcan_usb_io_write(ch, CANFD_USB_CMD_DPIN_SET_LOW, v) == 0
               ? PCAN_ERROR_OK : PCAN_ERROR_ILLOPERATION;
    case PCAN_CHANNEL_IDENTIFYING:
        if (pcan_usb_identify(ch, v ? 1 : 0) != 0)
            return PCAN_ERROR_ILLOPERATION;
        return PCAN_ERROR_OK;
    case PCAN_MESSAGE_FILTER:
        if (v == PCAN_FILTER_OPEN || v == PCAN_FILTER_CLOSE) {
            ch->filter_mode = (uint8_t)v;
            ch->filter_count = 0;
            return PCAN_ERROR_OK;
        }
        if (v == PCAN_FILTER_CUSTOM) {
            ch->filter_mode = PCAN_FILTER_CUSTOM;
            return PCAN_ERROR_OK;
        }
        return PCAN_ERROR_ILLPARAMVAL;
    default:
        return PCAN_ERROR_ILLPARAMTYPE;
    }
}

EXPORT TPCANStatus CAN_GetErrorText(TPCANStatus Error, WORD Language, LPSTR Buffer)
{
    const char *s = "Unknown error";
    (void)Language;
    if (!Buffer)
        return PCAN_ERROR_ILLPARAMVAL;
    /* Bus conditions are flag bits that ride along with a base code, so they
     * are stripped before the lookup. When nothing but bus bits are present
     * the status IS the bus condition, and reporting "No error" for a bus-off
     * would be actively misleading. */
    if ((Error & ~PCAN_ERROR_ANYBUSERR) == PCAN_ERROR_OK &&
        (Error & PCAN_ERROR_ANYBUSERR)) {
        if (Error & PCAN_ERROR_BUSOFF)          s = "Bus-off";
        else if (Error & PCAN_ERROR_BUSPASSIVE) s = "Bus error: error passive";
        else if (Error & PCAN_ERROR_BUSHEAVY)   s = "Bus error: error counter at the heavy limit";
        else                                    s = "Bus error: error counter at the light limit";
        snprintf(Buffer, 256, "%s", s);
        return PCAN_ERROR_OK;
    }
    switch (Error & ~PCAN_ERROR_ANYBUSERR) {
    case PCAN_ERROR_OK:          s = "No error"; break;
    case PCAN_ERROR_QRCVEMPTY:   s = "Receive queue is empty"; break;
    case PCAN_ERROR_QOVERRUN:    s = "Receive queue overrun"; break;
    case PCAN_ERROR_NODRIVER:    s = "Driver not loaded"; break;
    case PCAN_ERROR_HWINUSE:     s = "Hardware already in use"; break;
    case PCAN_ERROR_NETINUSE:    s = "Channel already initialized"; break;
    case PCAN_ERROR_ILLHW:       s = "Invalid hardware handle"; break;
    case PCAN_ERROR_ILLCLIENT:   s = "Invalid client handle"; break;
    case PCAN_ERROR_ILLPARAMTYPE:s = "Invalid parameter"; break;
    case PCAN_ERROR_ILLPARAMVAL: s = "Invalid parameter value"; break;
    case PCAN_ERROR_RESOURCE:    s = "Resource cannot be created"; break;
    case PCAN_ERROR_ILLMODE:     s = "Wrong operation mode"; break;
    case PCAN_ERROR_INITIALIZE:  s = "Channel is not initialized"; break;
    case PCAN_ERROR_BUSOFF:      s = "Bus-off"; break;
    default:
        if (Error & PCAN_ERROR_ANYBUSERR)
            s = "CAN bus error";
        break;
    }
    snprintf(Buffer, 256, "%s", s);
    return PCAN_ERROR_OK;
}

static TPCANStatus CAN_InitializeXL_locked(TPCANHandle Channel,
                                           TPCANBitrateXL BitrateXL)
{
    struct pcan_channel *ch;
    TPCANStatus st;

    pcan_usb_refresh();
    st = need_ch(Channel, &ch, 0);
    if (st)
        return st;
    if (!BitrateXL)
        return PCAN_ERROR_ILLPARAMVAL;
    /* Only the PCAN-USB XL speaks this; everything else says so plainly. */
    if (ch->dev->family != PCAN_FAM_XL)
        return PCAN_ERROR_ILLOPERATION;
    if (ch->initialized)
        return PCAN_ERROR_NETINUSE;
    if (pcan_parse_bitrate_fd(BitrateXL, &ch->bt) != 0)
        return PCAN_ERROR_ILLPARAMVAL;
    snprintf(ch->bitrate_fd, sizeof(ch->bitrate_fd), "%s", BitrateXL);

    st = need_ch(Channel, &ch, 0);
    if (st)
        return st;
    if (pcan_usb_open_channel(ch) != 0)
        return PCAN_ERROR_HWINUSE;
    ch->fd_mode = 1;
    if (pcan_xl_configure(ch) != 0) {
        pcan_usb_close_channel(ch);
        return PCAN_ERROR_ILLPARAMVAL;
    }
    if (pcan_usb_bus_on(ch) != 0) {
        pcan_usb_close_channel(ch);
        return PCAN_ERROR_RESOURCE;
    }
    ch->initialized = 1;
    pcan_log("initialized handle 0x%02x CAN%u XL", Channel, ch->can_idx + 1);
    return PCAN_ERROR_OK;
}

static TPCANStatus CAN_ReadXL_locked(TPCANHandle Channel,
                                     TPCANMsgXL *MessageBuffer,
                                     TPCANTimestampXL *TimestampBuffer)
{
    struct pcan_channel *ch;
    uint64_t ts = 0;
    TPCANStatus st = need_ch(Channel, &ch, 1);

    if (st)
        return st;
    if (!MessageBuffer)
        return PCAN_ERROR_ILLPARAMVAL;
    if (!ch->xl_mode)
        return PCAN_ERROR_ILLMODE;
    if (!pcan_xl_queue_pop(ch, MessageBuffer, &ts))
        return PCAN_ERROR_QRCVEMPTY | (ch->status & PCAN_ERROR_ANYBUSERR);
    if (TimestampBuffer)
        *TimestampBuffer = ts;
    pcan_event_clear_if_empty(ch);
    return PCAN_ERROR_OK | (ch->status & PCAN_ERROR_ANYBUSERR);
}

static TPCANStatus CAN_WriteXL_locked(TPCANHandle Channel,
                                      TPCANMsgXL *MessageBuffer)
{
    struct pcan_channel *ch;
    TPCANStatus st = need_ch(Channel, &ch, 1);

    if (st)
        return st;
    if (!MessageBuffer)
        return PCAN_ERROR_ILLPARAMVAL;
    if (!ch->xl_mode)
        return PCAN_ERROR_ILLMODE;
    if (ch->status & PCAN_ERROR_BUSOFF)
        return PCAN_ERROR_BUSOFF;
    if (pcan_xl_write_msg(ch, MessageBuffer) != 0)
        return PCAN_ERROR_RESOURCE | (ch->status & PCAN_ERROR_ANYBUSERR);
    return PCAN_ERROR_OK | (ch->status & PCAN_ERROR_ANYBUSERR);
}

static TPCANStatus CAN_LookUpChannel_locked(LPSTR Parameters, TPCANHandle *FoundChannel)
{
    DWORD want_id = 0xffffffffu;
    int want_ctrl = -1;
    int i;
    char *p;

    if (!FoundChannel)
        return PCAN_ERROR_ILLPARAMVAL;
    *FoundChannel = PCAN_NONEBUS;
    pcan_usb_refresh();
    if (Parameters && *Parameters) {
        /* Only these two criteria can be honoured. Silently ignoring the
         * others would match an arbitrary channel and report success, so an
         * unsupported key is rejected instead. */
        if (strstr(Parameters, "devicetype=") ||
            strstr(Parameters, "ipaddress=") ||
            strstr(Parameters, "deviceguid="))
            return PCAN_ERROR_ILLPARAMVAL;
        p = strstr(Parameters, "deviceid=");
        if (p)
            want_id = (DWORD)strtoul(p + 9, NULL, 0);
        p = strstr(Parameters, "controllernumber=");
        if (p)
            want_ctrl = (int)strtol(p + 17, NULL, 0);
        if (want_id == 0xffffffffu && want_ctrl < 0)
            return PCAN_ERROR_ILLPARAMVAL;
    }
    for (i = 0; i < pcan_usb_handle_count(); i++) {
        struct pcan_handle *m = pcan_usb_lookup(pcan_usb_handle_at(i));
        if (!m)
            continue;
        if (want_ctrl >= 0 && m->can_idx != want_ctrl)
            continue;
        if (want_id != 0xffffffffu && m->dev->ch[m->can_idx].device_id != want_id)
            continue;
        *FoundChannel = m->handle;
        return PCAN_ERROR_OK;
    }
    return PCAN_ERROR_ILLHW;
}

/* ------------------------------------------------------------------ */
/* Public entry points: serialise, delegate, release.                  */
/* ------------------------------------------------------------------ */

#define API_CALL(expr) \
    TPCANStatus st__; \
    pthread_mutex_lock(&api_lock); \
    st__ = (expr); \
    pthread_mutex_unlock(&api_lock); \
    return st__

EXPORT TPCANStatus CAN_Initialize(TPCANHandle Channel, TPCANBaudrate Btr0Btr1,
                                  TPCANType HwType, DWORD IOPort, WORD Interrupt)
{
    API_CALL(CAN_Initialize_locked(Channel, Btr0Btr1, HwType, IOPort, Interrupt));
}

EXPORT TPCANStatus CAN_InitializeFD(TPCANHandle Channel, TPCANBitrateFD BitrateFD)
{
    API_CALL(CAN_InitializeFD_locked(Channel, BitrateFD));
}

EXPORT TPCANStatus CAN_Uninitialize(TPCANHandle Channel)
{
    API_CALL(CAN_Uninitialize_locked(Channel));
}

EXPORT TPCANStatus CAN_Reset(TPCANHandle Channel)
{
    API_CALL(CAN_Reset_locked(Channel));
}

EXPORT TPCANStatus CAN_GetStatus(TPCANHandle Channel)
{
    API_CALL(CAN_GetStatus_locked(Channel));
}

EXPORT TPCANStatus CAN_Read(TPCANHandle Channel, TPCANMsg *MessageBuffer,
                            TPCANTimestamp *TimestampBuffer)
{
    API_CALL(CAN_Read_locked(Channel, MessageBuffer, TimestampBuffer));
}

EXPORT TPCANStatus CAN_ReadFD(TPCANHandle Channel, TPCANMsgFD *MessageBuffer,
                              TPCANTimestampFD *TimestampBuffer)
{
    API_CALL(CAN_ReadFD_locked(Channel, MessageBuffer, TimestampBuffer));
}

EXPORT TPCANStatus CAN_Write(TPCANHandle Channel, TPCANMsg *MessageBuffer)
{
    API_CALL(CAN_Write_locked(Channel, MessageBuffer));
}

EXPORT TPCANStatus CAN_WriteFD(TPCANHandle Channel, TPCANMsgFD *MessageBuffer)
{
    API_CALL(CAN_WriteFD_locked(Channel, MessageBuffer));
}

EXPORT TPCANStatus CAN_FilterMessages(TPCANHandle Channel, DWORD FromID,
                                      DWORD ToID, TPCANMode Mode)
{
    API_CALL(CAN_FilterMessages_locked(Channel, FromID, ToID, Mode));
}

EXPORT TPCANStatus CAN_GetValue(TPCANHandle Channel, TPCANParameter Parameter,
                                void *Buffer, DWORD BufferLength)
{
    API_CALL(CAN_GetValue_locked(Channel, Parameter, Buffer, BufferLength));
}

EXPORT TPCANStatus CAN_SetValue(TPCANHandle Channel, TPCANParameter Parameter,
                                void *Buffer, DWORD BufferLength)
{
    API_CALL(CAN_SetValue_locked(Channel, Parameter, Buffer, BufferLength));
}

EXPORT TPCANStatus CAN_LookUpChannel(LPSTR Parameters, TPCANHandle *FoundChannel)
{
    API_CALL(CAN_LookUpChannel_locked(Parameters, FoundChannel));
}

EXPORT TPCANStatus CAN_InitializeXL(TPCANHandle Channel, TPCANBitrateXL BitrateXL)
{
    API_CALL(CAN_InitializeXL_locked(Channel, BitrateXL));
}

EXPORT TPCANStatus CAN_ReadXL(TPCANHandle Channel, TPCANMsgXL *MessageBuffer,
                              TPCANTimestampXL *TimestampBuffer)
{
    API_CALL(CAN_ReadXL_locked(Channel, MessageBuffer, TimestampBuffer));
}

EXPORT TPCANStatus CAN_WriteXL(TPCANHandle Channel, TPCANMsgXL *MessageBuffer)
{
    API_CALL(CAN_WriteXL_locked(Channel, MessageBuffer));
}
