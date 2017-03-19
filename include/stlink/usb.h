/*
 * File:   stlink/usb.h
 * Author: karl
 *
 * Created on October 1, 2011, 11:29 PM
 */

#ifndef STLINK_USB_H
#define STLINK_USB_H

#include <stdbool.h>
#include <libusb.h>

#include "stlink.h"
#include "stlink/logging.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STLINK_USB_VID_ST            0x0483
#define STLINK_USB_PID_STLINK        0x3744
#define STLINK_USB_PID_STLINK_32L    0x3748
#define STLINK_USB_PID_STLINK_NUCLEO 0x374b

#define STLINK_SG_SIZE 31
#define STLINK_CMD_SIZE 16

#define STLINK_PROTOCOLL_V1   0x00010000
#define STLINK_PROTOCOLL_V2   0x00020000
#define STLINK_PROTOCOLL_V2_1 0x00020001

    struct stlink_libusb {
        libusb_context* libusb_ctx;
        libusb_device_handle* usb_handle;
        unsigned int ep_req;
        unsigned int ep_rep;
        int protocoll;
        unsigned int sg_transfer_idx;
        unsigned int cmd_len;
    };

    /**
     * Open a stlink
     * @param verbose Verbosity loglevel
     * @param reset   Reset stlink programmer
     * @param serial  Serial number to search for, when NULL the first stlink found is opened (binary format)
     * @retval NULL   Error while opening the stlink
     * @retval !NULL  Stlink found and ready to use
     */
    //stlink_t *stlink_open_usb(enum ugly_loglevel verbose, bool reset, uint8_t serial[STLINK_SERIAL_SIZE]);
    stlink_t *stlink_open_usb(enum ugly_loglevel verbose, bool reset, const stlink_serial_t *serial);
    size_t    stlink_probe_usb(stlink_t **stdevs[]);
    void      stlink_probe_usb_free(stlink_t **stdevs[], size_t size);

    int stlink_serial_convert(const stlink_serial_t *sst_from, stlink_serial_t *sst_to, enum stlink_serial_format_type toformat);
    int stlink_serial_init(stlink_serial_t *sst, enum stlink_serial_format_type format, size_t size);
    int stlink_serial_clear(stlink_serial_t *sst);

    ssize_t stlink_init_devices();
    stlink_t* stlink_device_next();
    void stlink_devices_rewind();
    void stlink_free_devices();

#ifdef __cplusplus
}
#endif

#endif /* STLINK_USB_H */

