#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/types.h>
#include <libusb.h>
#include <errno.h>
#include <unistd.h>
#include <ctype.h>

#include "stlink.h"

enum SCSI_Generic_Direction {SG_DXFER_TO_DEV=0, SG_DXFER_FROM_DEV=0x80};

void _stlink_usb_close(stlink_t* sl) {
    if (!sl)
        return;

    stlink_serial_clear(&sl->serial);

    struct stlink_libusb * const handle = sl->backend_data;
    // maybe we couldn't even get the usb device?
    if (handle != NULL) {
        if (handle->usb_handle != NULL) {
            libusb_close(handle->usb_handle);
        }

        free(handle);
    }
}

ssize_t send_recv(struct stlink_libusb* handle, int terminate,
        unsigned char* txbuf, size_t txsize,
        unsigned char* rxbuf, size_t rxsize) {
    /* note: txbuf and rxbuf can point to the same area */
    int res = 0;
    int t;

    t = libusb_bulk_transfer(handle->usb_handle, handle->ep_req,
            txbuf,
            (int) txsize,
            &res,
            3000);
    if (t) {
        printf("[!] send_recv send request failed: %s\n", libusb_error_name(t));
        return -1;
    } else if ((size_t)res != txsize) {
        printf("[!] send_recv send request wrote %u bytes (instead of %u).\n",
       (unsigned int)res, (unsigned int)txsize);
    }

    if (rxsize != 0) {
        t = libusb_bulk_transfer(handle->usb_handle, handle->ep_rep,
                rxbuf,
                (int) rxsize,
                &res,
                3000);
        if (t) {
            printf("[!] send_recv read reply failed: %s\n",
                    libusb_error_name(t));
            return -1;
        }
    }

    if ((handle->protocoll == STLINK_PROTOCOLL_V1) && terminate) {
        /* Read the SG reply */
        unsigned char sg_buf[13];
        t = libusb_bulk_transfer(handle->usb_handle, handle->ep_rep,
                sg_buf,
                13,
                &res,
                3000);
        if (t) {
            printf("[!] send_recv read storage failed: %s\n",
                    libusb_error_name(t));
            return -1;
        }
        /* The STLink doesn't seem to evaluate the sequence number */
        handle->sg_transfer_idx++;
    }

    return res;
}

static inline int send_only
(struct stlink_libusb* handle, int terminate,
 unsigned char* txbuf, size_t txsize) {
    return (int) send_recv(handle, terminate, txbuf, txsize, NULL, 0);
}


static int fill_command
(stlink_t * sl, enum SCSI_Generic_Direction dir, uint32_t len) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const cmd = sl->c_buf;
    int i = 0;
    memset(cmd, 0, sizeof (sl->c_buf));
    if(slu->protocoll == STLINK_PROTOCOLL_V1) {
        cmd[i++] = 'U';
        cmd[i++] = 'S';
        cmd[i++] = 'B';
        cmd[i++] = 'C';
        write_uint32(&cmd[i], slu->sg_transfer_idx);
        write_uint32(&cmd[i + 4], len);
        i += 8;
        cmd[i++] = (dir == SG_DXFER_FROM_DEV)?0x80:0;
        cmd[i++] = 0; /* Logical unit */
        cmd[i++] = 0xa; /* Command length */
    }
    return i;
}

int _stlink_usb_version(stlink_t *sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    uint32_t rep_len = 6;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_GET_VERSION;

    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_GET_VERSION\n");
        return (int) size;
    }

    return 0;
}

int32_t _stlink_usb_target_voltage(stlink_t *sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const rdata = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    uint32_t rep_len = 8;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);
    uint32_t factor, reading;
    int voltage;

    cmd[i++] = STLINK_GET_TARGET_VOLTAGE;

    size = send_recv(slu, 1, cmd, slu->cmd_len, rdata, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_GET_TARGET_VOLTAGE\n");
        return -1;
    } else if (size != 8) {
        printf("[!] wrong length STLINK_GET_TARGET_VOLTAGE\n");
        return -1;
    }

    factor = (rdata[3] << 24) | (rdata[2] << 16) | (rdata[1] << 8) | (rdata[0] << 0);
    reading = (rdata[7] << 24) | (rdata[6] << 16) | (rdata[5] << 8) | (rdata[4] << 0);
    voltage = 2400 * reading / factor;

    return voltage;
}

int _stlink_usb_read_debug32(stlink_t *sl, uint32_t addr, uint32_t *data) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const rdata = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    const int rep_len = 8;

    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);
    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_JTAG_READDEBUG_32BIT;
    write_uint32(&cmd[i], addr);
    size = send_recv(slu, 1, cmd, slu->cmd_len, rdata, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_JTAG_READDEBUG_32BIT\n");
        return (int) size;
    }
    *data = read_uint32(rdata, 4);
    return 0;
}

int _stlink_usb_write_debug32(stlink_t *sl, uint32_t addr, uint32_t data) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const rdata = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    const int rep_len = 2;

    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);
    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_JTAG_WRITEDEBUG_32BIT;
    write_uint32(&cmd[i], addr);
    write_uint32(&cmd[i + 4], data);
    size = send_recv(slu, 1, cmd, slu->cmd_len, rdata, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_JTAG_WRITEDEBUG_32BIT\n");
        return (int) size;
    }

    return 0;
}

int _stlink_usb_write_mem32(stlink_t *sl, uint32_t addr, uint16_t len) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    int i, ret;

    i = fill_command(sl, SG_DXFER_TO_DEV, len);
    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_WRITEMEM_32BIT;
    write_uint32(&cmd[i], addr);
    write_uint16(&cmd[i + 4], len);
    ret = send_only(slu, 0, cmd, slu->cmd_len);
    if (ret == -1)
        return ret;

    ret = send_only(slu, 1, data, len);
    if (ret == -1)
        return ret;

    return 0;
}

int _stlink_usb_write_mem8(stlink_t *sl, uint32_t addr, uint16_t len) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    int i, ret;

    i = fill_command(sl, SG_DXFER_TO_DEV, 0);
    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_WRITEMEM_8BIT;
    write_uint32(&cmd[i], addr);
    write_uint16(&cmd[i + 4], len);
    ret = send_only(slu, 0, cmd, slu->cmd_len);
    if (ret == -1)
        return ret;

    ret = send_only(slu, 1, data, len);
    if (ret == -1)
        return ret;

    return 0;
}


int _stlink_usb_current_mode(stlink_t * sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const cmd  = sl->c_buf;
    unsigned char* const data = sl->q_buf;
    ssize_t size;
    int rep_len = 2;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_GET_CURRENT_MODE;
    size = send_recv(slu, 1, cmd,  slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_GET_CURRENT_MODE\n");
        return -1;
    }
    return sl->q_buf[0];
}

int _stlink_usb_core_id(stlink_t * sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const cmd  = sl->c_buf;
    unsigned char* const data = sl->q_buf;
    ssize_t size;
    int rep_len = 4;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_READCOREID;

    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_READCOREID\n");
        return -1;
    }

    sl->core_id = read_uint32(data, 0);
    return 0;
}

int _stlink_usb_status(stlink_t * sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    int rep_len = 2;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_GETSTATUS;

    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_GETSTATUS\n");
        return (int) size;
    }
    sl->q_len = (int) size;

    return 0;
}

int _stlink_usb_force_debug(stlink_t *sl) {
    struct stlink_libusb *slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    int rep_len = 2;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_FORCEDEBUG;
    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_FORCEDEBUG\n");
        return (int) size;
    }

    return 0;
}

int _stlink_usb_enter_swd_mode(stlink_t * sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    const int rep_len = 0;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_ENTER;
    cmd[i++] = STLINK_DEBUG_ENTER_SWD;

    size = send_only(slu, 1, cmd, slu->cmd_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_ENTER\n");
        return (int) size;
    }

    return 0;
}

int _stlink_usb_exit_dfu_mode(stlink_t* sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const cmd = sl->c_buf;
    ssize_t size;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, 0);

    cmd[i++] = STLINK_DFU_COMMAND;
    cmd[i++] = STLINK_DFU_EXIT;

    size = send_only(slu, 1, cmd, slu->cmd_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DFU_EXIT\n");
        return (int) size;
    }

    return 0;
}

/**
 * TODO - not convinced this does anything...
 * @param sl
 */
int _stlink_usb_reset(stlink_t * sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd = sl->c_buf;
    ssize_t size;
    int rep_len = 2;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_RESETSYS;

    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_RESETSYS\n");
        return (int) size;
    }

    return 0;
}


int _stlink_usb_jtag_reset(stlink_t * sl, int value) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd = sl->c_buf;
    ssize_t size;
    int rep_len = 2;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_JTAG_DRIVE_NRST;
    cmd[i++] = value;

    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_JTAG_DRIVE_NRST\n");
        return (int) size;
    }

    return 0;
}


int _stlink_usb_step(stlink_t* sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd = sl->c_buf;
    ssize_t size;
    int rep_len = 2;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_STEPCORE;

    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_STEPCORE\n");
        return (int) size;
    }

    return 0;
}

/**
 * This seems to do a good job of restarting things from the beginning?
 * @param sl
 */
int _stlink_usb_run(stlink_t* sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd = sl->c_buf;
    ssize_t size;
    int rep_len = 2;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_RUNCORE;

    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_RUNCORE\n");
        return (int) size;
    }

    return 0;
}


int _stlink_usb_set_swdclk(stlink_t* sl, uint16_t clk_divisor) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd = sl->c_buf;
    ssize_t size;
    int rep_len = 2;
    int i;
    
    // clock speed only supported by stlink/v2 and for firmware >= 22
    if (sl->version.stlink_v >= 2 && sl->version.jtag_v >= 22) {
        i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

        cmd[i++] = STLINK_DEBUG_COMMAND;
        cmd[i++] = STLINK_DEBUG_APIV2_SWD_SET_FREQ;
        cmd[i++] = clk_divisor & 0xFF;
        cmd[i++] = (clk_divisor >> 8) & 0xFF;

        size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
        if (size == -1) {
            printf("[!] send_recv STLINK_DEBUG_APIV2_SWD_SET_FREQ\n");
            return (int) size;
        }

        return 0;
    } else {
        return -1;
    }
}

int _stlink_usb_exit_debug_mode(stlink_t *sl) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const cmd = sl->c_buf;
    ssize_t size;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, 0);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_EXIT;

    size = send_only(slu, 1, cmd, slu->cmd_len);
    if (size == -1) {
        printf("[!] send_only STLINK_DEBUG_EXIT\n");
        return (int) size;
    }

    return 0;
}

int _stlink_usb_read_mem32(stlink_t *sl, uint32_t addr, uint16_t len) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd = sl->c_buf;
    ssize_t size;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_READMEM_32BIT;
    write_uint32(&cmd[i], addr);
    write_uint16(&cmd[i + 4], len);

    size = send_recv(slu, 1, cmd, slu->cmd_len, data, len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_READMEM_32BIT\n");
        return (int) size;
    }

    sl->q_len = (int) size;

    stlink_print_data(sl);
    return 0;
}

int _stlink_usb_read_all_regs(stlink_t *sl, struct stlink_reg *regp) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const cmd = sl->c_buf;
    unsigned char* const data = sl->q_buf;
    ssize_t size;
    uint32_t rep_len = 84;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_READALLREGS;
    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_READALLREGS\n");
        return (int) size;
    }
    sl->q_len = (int) size;
    stlink_print_data(sl);
    for(i=0; i<16; i++)
        regp->r[i]= read_uint32(sl->q_buf, i*4);
    regp->xpsr       = read_uint32(sl->q_buf, 64);
    regp->main_sp    = read_uint32(sl->q_buf, 68);
    regp->process_sp = read_uint32(sl->q_buf, 72);
    regp->rw         = read_uint32(sl->q_buf, 76);
    regp->rw2        = read_uint32(sl->q_buf, 80);
    if (sl->verbose < 2)
        return 0;

    DLOG("xpsr       = 0x%08x\n", read_uint32(sl->q_buf, 64));
    DLOG("main_sp    = 0x%08x\n", read_uint32(sl->q_buf, 68));
    DLOG("process_sp = 0x%08x\n", read_uint32(sl->q_buf, 72));
    DLOG("rw         = 0x%08x\n", read_uint32(sl->q_buf, 76));
    DLOG("rw2        = 0x%08x\n", read_uint32(sl->q_buf, 80));

    return 0;
}

int _stlink_usb_read_reg(stlink_t *sl, int r_idx, struct stlink_reg *regp) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    uint32_t r;
    uint32_t rep_len = 4;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_READREG;
    cmd[i++] = (uint8_t) r_idx;
    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_READREG\n");
        return (int) size;
    }
    sl->q_len = (int) size;
    stlink_print_data(sl);
    r = read_uint32(sl->q_buf, 0);
    DLOG("r_idx (%2d) = 0x%08x\n", r_idx, r);

    switch (r_idx) {
    case 16:
        regp->xpsr = r;
        break;
    case 17:
        regp->main_sp = r;
        break;
    case 18:
        regp->process_sp = r;
        break;
    case 19:
        regp->rw = r; /* XXX ?(primask, basemask etc.) */
        break;
    case 20:
        regp->rw2 = r; /* XXX ?(primask, basemask etc.) */
        break;
    default:
        regp->r[r_idx] = r;
    }

    return 0;
}

/* See section C1.6 of the ARMv7-M Architecture Reference Manual */
int _stlink_usb_read_unsupported_reg(stlink_t *sl, int r_idx, struct stlink_reg *regp) {
    uint32_t r;
    int ret;

    sl->q_buf[0] = (unsigned char) r_idx;
    for (int i = 1; i < 4; i++) {
        sl->q_buf[i] = 0;
    }

    ret = _stlink_usb_write_mem32(sl, STLINK_REG_DCRSR, 4);
    if (ret == -1)
        return ret;

    _stlink_usb_read_mem32(sl, STLINK_REG_DCRDR, 4);
    if (ret == -1)
        return ret;

    r = read_uint32(sl->q_buf, 0);
    DLOG("r_idx (%2d) = 0x%08x\n", r_idx, r);

    switch (r_idx) {
    case 0x14:
        regp->primask = (uint8_t) (r & 0xFF);
        regp->basepri = (uint8_t) ((r>>8) & 0xFF);
        regp->faultmask = (uint8_t) ((r>>16) & 0xFF);
        regp->control = (uint8_t) ((r>>24) & 0xFF);
        break;
    case 0x21:
        regp->fpscr = r;
        break;
    default:
        regp->s[r_idx - 0x40] = r;
        break;
    }

    return 0;
}

int _stlink_usb_read_all_unsupported_regs(stlink_t *sl, struct stlink_reg *regp) {
    int ret;

    ret = _stlink_usb_read_unsupported_reg(sl, 0x14, regp);
    if (ret == -1)
        return ret;

    ret = _stlink_usb_read_unsupported_reg(sl, 0x21, regp);
    if (ret == -1)
        return ret;

    for (int i = 0; i < 32; i++) {
        ret = _stlink_usb_read_unsupported_reg(sl, 0x40+i, regp);
        if (ret == -1)
            return ret;
    }

    return 0;
}

/* See section C1.6 of the ARMv7-M Architecture Reference Manual */
int _stlink_usb_write_unsupported_reg(stlink_t *sl, uint32_t val, int r_idx, struct stlink_reg *regp) {
    int ret;

    if (r_idx >= 0x1C && r_idx <= 0x1F) { /* primask, basepri, faultmask, or control */
        /* These are held in the same register */
        ret = _stlink_usb_read_unsupported_reg(sl, 0x14, regp);
        if (ret == -1)
            return ret;

        val = (uint8_t) (val>>24);

        switch (r_idx) {
        case 0x1C:  /* control */
            val = (((uint32_t) val) << 24) | (((uint32_t) regp->faultmask) << 16) | (((uint32_t) regp->basepri) << 8) | ((uint32_t) regp->primask);
            break;
        case 0x1D:  /* faultmask */
            val = (((uint32_t) regp->control) << 24) | (((uint32_t) val) << 16) | (((uint32_t) regp->basepri) << 8) | ((uint32_t) regp->primask);
            break;
        case 0x1E:  /* basepri */
            val = (((uint32_t) regp->control) << 24) | (((uint32_t) regp->faultmask) << 16) | (((uint32_t) val) << 8) | ((uint32_t) regp->primask);
            break;
        case 0x1F:  /* primask */
            val = (((uint32_t) regp->control) << 24) | (((uint32_t) regp->faultmask) << 16) | (((uint32_t) regp->basepri) << 8) | ((uint32_t) val);
            break;
        }

        r_idx = 0x14;
    }

    write_uint32(sl->q_buf, val);

    ret = _stlink_usb_write_mem32(sl, STLINK_REG_DCRDR, 4);
    if (ret == -1)
        return ret;

    sl->q_buf[0] = (unsigned char) r_idx;
    sl->q_buf[1] = 0;
    sl->q_buf[2] = 0x01;
    sl->q_buf[3] = 0;

    return _stlink_usb_write_mem32(sl, STLINK_REG_DCRSR, 4);
}

int _stlink_usb_write_reg(stlink_t *sl, uint32_t reg, int idx) {
    struct stlink_libusb * const slu = sl->backend_data;
    unsigned char* const data = sl->q_buf;
    unsigned char* const cmd  = sl->c_buf;
    ssize_t size;
    uint32_t rep_len = 2;
    int i = fill_command(sl, SG_DXFER_FROM_DEV, rep_len);

    cmd[i++] = STLINK_DEBUG_COMMAND;
    cmd[i++] = STLINK_DEBUG_WRITEREG;
    cmd[i++] = idx;
    write_uint32(&cmd[i], reg);
    size = send_recv(slu, 1, cmd, slu->cmd_len, data, rep_len);
    if (size == -1) {
        printf("[!] send_recv STLINK_DEBUG_WRITEREG\n");
        return (int) size;
    }
    sl->q_len = (int) size;
    stlink_print_data(sl);

    return 0;
}

static stlink_backend_t _stlink_usb_backend = {
    _stlink_usb_close,
    _stlink_usb_exit_debug_mode,
    _stlink_usb_enter_swd_mode,
    NULL,  // no enter_jtag_mode here...
    _stlink_usb_exit_dfu_mode,
    _stlink_usb_core_id,
    _stlink_usb_reset,
    _stlink_usb_jtag_reset,
    _stlink_usb_run,
    _stlink_usb_status,
    _stlink_usb_version,
    _stlink_usb_read_debug32,
    _stlink_usb_read_mem32,
    _stlink_usb_write_debug32,
    _stlink_usb_write_mem32,
    _stlink_usb_write_mem8,
    _stlink_usb_read_all_regs,
    _stlink_usb_read_reg,
    _stlink_usb_read_all_unsupported_regs,
    _stlink_usb_read_unsupported_reg,
    _stlink_usb_write_unsupported_reg,
    _stlink_usb_write_reg,
    _stlink_usb_step,
    _stlink_usb_current_mode,
    _stlink_usb_force_debug,
    _stlink_usb_target_voltage,
    _stlink_usb_set_swdclk
};

/* unsafe outserial length */
int stlink_parse_serial(const char *serial, uint8_t *outserial, int *new_serial_size)
{
    /** @todo This is not really portable, as strlen really returns size_t we need to obey and not cast it to a signed type. */
    size_t j = strlen(serial);
    size_t length = j / 2;  //the length of the destination-array
    if ((j % 2 != 0) || (length > STLINK_SERIAL_SIZE)) return -1;

    /* Verify that serial is in ascii representation
       @ref http://www.st.com/resource/en/application_note/cd00169020.pdf */
    char buffer[3] = { 0, 0, 0};
    for (size_t k = 0; k<j; ++k) {
        if ( (serial[k] >= '0' && serial[k] <= '9') ||
             (isalpha(serial[k]) && tolower(serial[k])>='a' && tolower(serial[k])<='f') )
            continue;
        DLOG("Not ascii serial. Raw copy serial.\n");
        goto rawcopy; // Could not convert non-ascii serial representation to hex
    }

    for (size_t k = 0, f=0; k<length && k<STLINK_SERIAL_SIZE; ++k, ++f, ++f) {
        // Usage memcpy here is overhead
        buffer[0] = serial[f];
        buffer[1] = serial[f+1];
        outserial[k] = (uint8_t)strtoul(buffer, NULL, 16);
    }
    if (new_serial_size != NULL)
        *new_serial_size=(int)length;

    return 0;

rawcopy:
    for (size_t k = 0; k<j; ++k)
        outserial[k] = serial[k];

    if (new_serial_size != NULL)
        *new_serial_size=(int)j;

    return -2;
}

int stlink_serial_clear(stlink_serial_t *sst)
{
    if (sst->data != NULL) {
        free(sst->data);
        sst->data = NULL;
    }
    sst->size   = 0;
    sst->format = STSERIALF_UNKNOWN;
    return 0;
}
int stlink_serial_init(stlink_serial_t *sst, enum stlink_serial_format_type format, size_t size)
{
    if (size>0) {
        sst->format = format;
        sst->size   = size;
        sst->data   = (uint8_t *)calloc(size, sizeof(uint8_t));
    }
    return 0;
}
enum stlink_serial_format_type
stlink_serial_format_detect(const stlink_serial_t *sst)
{
    /* @todo: parse data until unprintable chars
     * like ASCII "3031" is HEX "01" and is BINARY "\x01"
     */
    stlink_serial_t probe = {
        .format=(sst->format==STSERIALF_UNKNOWN ? STSERIALF_BINARY : sst->format),
        .data=sst->data,
        .size=sst->size
    };

    // Check for ascii to hex convertation
    for(;;) {
        return STSERIALF_UNKNOWN;
    }

    probe.format++;
    if (probe.format == STSERIALF_ASCII)
        return STSERIALF_ASCII;
    else
        return stlink_serial_format_detect(&probe);
}

int stlink_serial_cmp(const stlink_serial_t *sst_first, stlink_serial_t *sst_second)
{
    return 0;
}

int stlink_serial_convert(const stlink_serial_t *sst_from, stlink_serial_t *sst_to, enum stlink_serial_format_type toformat)
{
    if ( (sst_from->format != STSERIALF_BINARY) && (sst_from->size % 2) != 0)
        return -1;

    // Simple copy "as is"
    if (sst_from->format == toformat) {
        sst_to->format = sst_from->format;
        sst_to->size   = sst_from->size;
        sst_to->data   = (uint8_t*)malloc(sst_from->size*sizeof(uint8_t));
        memcpy(sst_to->data, sst_from->data, sst_from->size);
        //int x;
        //for(x=0; x<sst_from->size; ++x)
        //    printf("%02x", sst_from->data[x]);
        //printf("\n");
        return 0;
    }
    printf("reformat %d -> %d\n", sst_from->format, toformat);

    // Alloc enought size to convering without reallocation
    ssize_t bufsz = ( (sst_from->format > toformat) ? sst_from->size
                        : (sst_from->size*2*(toformat - sst_from->format)) );
    uint8_t *buf = (uint8_t*)calloc(bufsz, sizeof(uint8_t));
    uint8_t curfmt = sst_from->format;

    memcpy(buf, sst_from->data, sst_from->size);

    if (curfmt>toformat) {
        for(; curfmt>toformat; --curfmt) {
            for(int it=0, ot=0; it<bufsz; ++it, ++it, ++ot) {
                char conv[3] = { buf[it], buf[it+1], 0 };
                long res = strtol(conv, NULL, 16);
                buf[ot] = (uint8_t)res;
            }
            bufsz /= 2;
        }
    } else {
        for(; curfmt<toformat; ++curfmt) {
            for(int it=bufsz-1, ot=it*2; it>=0; --it, --ot, --ot) {
                char conv[3] = { 0, 0, 0 };
                snprintf(conv, 2, "%2x", buf[it]);
                buf[ot  ]=conv[1];
                buf[ot-1]=conv[0];
            }
            bufsz *= 2;
        }
    }
    //

    free(buf);
    return -1;
}

struct stlink_usr_match_t {
    int devBus;
    int devAddr;
    stlink_serial_t serial;
    //uint8_t serial[STLINK_SERIAL_SIZE];
    //int serial_size;
};

/* @todo Ready to move out from stlink_open_usb. See comment in stlink_open_usb */
static const
struct stlink_usr_match_t* _stlink_get_usr_match() {
    static struct stlink_usr_match_t susm;
    const char *device, *serial;

    susm.devBus = 0;
    susm.devAddr = 0;
    stlink_serial_clear(&susm.serial);

    device = getenv("STLINK_DEVICE");
    serial = getenv("STLINK_SERIAL");

    if (device) {
        for (const char *devvrf=device; *devvrf != '\0'; ++devvrf) {
            if (*devvrf != ':' && ! (*devvrf>='0' && *devvrf<='9')) {
                WLOG("STLINK_DEVICE should match pattern '[0-9]\\+:[0-9]\\+'\n");
                return NULL;
            }
        }
        char *c = strchr(device,':');
        if (c==NULL) {
            WLOG("STLINK_DEVICE must be <USB_BUS>:<USB_ADDR> format\n");
            return NULL;
        }
        susm.devBus=atoi(device);
        //*c++=0; // unnecessary
        susm.devAddr=atoi(++c);
        ILOG("bus %03d dev %03d\n", susm.devBus, susm.devAddr);
    }

    if (serial) {
        DLOG("User serial set: %s\n", serial);
        const stlink_serial_t sst = { .format=STSERIALF_HEX, .data=(uint8_t*)serial, .size=strlen(serial) };
        //if (stlink_parse_serial(serial, &susm.serial) == -1)
        if (stlink_serial_convert(&sst, &susm.serial, STSERIALF_BINARY) == -1)
            return NULL;
    }

    return &susm;
}


//static libusb_context  *libusb_ctx=NULL;
static libusb_device  **libusb_devices=NULL;

static stlink_t       **stlink_devices=NULL;
static stlink_t       **stlink_device_current=NULL;
static ssize_t          stlink_devices_count=0;

//static
ssize_t stlink_init_devices()
{
    if (libusb_devices == NULL) {
        static libusb_context  *libusb_ctx=NULL;
        if (libusb_init(&libusb_ctx)) {
            WLOG("failed to init libusb context, wrong version of libraries?\n");
            return -2;
        }

        ssize_t cnt = (int) libusb_get_device_list(NULL, &libusb_devices);
        if (cnt<0)
            return 0;

        stlink_t **_stlink_devices = NULL;
        struct  libusb_device_descriptor  desc;
        struct  libusb_device_handle     *handle = NULL;
        int     serial_size;
        uint8_t serial[256];
        int     config;

        stlink_t *sl = NULL;
        struct stlink_libusb* slu = NULL;

        ssize_t id;
        for (id=0; id<cnt; ++id) {
            int ret = libusb_get_device_descriptor(libusb_devices[id], &desc);
            if (ret < 0) {
                WLOG("failed to get libusb device descriptor\n");
                return -1;
            }

            if (desc.idVendor != STLINK_USB_VID_ST)
                continue;
            if (desc.idProduct != STLINK_USB_PID_STLINK &&
                desc.idProduct != STLINK_USB_PID_STLINK_32L &&
                desc.idProduct != STLINK_USB_PID_STLINK_NUCLEO) {
                WLOG("Unsupported STLINK device: %04x:%04x", desc.idVendor, desc.idProduct);
                WLOG("Please, contact to owner");
                continue;
            }

            ret = libusb_open(libusb_devices[id], &handle);
            if (ret < 0) {
                ugly_init(UERROR); /* Don't be silent when error is occured */
                ELOG("Cannot open device %04x:%04x on %03d:%03d: %s(%d)\n", desc.idVendor, desc.idProduct,
                        libusb_get_bus_number(libusb_devices[id]), libusb_get_device_address(libusb_devices[id]),
                        libusb_error_name(ret), ret);
                continue;
            }

            memset(serial, 0x0, sizeof(serial));
            serial_size = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                                                             (uint8_t *)serial, sizeof(serial));
            if (serial_size < 0) {
                continue;
            }

            ++stlink_devices_count;



            sl  = (stlink_t *) calloc(1, sizeof(stlink_t));
            slu = calloc(1, sizeof (struct stlink_libusb));
            if (!sl) {
                return -2;
            }
            if (!slu) {
                free(sl);
                return -2;
            }
            sl->backend = &_stlink_usb_backend;
            sl->backend_data = slu;

            sl->core_stat = STLINK_CORE_STAT_UNKNOWN;

            // Convert (copy) serial
            stlink_serial_t ssf = { .format=STSERIALF_BINARY, .data=serial, .size=serial_size };
            stlink_serial_convert(&ssf, &sl->serial, STSERIALF_BINARY);

            slu->usb_handle = handle;
            slu->libusb_ctx = libusb_ctx;
            if (desc.idProduct == STLINK_USB_PID_STLINK) {
                slu->protocoll = STLINK_PROTOCOLL_V1;
            }

            if (libusb_kernel_driver_active(slu->usb_handle, 0) == 1) {
                ret = libusb_detach_kernel_driver(slu->usb_handle, 0);
                if (ret < 0) {
                    WLOG("libusb_detach_kernel_driver(() error %s\n", libusb_error_name(ret));
                    goto on_libusb_error;
                }
            }

            if (libusb_get_configuration(slu->usb_handle, &config)) {
                /* this may fail for a previous configured device */
                WLOG("libusb_get_configuration()\n");
                goto on_libusb_error;
            }

            if (config != 1) {
                printf("setting new configuration (%d -> 1)\n", config);
                if (libusb_set_configuration(slu->usb_handle, 1)) {
                    /* this may fail for a previous configured device */
                    WLOG("libusb_set_configuration() failed\n");
                    goto on_libusb_error;
                }
            }

            if (libusb_claim_interface(slu->usb_handle, 0)) {
                WLOG("Stlink usb device found, but unable to claim (probably already in use?)\n");
                goto on_libusb_error;
            }

            // TODO - could use the scanning techniq from stm8 code here...
            slu->ep_rep = 1 /* ep rep */ | LIBUSB_ENDPOINT_IN;
            if (desc.idProduct == STLINK_USB_PID_STLINK_NUCLEO) {
                slu->ep_req = 1 /* ep req */ | LIBUSB_ENDPOINT_OUT;
            } else {
                slu->ep_req = 2 /* ep req */ | LIBUSB_ENDPOINT_OUT;
            }

            slu->sg_transfer_idx = 0;
            // TODO - never used at the moment, always CMD_SIZE
            slu->cmd_len = (slu->protocoll == STLINK_PROTOCOLL_V1) ? STLINK_SG_SIZE : STLINK_CMD_SIZE;

            if (stlink_current_mode(sl) == STLINK_DEV_DFU_MODE) {
                ILOG("-- exit_dfu_mode\n");
                stlink_exit_dfu_mode(sl);
            }

            if (stlink_current_mode(sl) != STLINK_DEV_DEBUG_MODE) {
                stlink_enter_swd_mode(sl);
            }

            // Initialize stlink version (sl->version)
            stlink_version(sl);



            _stlink_devices = (stlink_t **)realloc(_stlink_devices, sizeof(stlink_t *) * (stlink_devices_count+1));
            if (!_stlink_devices) {
                stlink_close(sl);
                return -2;
            }
            _stlink_devices[stlink_devices_count]   = NULL;
            _stlink_devices[stlink_devices_count-1] = sl;
        }
        if (stlink_devices_count>0) {
            stlink_devices = _stlink_devices;
        }
    }
on_libusb_error:
    return stlink_devices_count;
}

//static
ssize_t stlink_get_devices(stlink_t ***devs)
{
    if (devs != NULL)
        *devs = stlink_devices;
    return stlink_devices_count;
}

//static
stlink_t* stlink_device_next()
{
    if (stlink_device_current == NULL) {
        stlink_device_current = stlink_devices;
        return *stlink_device_current;
    }
    if (*stlink_device_current == NULL)
        return NULL;
    return *(++stlink_device_current);
}

//static
void stlink_devices_rewind()
{
    stlink_device_current = NULL;
}

//static
void stlink_free_devices()
{
    struct libusb_context *libusb_ctx = NULL;
    if (stlink_devices) {
        stlink_t **sl;
        for(sl = stlink_devices; *sl != NULL; ++sl) {
            struct stlink_libusb *slu = (*sl)->backend_data;
            printf("STLINK V%d", (*sl)->version.stlink_v);
            if (slu->protocoll == STLINK_PROTOCOLL_V1) {
                if( (*sl)->version.stlink_v > 1 )
                    stlink_jtag_reset(sl, 2);
                stlink_reset(*sl);
                stlink_exit_debug_mode(*sl);
            }
            libusb_ctx = slu->libusb_ctx;
            stlink_close(*sl);
        }
        free(stlink_devices);
        stlink_devices = NULL;
    }

    stlink_devices_count = 0;

    if (libusb_devices) {
        libusb_free_device_list(libusb_devices, 1);
        libusb_devices = NULL;
    }

    //libusb_exit(NULL);
    if (libusb_ctx)
        libusb_exit(libusb_ctx);
}


stlink_t *stlink_open_usb(enum ugly_loglevel verbose, bool reset, const stlink_serial_t *serial)
{
    stlink_t* sl = NULL;
    struct stlink_libusb* slu = NULL;
    int ret = -1;
    int config;

    stlink_init_devices();
    stlink_devices_rewind();
    while ((sl = stlink_device_next()) != NULL) {
        printf("Device iterate %p\n", sl);
        if (serial->size == 0)
            break;
        if (serial->size == sl->serial.size &&
            memcmp(serial->data, sl->serial.data, sl->serial.size) == 0)
            break;
    }
    if (!sl) {
        stlink_free_devices();
        sl = NULL;
    }
    return sl;

    sl  = calloc(1, sizeof (stlink_t));
    slu = calloc(1, sizeof (struct stlink_libusb));
    if (sl == NULL)
        goto on_malloc_error;
    if (slu == NULL)
        goto on_malloc_error;

    ugly_init(verbose);
    sl->backend = &_stlink_usb_backend;
    sl->backend_data = slu;

    sl->core_stat = STLINK_CORE_STAT_UNKNOWN;
    if (libusb_init(&(slu->libusb_ctx))) {
        WLOG("failed to init libusb context, wrong version of libraries?\n");
        goto on_error;
    }

    libusb_device **list;
    /** @todo We should use ssize_t and use it as a counter if > 0. As per libusb API: ssize_t libusb_get_device_list (libusb_context *ctx, libusb_device ***list) */
    ssize_t cnt = (int) libusb_get_device_list(slu->libusb_ctx, &list);
    struct libusb_device_descriptor desc;

    /* @TODO: Reading a environment variable in a usb open function is not very nice, this
      should be refactored and moved into the CLI tools, and instead of giving USB_BUS:USB_ADDR a real stlink
      serial string should be passed to this function. Probably people are using this but this is very odd because
      as programmer can change to multiple busses and it is better to detect them based on serial.  */
    struct stlink_usr_match_t *susm = _stlink_get_usr_match();
    if (susm == NULL)
        goto on_error;
    if (serial && serial->size>0)
        stlink_serial_convert(serial, &susm->serial, STSERIALF_BINARY);

    while (cnt-->0) {
        libusb_get_device_descriptor( list[cnt], &desc );
        if (susm->devBus && susm->devAddr) {
            if ((libusb_get_bus_number(list[cnt]) != susm->devBus)
                || (libusb_get_device_address(list[cnt]) != susm->devAddr)) {
                continue;
            }
            if (desc.idVendor != STLINK_USB_VID_ST) {
                ugly_init(UWARN); /* Don't be silent when error is occured */
                WLOG("Supported only vendor %04x in STLINK_DEVICE. Choosen device has vendor %04x\n", STLINK_USB_VID_ST, desc.idVendor);
                goto on_error;
            }
        } else {
            if (desc.idVendor != STLINK_USB_VID_ST)
                continue;
        }

        if ((desc.idProduct == STLINK_USB_PID_STLINK) ||
            (desc.idProduct == STLINK_USB_PID_STLINK_32L) ||
            (desc.idProduct == STLINK_USB_PID_STLINK_NUCLEO)) {
            struct libusb_device_handle *handle;

            ret = libusb_open(list[cnt], &handle);
            if (ret < 0) {
                ugly_init(UERROR); /* Don't be silent when error is occured */
                ELOG("Cannot open device %04x:%04x on %03d:%03d: %s(%d)\n", desc.idVendor, desc.idProduct,
                        libusb_get_bus_number(list[cnt]), libusb_get_device_address(list[cnt]),
                        libusb_error_name(ret), ret);
                if (susm->devBus && susm->devAddr) goto on_error;
                continue;
            }

            uint8_t serial_buf[256];
            int serial_size;

            memset(serial_buf, 0x0, sizeof(serial_buf));
            serial_size = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                                                             (uint8_t *)serial_buf, sizeof(serial_buf));

            if (serial_size==12 && memcmp(serial_buf, "000000000001", 12)==0) {
                reset = 2;
            }

            libusb_close(handle);

            if (serial_size < 0)
                continue;

            // Set protocoll version before possible breaks
            slu->protocoll = STLINK_PROTOCOLL_V2;
            if (desc.idProduct == STLINK_USB_PID_STLINK) {
                slu->protocoll = STLINK_PROTOCOLL_V1;
            }

            const stlink_serial_t devserial = { .format=(desc.idProduct==STLINK_USB_PID_STLINK ? STSERIALF_BINARY : STSERIALF_HEX), .data=serial_buf, .size=serial_size };
            //stlink_parse_serial((const char *)sl->serial, sl->serial, &sl->serial_size);
            stlink_serial_convert(&devserial, &sl->serial, STSERIALF_BINARY);

            if (serial) {
                printf("serial: "); for(unsigned int x=0; x<serial->size; ++x)
                    { printf("%02x", serial->data[x]); } printf("\n");
            }
            printf("sl->serial: "); for(unsigned int x=0; x<sl->serial.size; ++x)
                { printf("%02x", sl->serial.data[x]); } printf("\n");
            printf("susm->serial: "); for(unsigned int x=0; x<susm->serial.size; ++x)
                { printf("%02x", susm->serial.data[x]); } printf("\n");

            // Match user's serial
            if (susm->serial.size>0) {
                if (susm->serial.size == sl->serial.size &&
                    memcmp(susm->serial.data, sl->serial.data, sl->serial.size) == 0)
                    break;
            } else if (susm->serial.size == 0)
            // else if ((serial == NULL) || (serial->size == 0))
                break;

            /*
            if (serial &&
                (serial->size == sl->serial.size) &&
                (memcmp(serial->data, &sl->serial.data, sl->serial.size) == 0))
                break;
            */

            if (susm->devBus && susm->devAddr) goto on_error;
            else continue;
        }
    }

    if (cnt < 0) {
        ugly_init(UWARN); /* Don't be silent when error is occured */
        WLOG ("Couldn't find %s ST-Link/V2 devices\n", ((susm->devBus && susm->devAddr) || (susm->serial.size))?"matched":"any");
        goto on_error;
    } else {
        ret = libusb_open(list[cnt], &slu->usb_handle);
        if (ret < 0) {
            ugly_init(UERROR); /* Don't be silent when error is occured */
            ELOG ("Cannot open ST-Link/V2 device on %03d:%03d: %s(%d)\n",
                 libusb_get_bus_number(list[cnt]), libusb_get_device_address(list[cnt]),
                 libusb_error_name(ret), ret);
            goto on_error;
        }
    }

    libusb_free_device_list(list, 1);

    if (libusb_kernel_driver_active(slu->usb_handle, 0) == 1) {
        ret = libusb_detach_kernel_driver(slu->usb_handle, 0);
        if (ret < 0) {
            WLOG("libusb_detach_kernel_driver(() error %s\n", libusb_error_name(ret));
            goto on_libusb_error;
        }
    }

    if (libusb_get_configuration(slu->usb_handle, &config)) {
        /* this may fail for a previous configured device */
        WLOG("libusb_get_configuration()\n");
        goto on_libusb_error;
    }

    if (config != 1) {
        printf("setting new configuration (%d -> 1)\n", config);
        if (libusb_set_configuration(slu->usb_handle, 1)) {
            /* this may fail for a previous configured device */
            WLOG("libusb_set_configuration() failed\n");
            goto on_libusb_error;
        }
    }

    if (libusb_claim_interface(slu->usb_handle, 0)) {
        WLOG("Stlink usb device found, but unable to claim (probably already in use?)\n");
        goto on_libusb_error;
    }

    // TODO - could use the scanning techniq from stm8 code here...
    slu->ep_rep = 1 /* ep rep */ | LIBUSB_ENDPOINT_IN;
    if (desc.idProduct == STLINK_USB_PID_STLINK_NUCLEO) {
        slu->ep_req = 1 /* ep req */ | LIBUSB_ENDPOINT_OUT;
    } else {
        slu->ep_req = 2 /* ep req */ | LIBUSB_ENDPOINT_OUT;
    }

    slu->sg_transfer_idx = 0;
    // TODO - never used at the moment, always CMD_SIZE
    slu->cmd_len = (slu->protocoll == STLINK_PROTOCOLL_V1) ? STLINK_SG_SIZE : STLINK_CMD_SIZE;

    int curmode = stlink_current_mode(sl);
    if (curmode == STLINK_DEV_DFU_MODE) {
        ILOG("-- exit_dfu_mode\n");
        stlink_exit_dfu_mode(sl);
    }
    if (curmode != STLINK_DEV_DEBUG_MODE) {
        stlink_enter_swd_mode(sl);
    }

    // Initialize stlink version (sl->version)
    stlink_version(sl);

    if (reset) {
        if( sl->version.stlink_v > 1 )
            stlink_jtag_reset(sl, 2);
        stlink_reset(sl);
        usleep(10000);
    }

    ret = stlink_load_device_params(sl);

    // Set the stlink clock speed (default is 1800kHz)
    stlink_set_swdclk(sl, STLINK_SWDCLK_1P8MHZ_DIVISOR);    

on_libusb_error:
    if (ret == -1) {
        stlink_close(sl);
        return NULL;
    }

    return sl;

on_error:
    if (slu->libusb_ctx)
        libusb_exit(slu->libusb_ctx);

on_malloc_error:
    if (sl != NULL)
        free(sl);
    if (slu != NULL)
        free(slu);

    return NULL;
}

static size_t stlink_probe_usb_devs(libusb_device **devs, stlink_t **sldevs[]) {
    stlink_t **_sldevs;
    int i = 0;
    int ret = 0;
    size_t slcnt = 0;
    size_t slcur = 0;

    /* Count stlink */
    for (i=0; devs[i] != NULL; ++i) {
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(devs[i], &desc);
        if (r < 0) {
            WLOG("failed to get libusb device descriptor\n");
            break;
        }

        if (desc.idProduct == STLINK_USB_PID_STLINK &&
            desc.idProduct == STLINK_USB_PID_STLINK_32L &&
            desc.idProduct == STLINK_USB_PID_STLINK_NUCLEO)
            ++slcnt;
        else if (desc.idVendor == STLINK_USB_VID_ST)
            WLOG("unsupported stlink device %04x:%04x", desc.idVendor, desc.idProduct);
    }

    /* Allocate list of pointers */
    _sldevs = calloc(slcnt, sizeof(stlink_t *));
    if (!_sldevs) {
        *sldevs = NULL;
        return 0;
    }

    /* Open stlinks and attach to list */
    for (i=0; devs[i] != NULL; ++i) {
        struct libusb_device_descriptor desc;
        ret = libusb_get_device_descriptor(devs[i], &desc);
        if (ret < 0) {
            WLOG("failed to get libusb device descriptor\n");
            break;
        }

        if (desc.idProduct != STLINK_USB_PID_STLINK &&
            desc.idProduct != STLINK_USB_PID_STLINK_32L &&
            desc.idProduct != STLINK_USB_PID_STLINK_NUCLEO)
            continue;

        struct libusb_device_handle* handle;
        ret = libusb_open(devs[i], &handle);
        if (ret < 0) {
            WLOG("failed to get libusb device descriptor\n");
            break;
        }

        uint8_t serial_buf[256];
        stlink_serial_t sst;
        memset(&sst, 0, sizeof(sst));

        ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, serial_buf, sizeof(serial_buf));
        stlink_serial_clear(&sst);
        if (ret >= 0) {
            //stlink_serial_init(&sst, STSERIALF_HEX, ret);
            stlink_serial_t ssf = { .format=STSERIALF_BINARY, .data=serial_buf, .size=ret };
            stlink_serial_convert(&ssf, &sst, STSERIALF_BINARY);
        }
        libusb_close(handle);

        stlink_t *sl = NULL;
        sl = stlink_open_usb(0, 1, &sst);

        //stlink_serial_clear(&sst);
        if (!sl)
            continue;

        _sldevs[slcur] = sl;
        slcur++;
    }

    /* Something went wrong */
    if (ret < 0) {
        free(_sldevs);
        *sldevs = NULL;
        return 0;
    }

    *sldevs = _sldevs;
    return slcnt;
}

size_t stlink_probe_usb(stlink_t **stdevs[]) {
    libusb_device **devs;
    stlink_t **sldevs;

    size_t slcnt = 0;
    int r;
    ssize_t cnt;

    r = libusb_init(NULL);
    if (r < 0)
        return 0;

    cnt = libusb_get_device_list(NULL, &devs);
    if (cnt < 0)
        return 0;

    slcnt = stlink_probe_usb_devs(devs, &sldevs);
    libusb_free_device_list(devs, 1);

    libusb_exit(NULL);

    *stdevs = sldevs;
    return slcnt;
}

void stlink_probe_usb_free(stlink_t ***stdevs, size_t size) {
    if (stdevs == NULL || *stdevs == NULL || size == 0)
        return;

    for (size_t n = 0; n < size; n++)
        stlink_close((*stdevs)[n]);
    free(*stdevs);
    *stdevs = NULL;
}
