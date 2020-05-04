// Copyright (c) 2020 Dalibor Drgon <dalibor.drgon@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)
/**
 * @file enc28j60.c
 * @author Dalibor Drgon <dalibor.drgon@gmail.com>
 * @brief Part of enc28j60 driver
 * @version 1.0
 * @date 2020-05-01
 * 
 * @copyright Copyright (c) 2020 Dalibor Drgon <dalibor.drgon@gmail.com>
 * 
 * This code is licensed under MIT license (see LICENSE.txt for details)
 */

#include "enc28j60.h"
#include <stddef.h>

// void debugm(const char *c);
// void debugu(uint32_t c);

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
int enc28j60_spi_transfer(
	enc28j60 *instance,
	const char* tx, 
	char *rx, 
	unsigned length,
	enc28j60_spi_callback on_finish
) {
    return enc28j60_spi_advanced_transfer(
        instance,
        tx, rx, length,
        NULL, NULL, 0,
        on_finish
    );
}
#endif

int enc28j60_blockingio(enc28j60 *ins, const char *tx, char *rx, unsigned length) {
    enc28j60_spi_transfer_join(ins);

    int err_code;
    bool repeat;
    uint16_t retry = 1;
    do {
        err_code = enc28j60_spi_transfer(ins, tx, rx, length, NULL);
        if(err_code == 0) break;
        repeat = enc28j60_spi_error_handler(ins, err_code, retry);
        retry++;
    } while(repeat);
    return err_code;
}
static int enc28j60_writebank(enc28j60 *ins, uint8_t next) {
    next = next & 3;
    uint8_t cur = ins->bank & 0x3;
    int err_code = 0;

    if((cur | next) != next) {
        char tx1[] = {ENC28J60_OP_BIT_FIELD_CLR | (ENC28J60_ECON1 & 0x1f), (~next) & 3};
        err_code = enc28j60_blockingio(ins, tx1, NULL, 2);
    }
    if(err_code == 0 && (cur & next) != next) {
        char tx2[] = {ENC28J60_OP_BIT_FIELD_SET | (ENC28J60_ECON1 & 0x1f), next & 3};
        err_code = enc28j60_blockingio(ins, tx2, NULL, 2);
    }
    if(err_code == 0) {
        ins->bank = next;
    }
    return err_code;
}


int enc28j60_ensurebank(enc28j60 *ins, uint8_t bank) {
    bank &= 3;
    uint8_t cur_bank = ins->bank & 3;
    if(cur_bank == bank) {
        return 0;
    }
    
    return enc28j60_writebank(ins, bank);
}

static int enc28j60_prepareforaccess(enc28j60 *ins, uint8_t reg) {
    uint8_t bank = (reg >> 5) & 0x3;
    reg = reg & 0x1F;
    if(reg >= ENC28J60_EIE-1 && reg <= ENC28J60_ECON1) {
        // Those are shared registers, no need to change bank
        return 0;
    }
    // Not shared registers, ensure correct bank is used 
    return enc28j60_ensurebank(ins, bank);
}

int enc28j60_io8(enc28j60 *ins, uint8_t op, uint8_t reg, uint8_t val, uint8_t *out) {
    int status = enc28j60_prepareforaccess(ins, reg);
    if(status) {
        return status;
    }
    char tx[] = {op | (reg & 0x1F), val, 0};
    char rx[] = {0, 0, 0};
    unsigned length = (op == ENC28J60_OP_READ_CTRL_REG && (reg & 0x80)) ? 3 : 2;
    status = enc28j60_blockingio(ins, tx, rx, length);
    if(out) {
        *out = rx[length-1];
    }
    return status;
}

int enc28j60_io16(enc28j60 *ins, uint8_t op, uint8_t reg, uint16_t val, uint16_t *out) {
    int status = enc28j60_prepareforaccess(ins, reg);
    if(status) {
        return status;
    }
    unsigned len = (op == ENC28J60_OP_READ_CTRL_REG) ? 4 : 3;
    char tx[] = {op | (reg & 0x1F), val & 0xFF, val >> 8, 0};
    char rx[] = {0, 0, 0, 0};
    status = enc28j60_blockingio(ins, tx, rx, len);
    if(out) {
        *out = rx[2] | ((uint16_t) rx[3] << 8);
    }
    return status;
}

static char tx_buffer[3];
int enc28j60_o8asynctry(enc28j60 *ins, uint8_t op, uint8_t reg, uint8_t val, enc28j60_spi_callback on_finish) {
    if(on_finish == NULL) {
        return enc28j60_io8(ins, op, reg, val, NULL);
    }
    int status = enc28j60_prepareforaccess(ins, reg);
    if(status) {
        return status;
    }
    tx_buffer[0] = op | (reg & 0x1F);
    tx_buffer[1] = val;
    enc28j60_spi_transfer(ins, tx_buffer, NULL, 2, on_finish);
    return 0;
}

int enc28j60_o16asynctry(enc28j60 *ins, uint8_t op, uint8_t reg, uint16_t val, enc28j60_spi_callback on_finish) {
    if(on_finish == NULL) {
        return enc28j60_io8(ins, op, reg, val, NULL);
    }
    int status = enc28j60_prepareforaccess(ins, reg);
    if(status) {
        return status;
    }
    tx_buffer[0] = op | (reg & 0x1F);
    tx_buffer[1] = val & 0xff;
    tx_buffer[2] = val >> 8;
    enc28j60_spi_transfer(ins, tx_buffer, NULL, 3, on_finish);
    return 0;
}

int enc28j60_read8bitreg(enc28j60 *ins, uint8_t reg, uint8_t *content) {
    return enc28j60_io8(ins, ENC28J60_OP_READ_CTRL_REG, reg, 0, content);
}

int enc28j60_write8bitreg(enc28j60 *ins, uint8_t reg, uint8_t content) {
    return enc28j60_io8(ins, ENC28J60_OP_WRITE_CTRL_REG, reg, content, NULL);
}

int enc28j60_clear8bitreg(enc28j60 *ins, uint8_t reg, uint8_t content) {
    return enc28j60_io8(ins, ENC28J60_OP_BIT_FIELD_CLR, reg, content, NULL);
}

int enc28j60_set8bitreg(enc28j60 *ins, uint8_t reg, uint8_t content) {
    return enc28j60_io8(ins, ENC28J60_OP_BIT_FIELD_SET, reg, content, NULL);
}

int enc28j60_read16bitreg(enc28j60 *ins, uint8_t reg, uint16_t *content) {
    return enc28j60_io16(ins, ENC28J60_OP_READ_CTRL_REG, reg, 0, content);
    // uint8_t lo, hi;
    // int status = enc28j60_read8bitreg(ins, reg, &lo);
    // if(status == 0) {
    //     status = enc28j60_read8bitreg(ins, ((reg+1)&0x1f) | (reg & 0xe0), &hi);
    // }
    // if(status == 0 && content) {
    //     *content = lo | ((uint16_t) hi << 8);
    // }
    // return status;
}

int enc28j60_write16bitreg(enc28j60 *ins, uint8_t reg, uint16_t content) {
    return enc28j60_io16(ins, ENC28J60_OP_WRITE_CTRL_REG, reg, content, NULL);
}

int enc28j60_clear16bitreg(enc28j60 *ins, uint8_t reg, uint16_t content) {
    return enc28j60_io16(ins, ENC28J60_OP_BIT_FIELD_CLR, reg, content, NULL);
}

int enc28j60_set16bitreg(enc28j60 *ins, uint8_t reg, uint16_t content) {
    return enc28j60_io16(ins, ENC28J60_OP_BIT_FIELD_SET, reg, content, NULL);
}

int enc28j60_readphyreg(enc28j60 *ins, uint8_t reg, uint16_t *content) {
    int status = enc28j60_write8bitreg(ins, ENC28J60_MIREGADR, reg);
    if(status == 0) status = enc28j60_set8bitreg(ins, ENC28J60_MICMD, ENC28J60_MICMD_MIIRD);
    if(status == 0) {
        uint8_t mistat;
        do {
            status = enc28j60_read8bitreg(ins, ENC28J60_MISTAT, &mistat);
        } while(status == 0 && mistat & ENC28J60_MISTAT_BUSY);
    }
    if(status == 0) status = enc28j60_clear8bitreg(ins, ENC28J60_MICMD, ENC28J60_MICMD_MIIRD);
    if(status == 0) status = enc28j60_read16bitreg(ins, ENC28J60_MIRDL, content);
    // uint8_t a1, a2;
    // enc28j60_read8bitreg(ins, ENC28J60_MIRDL, &a1);
    // enc28j60_read8bitreg(ins, ENC28J60_MIRDH, &a2);
    // *content = a1 | (a2 << 8);
    return status;
}

int enc28j60_writephyreg(enc28j60 *ins, uint8_t reg, uint16_t content) {
    int status = enc28j60_write8bitreg(ins, ENC28J60_MIREGADR, reg);
    if(status == 0) status = enc28j60_write16bitreg(ins, ENC28J60_MIWRL, content);
    if(status == 0) {
        uint8_t mistat;
        do {
            status = enc28j60_read8bitreg(ins, ENC28J60_MISTAT, &mistat);
        } while(status == 0 && mistat & ENC28J60_MISTAT_BUSY);
    }
    return status;
}

int enc28j60_readrevision(enc28j60 *ins, uint8_t *rev) {
    int err = enc28j60_read8bitreg(ins, ENC28J60_EREVID, rev);
    if(err == 0 && rev) {
        // On success and when rev is not null
        if(*rev == 6) {
            // EREVID=6 is B7. Other than that, it is correct (B1=1, B4=4, B5=5)
            rev[0]++;
        }
    }
    return err;
}

int enc28j60_softreset(enc28j60 *ins) {
    // See errata 19
    int status = 0;
    do {
        // Clear PWRSV and wait for the device's power regulator to stabilize 
        // before issuing an system reset command
        status = enc28j60_clear8bitreg(ins, ENC28J60_ECON2, ENC28J60_ECON2_PWRSV);
        if(status == 0) {
            // Wait at least 300 us (we do 1ms)
#if ENC28J60_TIMER_TICK_PER_SECOND < 1000
            status = enc28j60_delay(ins, 1);
#else
            status = enc28j60_delay(ins, ENC28J60_TIMER_TICK_PER_SECOND * 1 / 1000);
#endif
        }
        if(status == 0) {
            // Issue SOFT RESET instruction
            char tx[] = {ENC28J60_OP_SOFT_RESET, ENC28J60_OP_SOFT_RESET};
            status = enc28j60_blockingio(ins, tx, NULL, 2);
        }
        if(status == 0) {
            // See errata 2. Wait 1000 us instead of checking ESTAT.CLKRDY bit.
#if ENC28J60_TIMER_TICK_PER_SECOND < 1000
            status = enc28j60_delay(ins, 1);
#else
            status = enc28j60_delay(ins, ENC28J60_TIMER_TICK_PER_SECOND * 1 / 1000);
#endif
        }
        if(status == 0) {
            uint8_t estat;
            status = enc28j60_read8bitreg(ins, ENC28J60_ESTAT, &estat);
            if(status == 0) {
                if((estat & (1<<0)) == 1 && (estat & (1<<3)) == 0) {
                    // Make sure bit 0 is set and bit 3 is clear
                } else {
                    continue;
                }
            }
        }
        break;
    } while(true);
    return status;
}


int enc28j60_enablemulticast(enc28j60 *ins, bool enable) {
    if(enable) {
        return enc28j60_set8bitreg(ins, ENC28J60_ERXFCON, ENC28J60_ERXFCON_MCEN);
    } else {
        return enc28j60_clear8bitreg(ins, ENC28J60_ERXFCON, ENC28J60_ERXFCON_MCEN);
    }
}

int enc28j60_enablebroadcast(enc28j60 *ins, bool enable) {
    if(enable) {
        return enc28j60_set8bitreg(ins, ENC28J60_ERXFCON, ENC28J60_ERXFCON_BCEN);
    } else {
        return enc28j60_clear8bitreg(ins, ENC28J60_ERXFCON, ENC28J60_ERXFCON_BCEN);
    }
}

int enc28j60_discardcrcerrors(enc28j60 *ins, bool discard) {
	if(discard) {
		return enc28j60_set8bitreg(ins, ENC28J60_ERXFCON, ENC28J60_ERXFCON_CRCEN);
	}
	return enc28j60_clear8bitreg(ins, ENC28J60_ERXFCON, ENC28J60_ERXFCON_CRCEN);
}

int enc28j60_init(enc28j60 *ins, enc28j60_init_struct *init) {
    int status = enc28j60_init_bank(ins);
    if(status != 0) return status;

	status = enc28j60_init_mac(ins, init->mac_address);
    if(status != 0) return status;

	status = enc28j60_write8bitreg(ins, ENC28J60_MACON1, 
		((init->is_full_duplex) ? ENC28J60_MACON1_TXPAUS | ENC28J60_MACON1_RXPAUS : 0) 
        | ENC28J60_MACON1_MARXEN);
    if(status != 0) return status;

	status = enc28j60_write8bitreg(ins, ENC28J60_MACON3, 
		((init->is_full_duplex) ? ENC28J60_MACON3_FULDPX : 0)
	); // enable CRC generation
    if(status != 0) return status;

    status = enc28j60_discardcrcerrors(ins, init->discard_crc_errors);
    if(status != 0) return status;

    if(init->is_full_duplex) {
        status = enc28j60_set8bitreg(ins, ENC28J60_MACON4, ENC28J60_MACON4_DEFER);
        if(status != 0) return status;

        // set inter-frame gap (non-back-to-back)
        status = enc28j60_write16bitreg(ins, ENC28J60_MAIPGL, 0x0012);
        if(status != 0) return status;

        // set inter-frame gap (back-to-back)
        status = enc28j60_write8bitreg(ins, ENC28J60_MABBIPG, 0x15);
        if(status != 0) return status;

        status = enc28j60_writephyreg(ins, ENC28J60_PHCON1, ENC28J60_PHCON1_PDPXMD);
        if(status != 0) return status;

        status = enc28j60_writephyreg(ins, ENC28J60_PHCON2, 0);
        if(status != 0) return status;
    } else {
        // set inter-frame gap (non-back-to-back)
        status = enc28j60_write16bitreg(ins, ENC28J60_MAIPGL, 0x0C12);
        if(status != 0) return status;

        // set inter-frame gap (back-to-back)
        status = enc28j60_write8bitreg(ins, ENC28J60_MABBIPG, 0x12);
        if(status != 0) return status;

        status = enc28j60_writephyreg(ins, ENC28J60_PHCON1, 0);
        if(status != 0) return status;

        // Errata #9 fix
        status = enc28j60_writephyreg(ins, ENC28J60_PHCON2, ENC28J60_PHCON2_HDLDIS);
        if(status != 0) return status;
    }


	status = enc28j60_write16bitreg(ins, ENC28J60_MAMXFLL, ENC28J60_MAX_FRAMELEN_ETH);
    if(status != 0) return status;

    return 0;
}

int enc28j60_init_bank(enc28j60 *ins) {
    // Read ECON1 register and save it
    return enc28j60_read8bitreg(ins, ENC28J60_ECON1, &ins->bank);
}

int enc28j60_init_txbuffer(enc28j60 *ins, uint16_t start, uint16_t length) {
    // Pointing to the last byte, length includes first control byte
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
    uint16_t end = (start + length) & 0x1fff;
#else
    uint16_t end = (start + length -1) & 0x1fff;
#endif
	int status = enc28j60_write16bitreg(ins, ENC28J60_ETXSTL, start);
	if(status == 0) {
	    status = enc28j60_write16bitreg(ins, ENC28J60_ETXNDL, end);
	}
	return status;
}

int enc28j60_init_rxbuffer(enc28j60 *ins, uint16_t start, uint16_t end) {
	int status = enc28j60_write16bitreg(ins, ENC28J60_ERXSTL, start);
	if(status == 0) {
        ins->erxst = start;
		ins->cur_rx_frame = start;
	    status = enc28j60_write16bitreg(ins, ENC28J60_ERXNDL, end);
	}
	if(status == 0) {
        ins->erxnd = end;
		status = enc28j60_writereadpointer(ins, start);
	}
    if(status == 0) {
        status = enc28j60_write16bitreg(ins, ENC28J60_ERXRDPTL, start);
    }
	return status;
}

int enc28j60_init_mac(enc28j60 *ins, char mac_addr[6]) {
    // Make sure we are in the correct bank
    int status = enc28j60_prepareforaccess(ins, ENC28J60_MAADR5);
    if(status) {
        return status;
    }

    status = enc28j60_write8bitreg(ins, ENC28J60_MAADR6, mac_addr[5]);
    if(status == 0) status = enc28j60_write8bitreg(ins, ENC28J60_MAADR5, mac_addr[4]);
    if(status == 0) status = enc28j60_write8bitreg(ins, ENC28J60_MAADR4, mac_addr[3]);
    if(status == 0) status = enc28j60_write8bitreg(ins, ENC28J60_MAADR3, mac_addr[2]);
    if(status == 0) status = enc28j60_write8bitreg(ins, ENC28J60_MAADR2, mac_addr[1]);
    if(status == 0) status = enc28j60_write8bitreg(ins, ENC28J60_MAADR1, mac_addr[0]);
    return status;

    // Construct SPI frame in a way to fill the registers in the correct order
    // char tx[] = {ENC28J60_OP_WRITE_CTRL_REG | (ENC28J60_MAADR5 & 0x1f), 
    //     mac_addr[4], mac_addr[5],
    //     mac_addr[2], mac_addr[3],
    //     mac_addr[0], mac_addr[1]
    // };
    // return enc28j60_blockingio(ins, tx, NULL, 7);
}

int enc28j60_writewritepointer(enc28j60 *ins, uint16_t ptr) {
	return enc28j60_write16bitreg(ins, ENC28J60_EWRPTL, ptr);
}

int enc28j60_readwritepointer(enc28j60 *ins, uint16_t *ptr) {
	return enc28j60_read16bitreg(ins, ENC28J60_EWRPTL, ptr);
}

int enc28j60_writereadpointer(enc28j60 *ins, uint16_t ptr) {
	return enc28j60_write16bitreg(ins, ENC28J60_ERDPTL, ptr);
}

int enc28j60_readreadpointer(enc28j60 *ins, uint16_t *ptr) {
	return enc28j60_read16bitreg(ins, ENC28J60_ERDPTL, ptr);
}

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
static char static_header[] = {ENC28J60_OP_READ_BUF_MEM};
#endif

static void enc28j60_readwriteframe_callback(
	enc28j60 *ins, 
	int status 
) {
    if(ENC28J60_ACTION_IS_RX(ins->current_action)) {
        // Called enc28j60_readframe() or enc28j60_read()
        if(ENC28J60_ACTION_IS_REG(ins->current_action)) {
            if(status == 0) {
                ins->current_action = ENC28J60_ACTION_RX |
                        (ins->current_action & ENC28J60_ACTION_SET_RX_POINTER);
                ins->retry_count++;
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
                enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, NULL, (char *) ins->frame, ins->length, enc28j60_readwriteframe_callback);
#else
                enc28j60_header *frame = (enc28j60_header *) ins->frame;
                frame->opcode = ENC28J60_OP_READ_BUF_MEM;
                enc28j60_spi_transfer(ins, (char *) frame, (char *) ins->frame, ins->length, enc28j60_readwriteframe_callback);
#endif
            } else if(enc28j60_spi_error_handler(ins, status, ins->retry_count_reg)) {
                ins->retry_count_reg++;
                enc28j60_o16asynctry(ins, 
                    ENC28J60_OP_WRITE_CTRL_REG, ENC28J60_ERDPTL, 
                    ins->current_pointer, enc28j60_readwriteframe_callback);
            } else {
                ins->callback(ins, status);
            }
        } else {
            if(status == 0) {
                // If successfully transfered data from enc28j60 to ram, finish
                if(ins->current_action & ENC28J60_ACTION_SET_RX_POINTER) {
                    ins->cur_rx_frame = ins->next_rx_frame;
                }
                ins->callback(ins, status);
            } else if(enc28j60_spi_error_handler(ins, status, ins->retry_count)) {
                ins->retry_count_reg = 1;
                ins->current_action = ENC28J60_ACTION_RX_REG |
                        (ins->current_action & ENC28J60_ACTION_SET_RX_POINTER);
                enc28j60_o16asynctry(ins, 
                    ENC28J60_OP_WRITE_CTRL_REG, ENC28J60_ERDPTL, 
                    ins->current_pointer, enc28j60_readwriteframe_callback);
            } else {
                ins->callback(ins, status);
            }
        }
        #if 0
        if(status == 0) {
            // If successfully transfered data from enc28j60 to ram, finish
            if(ins->current_action & ENC28J60_ACTION_SET_RX_POINTER) {
                ins->cur_rx_frame = ins->next_rx_frame;
            }
            ins->callback(ins, status);
        } else if(enc28j60_spi_error_handler(ins, status, ins->retry_count)) {
            // Repairable error
            // Skip the 6 byte header
            status = enc28j60_writereadpointer(ins, ins->current_pointer);

            if(status != 0) {
                // Unrepairable error while writing read-pointer, finish
                ins->callback(ins, status);
            } else {
                // Else read-pointer was written succesfully, retry reading
                ins->retry_count++;

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
                enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, NULL, (char *) ins->frame, ins->length, enc28j60_readwriteframe_callback);
#else
                enc28j60_header *frame = (enc28j60_header *) ins->frame;
                frame->opcode = ENC28J60_OP_READ_BUF_MEM;
                enc28j60_spi_transfer(ins, (char *) frame, (char *) ins->frame, ins->length, enc28j60_readwriteframe_callback);
#endif
            }

        } else {
            // Non-repeairable error, just call callback function
            ins->callback(ins, status);
        }
        #endif
    } else {
        if(ENC28J60_ACTION_IS_REG(ins->current_action)) {
            if(status == 0) {
                ins->current_action = ENC28J60_ACTION_TX;
                ins->retry_count++;
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
                enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, (char *) ins->frame, NULL, ins->length, enc28j60_readwriteframe_callback);
#else
                enc28j60_header *frame = (enc28j60_header *) ins->frame;
                frame->opcode = ENC28J60_OP_WRITE_BUF_MEM;
                enc28j60_spi_transfer(ins, (char *) frame, NULL, ins->length, enc28j60_readwriteframe_callback);
#endif
            } else if(enc28j60_spi_error_handler(ins, status, ins->retry_count_reg)) {
                ins->retry_count_reg++;
                enc28j60_o16asynctry(ins, 
                    ENC28J60_OP_WRITE_CTRL_REG, ENC28J60_EWRPTL, 
                    ins->current_pointer, enc28j60_readwriteframe_callback);
            } else {
                ins->callback(ins, status);
            }
        } else {
            if(status == 0) {
                // If successfully transfered data from ram to enc28j60, finish
                ins->callback(ins, status);
            } else if(enc28j60_spi_error_handler(ins, status, ins->retry_count)) {
                ins->retry_count_reg = 1;
                ins->current_action = ENC28J60_ACTION_TX_REG;
                enc28j60_o16asynctry(ins, 
                    ENC28J60_OP_WRITE_CTRL_REG, ENC28J60_EWRPTL, 
                    ins->current_pointer, enc28j60_readwriteframe_callback);
            } else {
                ins->callback(ins, status);
            }

        }
        #if 0
        // Called enc28j60_writeframe()
        if(status == 0) {
            // If successfully transfered data from ram to enc28j60, finish
            ins->callback(ins, status);
        } else if(enc28j60_spi_error_handler(ins, status, ins->retry_count)) {
            // Repairable error
            // Skip first control byte
            status = enc28j60_writewritepointer(ins, ins->current_pointer);
            if(status != 0) {
                // Unrepairable error while writing read-pointer, finish
                ins->callback(ins, status);
            } else {
                // Else read-pointer was written succesfully, retry reading
                ins->retry_count++;

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
                enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, (char *) frame, NULL, length, enc28j60_readwriteframe_callback);
#else
                enc28j60_header *frame = (enc28j60_header *) ins->frame;
                frame->opcode = ENC28J60_OP_WRITE_BUF_MEM;
                enc28j60_spi_transfer(ins, (char *) frame, NULL, ins->length, enc28j60_readwriteframe_callback);
#endif
            }
        } else {
            // Non-repeairable error, just call callback function
            ins->callback(ins, status);
        }
        #endif

    }

}

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
int enc28j60_write(enc28j60 *ins, 
        uint16_t start_ptr, char *frame, uint16_t length, 
        enc28j60_spi_callback on_finish)
#else
int enc28j60_write(enc28j60 *ins, 
        uint16_t start_ptr, enc28j60_header *frame, uint16_t length, 
        enc28j60_spi_callback on_finish)
#endif
{
    ins->length = length;
    ins->frame = (char *) frame;
    ins->retry_count = 1;
    ins->callback = on_finish;
    ins->current_action = ENC28J60_ACTION_TX;
    ins->current_pointer = start_ptr;

    int status = enc28j60_writewritepointer(ins, start_ptr);
    if(status != 0) {
        // Unrepairable error
        if(on_finish) {
            on_finish(ins, status);
            return 0;
        }
        return status;
    }

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
    static_header[0] = ENC28J60_OP_WRITE_BUF_MEM;
#else
    frame->opcode = ENC28J60_OP_WRITE_BUF_MEM;
#endif
    if(on_finish) {
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
        enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, (char *) frame, NULL, length, enc28j60_readwriteframe_callback);
#else
        enc28j60_spi_transfer(ins, (char *) frame, NULL, length, enc28j60_readwriteframe_callback);
#endif
        return 0;
    }

    do {
        // Else blocking request
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
        status = enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, (char *) frame, NULL, length, NULL);
#else
        status = enc28j60_spi_transfer(ins, (char *) frame, NULL, length, NULL);
#endif
        if(status == 0) {
            break;
        } else if(enc28j60_spi_error_handler(ins, status, ins->retry_count)) {
            // Repairable error
            // continue;
        } else {
            // Unrepairable error
            break;
        }

        // Skip first control byte
        status = enc28j60_writewritepointer(ins, start_ptr+1);
        if(status != 0) {
            // Unrepariable error
            break;
        }
    } while(true);
    return status;
}


#ifdef ENC28J60_HAS_ADVANCED_DRIVER
int enc28j60_writeframe(enc28j60 *ins, 
    uint8_t control, uint16_t start_ptr, char *frame, uint16_t length, 
    enc28j60_spi_callback on_finish)
#else
int enc28j60_writeframe(enc28j60 *ins, 
    uint8_t control, uint16_t start_ptr, enc28j60_header *frame, uint16_t length, 
    enc28j60_spi_callback on_finish)
#endif
{
    int status = 0;
    unsigned retry = 1;
    do {
        status = enc28j60_writewritepointer(ins, start_ptr);
        if(status != 0) {
            // Unrepairable error
            break;
        }

        // write control flag at the beginning of the ethernet frame - zero means use register MACON* settings
        char tx[] = {ENC28J60_OP_WRITE_BUF_MEM, control}; 
        status = enc28j60_spi_transfer(ins, tx, NULL, 2, NULL);
        if(status == 0) {
            // Success
            break;
        } else if(enc28j60_spi_error_handler(ins, status, retry) == false) {
            // Unrepairable error
            break;
        }
        // Repairable error
        retry++;
    } while(true);

    if(status != 0) {
        // Unrepairable error
        if(on_finish) {
            on_finish(ins, status);
            return 0;
        }
        return status;
    }

    ins->length = length;
    ins->frame = (char *) frame;
    ins->retry_count = 1;
    ins->callback = on_finish;
    ins->current_action = ENC28J60_ACTION_TX;
    ins->current_pointer = start_ptr+1;

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
    static_header[0] = ENC28J60_OP_WRITE_BUF_MEM;
#else
    frame->opcode = ENC28J60_OP_WRITE_BUF_MEM;
#endif
    if(on_finish) {
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
        enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, 
            (char *) frame, NULL, length, enc28j60_readwriteframe_callback);
#else
        enc28j60_spi_transfer(ins, (char *) frame, NULL, length, 
            enc28j60_readwriteframe_callback);
#endif
        return 0;
    }

    do {
        // Else blocking request
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
        status = enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, 
            (char *) frame, NULL, length, NULL);
#else
        status = enc28j60_spi_transfer(ins, (char *) frame, NULL, length, NULL);
#endif
        if(status == 0) {
            break;
        } else if(enc28j60_spi_error_handler(ins, status, ins->retry_count)) {
            // Repairable error
            // continue;
        } else {
            // Unrepairable error
            break;
        }

        // Skip first control byte
        status = enc28j60_writewritepointer(ins, start_ptr+1);
        if(status != 0) {
            // Unrepariable error
            break;
        }
    } while(true);
    return status;
}

int enc28j60_istx(enc28j60 *ins, bool *status) {
    uint8_t val = 0;
    // int err_code = enc28j60_read8bitreg(ins, ENC28J60_ECON1, &val);
    int err_code = enc28j60_read8bitreg(ins, ENC28J60_EIR, &val);
    if(err_code == 0 && status) {
        *status = !(val & (ENC28J60_EIR_TXIF | ENC28J60_EIR_TXERIF));
    }
    return err_code;
}

int enc28j60_jointx(enc28j60 *ins) {
    /// TODO: When there is not enough current during transmission, the enc28j60
    /// bugs and ECON1.TXRTS remainins set infinitely. Make sure to add some
    /// protection against that, by for example time out.
    // enc28j60_tick cur_ticks, goal_ticks;
    // int status = enc28j60_getticks(ins, &cur_ticks);
    bool istx = true;

    // if(status != 0) {
    //     // Fail on unrepairable error
    //     return status;
    // }

    // goal_ticks = cur_ticks + timeout;

    int status = 0;
    do {
        status = enc28j60_istx(ins, &istx);
        // if(status == 0) {
        //     status = enc28j60_getticks(ins, &cur_ticks);
        //     if(status == 0 && ENC28J60_CALC_REMAINING_TICKS(goal_ticks, cur_ticks) <= 0) {
        //         status = ENC28J60_ERROR_TX_TIMED_OUT;
        //         break;
        //     }
        // }
        // Break on unrepairable error or when transmission finishes
    } while(status == 0 && istx == true);
    return status;
}

int enc28j60_cleartxflags(enc28j60 *ins) {
    // Clear flags
    return enc28j60_clear8bitreg(ins, ENC28J60_EIR, ENC28J60_EIR_TXIF | ENC28J60_EIR_TXERIF);
}

int enc28j60_transmitframe(enc28j60 *ins) {
    // Start transmission
    return enc28j60_set8bitreg(ins, ENC28J60_ECON1, ENC28J60_ECON1_TXRTS);
}

int enc28j60_aftertx(enc28j60 *ins) {
    // Errata 12 check and reset
    uint8_t eir;
    int status = enc28j60_read8bitreg(ins, ENC28J60_EIR, &eir);
    if(status == 0 && eir & ENC28J60_EIR_TXERIF) {
        // If TXERIF flag is set, reset the transciever
        status = enc28j60_set8bitreg(ins, ENC28J60_ECON1, ENC28J60_ECON1_TXRST);
        if(status == 0) {
            status = enc28j60_clear8bitreg(ins, ENC28J60_ECON1, ENC28J60_ECON1_TXRST);
        }
        if(status == 0) {
            // The TXERIF flag may get set after clearing the TXRST bit, so
            // clear TXEIRF
            status = enc28j60_clear8bitreg(ins, ENC28J60_EIR, ENC28J60_EIR_TXERIF);
        }
    }
    return status;
}

void enc28j60_decrementretries(unsigned *remaining_tries) {
    if(remaining_tries != NULL) {
        remaining_tries[0]--;
    }
}

bool enc28j60_shouldretransmit(enc28j60_txstatus *status, unsigned remaining_tries) {
    if(status->status & ENC28J60_TXSTATUS_LATE_COLLISION) {
        return remaining_tries != 0;
    }
    return false;
}

int enc28j60_readtxstatus(enc28j60 *ins, uint16_t tx_start, uint16_t tx_length, 
        enc28j60_txstatus *status) {
    char buffer[8];
    int err;
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
    uint16_t tx_end = (tx_start + tx_length + 1) & 0x1fff;
    err = enc28j60_read(ins, tx_end, &buffer[1], 7, NULL);
#else
    uint16_t tx_end = (tx_start + tx_length) & 0x1fff;
    err = enc28j60_read(ins, tx_end, (enc28j60_header *) buffer, 8, NULL);
#endif

    if(err == 0 && status) {
        status->transmit_byte_count = buffer[1] | ((uint16_t) buffer[2] << 8);
        status->status = buffer[3] | (buffer[4] << 8) | (buffer[7] << 16);
        status->total_bytes_transmitted = buffer[5] | ((uint16_t) buffer[6] << 8);
    }
    return err;
}


int enc28j60_transmitframeblocking(enc28j60 *ins,
		uint16_t start_ptr, uint16_t length) {
    unsigned remaining_retries = 16;
    enc28j60_txstatus status;
    int err = 0;
    do {
        err = enc28j60_cleartxflags(ins);
        if(err == 0)
            err = enc28j60_transmitframe(ins);
        if(err == 0)
            err = enc28j60_jointx(ins);
        if(err == 0)
            err = enc28j60_aftertx(ins);
        if(err == 0)
            err = enc28j60_readtxstatus(ins, start_ptr, length, &status);
        enc28j60_decrementretries(&remaining_retries);
    } while(err == 0 && enc28j60_shouldretransmit(&status, remaining_retries));
    if(err) {
        return err;
    }
    if(remaining_retries == 0 && status.status & ENC28J60_TXSTATUS_LATE_COLLISION) {
        return ENC28J60_ERROR_TOO_MANY_COLLISIONS;
    }
    return 0;
}

/**
 * @brief Performs read from RX buffer.
 * 
 * NOTE: The packet 'rx' must be one byte longer than needed (and thus 'length'
 * must be incremented by 1 as well) since the first byte will contain opcode.
 * 
 * @param ins Instance of enc28j60.
 * @param rx Buffer where data will be received. Note that after reading you
 * should ignore first byte, as it will contain 0xff.
 * @param length How many bytes you wish to read including the first byte.
 * @return int Status or zero on success.
 */
int enc28j60_read_tryframe(enc28j60 *ins, char *rx, uint16_t length) {
    rx[0] = ENC28J60_OP_READ_BUF_MEM;
    return enc28j60_spi_transfer(ins, rx, rx, length, NULL);
}

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
int enc28j60_read_try(enc28j60 *ins, char *rx, uint16_t length, enc28j60_spi_callback on_finish) 
#else
int enc28j60_read_try(enc28j60 *ins, enc28j60_header *frame, uint16_t length, enc28j60_spi_callback on_finish) 
#endif
{
    #ifdef ENC28J60_HAS_ADVANCED_DRIVER
    enc28j60_spi_transfer_join(ins);
    static_header[0] = ENC28J60_OP_READ_BUF_MEM;
    return enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, NULL, rx, length, on_finish);
    #else
    frame->opcode = ENC28J60_OP_READ_BUF_MEM;
    return enc28j60_spi_transfer(ins, (char *) frame, (char *) frame, length, on_finish);
    #endif
}

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
int enc28j60_read(enc28j60 *ins, uint16_t ptr, char *frame, uint16_t length, enc28j60_spi_callback on_finish) 
#else
int enc28j60_read(enc28j60 *ins, uint16_t ptr, enc28j60_header *frame, uint16_t length, enc28j60_spi_callback on_finish) 
#endif
{
    ins->frame = (char*) frame;
    ins->length = length;
    ins->callback = on_finish;
    ins->current_action = ENC28J60_ACTION_RX;
    ins->current_pointer = ptr;
    ins->retry_count = 1;

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
    // use 'static char static_header[]'
    static_header[0] = ENC28J60_OP_READ_BUF_MEM;
#else
    frame->opcode = ENC28J60_OP_READ_BUF_MEM;
#endif
    if(on_finish) {
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
        enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, NULL, (char *) frame, length, enc28j60_readwriteframe_callback);
#else
        enc28j60_spi_transfer(ins, (char *) frame, (char *) frame, length, enc28j60_readwriteframe_callback);
#endif
        return 0;
    }

    int status = 0;
    unsigned retry = 1;
    do {
        // Make sure we are starting from the correct pointer
        status = enc28j60_writereadpointer(ins, ptr);
        if((status)) {
            // Unrepairable error
            break;
        }

        // Read a header that is before every ethernet frame

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
        status = enc28j60_read_try(ins, frame, length, NULL);
#else
        status = enc28j60_read_try(ins, frame, length, NULL);
#endif
        if(status == 0) {
            // Success
            break;
        } else if(enc28j60_spi_error_handler(ins, status, retry) == false) {
            // Unrepairable error
            break;
        }
        // Repairable error
        retry++;
    } while(true);

    return status;
}

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
int enc28j60_readframe(enc28j60 *ins, char *frame, uint16_t max_length, enc28j60_spi_callback on_finish)
#else
int enc28j60_readframe(enc28j60 *ins, enc28j60_header *frame, uint16_t max_length, enc28j60_spi_callback on_finish)
#endif
{
    char header[7] = {0};

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
    int status = enc28j60_read(ins, ins->cur_rx_frame, &header[1], 6, NULL);
#else
    int status = enc28j60_read(ins, ins->cur_rx_frame, (enc28j60_header *) header, 7, NULL);
#endif

    if((status)) {
        // Unrepairable error
        if(on_finish) {
            on_finish(ins, status);
            return 0;
        }
        return status;
    }

    // Convert the read header into next frame pointer, length and status
    ins->next_rx_frame = header[1] | (header[2] << 8);
    uint16_t length = (header[3] | (header[4] << 8));
    uint16_t rxstatus = header[5] | (header[6] << 8);

#ifndef ENC28J60_HAS_ADVANCED_DRIVER
    length = length + 1;
#endif
    if(length > max_length) {
        rxstatus |= ENC28J60_RXSTATUS_TRIMMED;
        length = max_length;
    }
    ins->length = length;
    ins->frame = (char *) frame;
    ins->rxstatus = rxstatus;
    ins->retry_count = 1;
    ins->callback = on_finish;
    ins->current_action = ENC28J60_ACTION_RX | ENC28J60_ACTION_SET_RX_POINTER;
    ins->current_pointer = ins->cur_rx_frame+6;

    // Get ready for reception
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
    // use 'static char static_header[]'
    static_header[0] = ENC28J60_OP_READ_BUF_MEM;
#else
    frame->opcode = ENC28J60_OP_READ_BUF_MEM;
#endif
    if(on_finish) {
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
        enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, NULL, (char *) frame, length, enc28j60_readwriteframe_callback);
#else
        enc28j60_spi_transfer(ins, (char *) frame, (char *) frame, length, enc28j60_readwriteframe_callback);
#endif
        return 0;
    }

    do {
        // TODO repeat in case of error
        if(status == 0) {
#ifdef ENC28J60_HAS_ADVANCED_DRIVER
            status = enc28j60_spi_advanced_transfer(ins, static_header, NULL, 1, NULL, (char *) frame, length, NULL);
#else
            status = enc28j60_spi_transfer(ins, (char *) frame, (char *) frame, length, NULL);
#endif
        }
        if(status == 0) {
            // If successfully transfered data from enc28j60 to ram, finish
            ins->cur_rx_frame = ins->next_rx_frame;
            break;
        } else if(!enc28j60_spi_error_handler(ins, status, ins->retry_count)) {
            // If error is not repairable, exit
            break;
        }
        ins->retry_count++;
        // Skip the six byte header
        status = enc28j60_writereadpointer(ins, ins->current_pointer);
        if(status != 0) {
            // Unrepairable error
            break;
        }
    } while(true);
    return status;
}

int enc28j60_finishreadframe(enc28j60 *ins) {
    int status = enc28j60_advancerxreadpointer(ins);
    if(status == 0) {
        // Set ECON.PKTDEC to 1 so that EPKTCNT is decremented by one
        status = enc28j60_set8bitreg(ins, ENC28J60_ECON2, ENC28J60_ECON2_PKTDEC);
    }
    return status;
}

int enc28j60_readrxframescount(enc28j60 *ins, uint8_t *cnt) {
    return enc28j60_read8bitreg(ins, ENC28J60_EPKTCNT, cnt);
}

int enc28j60_advancerxreadpointer(enc28j60 *ins) {
    // Errata 14 - write odd pointer to ERXRDPTL:ERXRDPTH register
    uint16_t ptr = ins->cur_rx_frame;
    if(ptr == ins->erxst) {
        ptr = ins->erxnd;
    } else {
        ptr = ptr - 1;
    }
    // debugu(ptr);
    // debugm(" ptr vs ");
    // debugu(ins->cur_rx_frame);
    // debugm(", erxst ");
    // debugu(ins->erxst);
    // debugm(", erxnd ");
    // debugu(ins->erxnd);
    // debugm(" ptr\r\n");
	return enc28j60_write16bitreg(ins, ENC28J60_ERXRDPTL, ptr);
}

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
int enc28j60_receiveframeblocking(enc28j60 *ins, 
        char *frame, uint16_t max_length, enc28j60_tick timeout) 
#else
int enc28j60_receiveframeblocking(enc28j60 *ins, 
        enc28j60_header *frame, uint16_t max_length, enc28j60_tick timeout)
#endif
{
    int status;
    if(timeout == 0) {
        while(1) {
            // Check if we have received any frame yet
            uint8_t cnt;
            status = enc28j60_readrxframescount(ins, &cnt);
            if(status != 0) {
                return status;
            }
            if(cnt != 0) {
                // If yes, break
                break;
            }
        }
    } else {
        // Get current time
        enc28j60_tick current_time;
        status = enc28j60_getticks(ins, &current_time);
        if(status != 0) {
            return status;
        }
        // to calculate our goal time.
        enc28j60_tick goal_time = current_time + timeout;

        while(1) {
            // Check if we have received any frame yet
            uint8_t cnt;
            status = enc28j60_readrxframescount(ins, &cnt);
            if(status != 0) {
                return status;
            }
            if(cnt != 0) {
                // If yes, break
                break;
            }

            // Otherwise check for time out
            status = enc28j60_getticks(ins, &current_time);
            if(status != 0) {
                return status;
            }
            if(ENC28J60_CALC_REMAINING_TICKS(goal_time, current_time) <= 0) {
                // and we timed out...
                return ENC28J60_ERROR_RX_TIMED_OUT;
            }
        }
    }
    // By this point we have a frame in the buffer

    // read that frame from enc28j60's memory
    status = enc28j60_readframe(ins, frame, max_length, NULL);

    // do cleanup
    if(status == 0) {
        status = enc28j60_finishreadframe(ins);
    }

    // and return
    return status;
}


int enc28j60_enablerx(enc28j60 *ins) {
	return enc28j60_set8bitreg(ins, ENC28J60_ECON1, ENC28J60_ECON1_RXEN);
}

int enc28j60_disablerx(enc28j60 *ins) {
    int status = enc28j60_clear8bitreg(ins, ENC28J60_ECON1, ENC28J60_ECON1_RXEN);
    if(status == 0) {
        uint8_t estat;
        do {
            status = enc28j60_read8bitreg(ins, ENC28J60_ESTAT, &estat);
        } while(status == 0 && estat & ENC28J60_ESTAT_RXBUSY);
    }
    return status;
}

