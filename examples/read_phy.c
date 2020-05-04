// Copyright (c) 2020 Dalibor Drgon <dalibor.drgon@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#include <enc28j60.h>

#include <stdio.h>
#include <string.h>

enc28j60 eth0;

/// If enc28j60 stores a pointer to SPI, then in this function assign it to the
/// enc28j60's struct.
///
/// For example, if you added spi descriptor to the enc28j60 struct under the
/// name `spi`, then just open the spi and assign obtained descriptor to the
/// enc28j60's struct.
///
/// ```c
/// void open_spi_for_enc28j60(enc28j60 *enc) {
///     int fd = spi_open(...);
///     enc->spi = fd;
/// }
/// ```
extern void open_spi_for_enc28j60(enc28j60 *enc);

/// Program entry point.
int program_start() {
    /* Initialize spi and assign it to the eth0 */
    open_spi_for_enc28j60(&eth0);

    /* Reset and initialize enc28j60 */
    enc28j60_init_struct init_struct = {
        .is_full_duplex = false,
        .discard_crc_errors = true
    };
    char mac[] = { 0x00, 0x00, 0x00, 0x01, 0x02, 0x03};
    memcpy(init_struct.mac_address, mac, 6);

    printf("Initializing\n");
    enc28j60_softreset(&eth0);
    enc28j60_init(&eth0, &init_struct);

    uint16_t id1, id2;
    enc28j60_readphyreg(&eth0, ENC28J60_PHID1, &id1);
    enc28j60_readphyreg(&eth0, ENC28J60_PHID2, &id2);
    printf("PHY ID %04X %04X\n", id1, id2);

    while(1) {
        uint16_t phcon1, phstat2;
        enc28j60_readphyreg(&eth0, ENC28J60_PHCON1, &phcon1);
        enc28j60_readphyreg(&eth0, ENC28J60_PHSTAT2, &phstat2);
        printf(" PHCON1: %04X\n", phcon1);
        printf("PHSTAT2: %04X\n", phstat2);
        // NOTE! enc28j60 does not support automatic duplex negotiation! This is
        // why full duplex will always be 'no' unless you have manually enabled
        // it
        printf("\tIs full duplex: %s\n", (phcon1 & ENC28J60_PHCON1_PDPXMD) ? "yes" : "no");
        printf("\t    Is link up: %s\n", (phstat2 & ENC28J60_PHSTAT2_LSTAT) ? "yes" : "no");
        fflush(stdout);

        // Delay 1 second and transmit again
        enc28j60_delay(&eth0, ENC28J60_TIMER_TICK_PER_SECOND);
    }

}