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

    printf("Initializing...\n");
    enc28j60_softreset(&eth0);
    enc28j60_init(&eth0, &init_struct);
    enc28j60_enablebroadcast(&eth0, true);
    enc28j60_enablemulticast(&eth0, true);
    // use whole memory as receive buffer
    enc28j60_init_rxbuffer(&eth0, 0, ENC28J60_MEMORY_SIZE-1);

    /* Enable receiver and start receiving packets */
    printf("Receiving...\n");
    fflush(stdout);
    enc28j60_enablerx(&eth0);
    static char frame[ENC28J60_MAX_FRAMELEN_ETH];

    while(1) {
        int status = enc28j60_receiveframeblocking(&eth0, frame, sizeof(frame), 0);

        printf("Receive status %u\n", status);
        if(status == 0) {
            printf("\tFrame length %u\n", eth0.length);
            printf("\tFirst byte is %02X, the last is %02X\n", frame[0], frame[eth0.length-1]);

            enc28j60_ethernet_header *header = (enc28j60_ethernet_header *) frame;
            printf("\t     Source: %02X:%02X:%02X:%02X:%02X:%02X\n",
                header->src[0], header->src[1], header->src[2],
                header->src[3], header->src[4], header->src[5]);
            printf("\tDestination: %02X:%02X:%02X:%02X:%02X:%02X\n",
                header->dest[0], header->dest[1], header->dest[2],
                header->dest[3], header->dest[4], header->dest[5]);
        }
        fflush(stdout);
    }

}
