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
    enc28j60_enablebroadcast(&eth0, true);
    enc28j60_enablemulticast(&eth0, true);


    // This is broadcast frame originally sent from my laptop. 
    // It is IPv4/UDP broadcast from 192.168.100.2 (to 255.255.255.255).
    //
    // send and capture any frame you want, just make sure to not include the
    // ethernet's CRC if you are using ENC28J60_PKTCTRL_PCRCEN as this arguments
    // forces the enc28j60 to generates CRC for you.
    // And note that when using enc28j60 in production (together with receiver
    // enabled), never use ENC28J60_PKTCTRL_PCRCEN as this will result in
    // receiver discarding currently received frame - always generate CRC in
    // software.
	static char tx_frame[] = {
		0, // first byte is opcode when not using advanced spi driver
        0xff ,0xff ,0xff ,0xff ,0xff ,0xff, // destination ethernet address
        0xac ,0x7b ,0xa1 ,0x85 ,0x84 ,0xd6, // source ethernet address
        0x08 ,0x00 ,0x45 ,0x00,
        0x00 ,0x47 ,0x2c ,0x23 ,0x40 ,0x00 ,0x40 ,0x11 ,0xe9 ,0xd8 ,0xc0 ,0xa8 ,0x64 ,0x02 ,0xff ,0xff,
        0xff ,0xff ,0xc2 ,0xa2 ,0x5d ,0xc0 ,0x00 ,0x33 ,0x7b ,0x41 ,0x41 ,0x42 ,0x43 ,0x44 ,0x45 ,0x46,
        0x47 ,0x48 ,0x49 ,0x4a ,0x4b ,0x4c ,0x4d ,0x4e ,0x4f ,0x50 ,0x52 ,0x53 ,0x54 ,0x41 ,0x42 ,0x43,
        0x44 ,0x45 ,0x46 ,0x47 ,0x48 ,0x49 ,0x4a ,0x4b ,0x4c ,0x4d ,0x4e ,0x4f ,0x50 ,0x52 ,0x53 ,0x54,
        0x55 ,0x58 ,0x59 ,0x5a ,0x0a
	};
    uint16_t start_ptr = 0;
    /* Write the frame into memory */
    enc28j60_writeframe(&eth0, 
        // Append CRC and make sure the frame is long at least 64 bytes
        ENC28J60_PKTCTRL_POVERRIDE | ENC28J60_PKTCTRL_PCRCEN | ENC28J60_PKTCTRL_PPADEN,
        start_ptr, (enc28j60_header *) tx_frame, sizeof(tx_frame), NULL
    );
    enc28j60_init_txbuffer(&eth0, start_ptr, sizeof(tx_frame));

    while(1) {
        printf("Transmitting...\t");
        enc28j60_transmitframeblocking(&eth0, start_ptr, sizeof(tx_frame));
        printf("done\n");
        fflush(stdout);

        // Delay 1 second and transmit again
        enc28j60_delay(&eth0, ENC28J60_TIMER_TICK_PER_SECOND);
    }
}