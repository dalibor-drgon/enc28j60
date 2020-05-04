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

    /* Reset enc28j60 */
    printf("Soft reset...\n");
    enc28j60_softreset(&eth0);

    uint8_t revision;
    enc28j60_readrevision(&eth0, &revision);
    printf("Revision B%u\n", revision);
    fflush(stdout);

    while(1);

}