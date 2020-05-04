
This is ENC28J60 driver practically platform agnostic, yet targetting mainly 
baremetal systems. Supports asynchronous writting and reading from enc28j60's 
internal buffer (fails back to synchronous transfers if the platform does 
not support asynchronous SPI IO), and is compliant with errata sheet for 
all versions. Comes with supplied examples and Doxygen documentation. 
Licensed under MIT license. To learn how to correctly use this library
to comply with the errata sheet, see below.

[Documentation](https://dalibor-drgon.github.io/enc28j60/)

[Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/39662e.pdf)

[Errata sheet](http://ww1.microchip.com/downloads/en/DeviceDoc/80349c.pdf)

# How to correctly use this library

When working with enc28j60, keep in mind following tips in order for your code
to comply with errata sheet:

- Make sure speed of your SPI is at least 8 MHz, or
- use the same 25 MHz oscillator and generate spi clock of 25/2, 25/3, 25/4 ...
  MHz (errata #1; for B1 and B4 only)
- When using `enc28j60_init_rxbuffer()`, make sure that the RX buffer starts at
  `0000h` (errata #5)
- Use `enc28j60_readrxframescount()` instead of checking for `PKTIF` flag
  (errata #6)
- Once the transmission finishes, make sure to call `enc28j60_aftertx()`,
  `enc28j60_readtxstatus()` and `enc28j60_shouldretransmit()`
  with additional code to retransmit the frame in case of late
  collision (errata #12, #14, #15; also note that `LATECOL` flag is unreliable,
  this is why it's important to call `enc28j60_readtxstatus()` and check for
  late collision there using 
  `txstatus.status & ENC28J60_TXSTATUS_LATE_COLLISION` - which is done
  inside `enc28j60_shouldretransmit()`): 
```c
    unsigned remaining_retries = 16;
    enc28j60_txstatus status;
    do {
        enc28j60_cleartxflags(&eth1);
        enc28j60_transmitframe(&eth1);
        enc28j60_jointx(&eth1); // or wait for interrupt
        enc28j60_aftertx(&eth1);
        enc28j60_readtxstatus(&eth1, tx_start, tx_length, &status);
        enc28j60_decrementretries(&remaining_retries);
    } while(enc28j60_shouldretransmit(&status, remaining_retries));
```
- When sending frames, do CRC calculation in software, as doing this by enc28j60
  will abort currently received packet. (errata #18)
- When using pattern match filters, note that extra packets not fitting the
  filter may be received. (errata #18)
- Note that enc28j60 does not support auto negotiation. That is, with most
  equipment, it can communicate in only half-duplex unless both ends are
  manually configured to work in full-duplex (Section 9.0 of datasheet)


# Errata


Module         | Feature                  | #   |  Summary                                                        |B1 |B4 |B5 |B7 | Implemented in/by
---------------|--------------------------|-----|-----------------------------------------------------------------|:-:|:-:|:-:|:-:|--------------------------------------------------------------------------------
MAC Interface  | -                        | 1.  |  MAC registers unreliable with slow asynchronous SPI clock      | X | X |   |   | *user* by setting correct SPI speed
Reset          | -                        | 2.  |  CLKRDY set early                                               | X | X | X | X | `enc28j60_softreset()`
Core           | Operating Specifications | 3.  |  Industrial (-40°C to +85°C) temperature range unsupported      | X | X |   |   | -
Oscillator     | CLKOUT pin               | 4.  |  CLKOUT unavailable in Power Save mode                          | X | X | X | X | -
Memory         | Ethernet Buffer          | 5.  |  Receive buffer must start at 0000h                             | X | X | X | X | *user* when using `enc28j60_init_rxbuffer()`
Interrupts     | -                        | 6.  |  Receive Packet Pending Interrupt Flag (PKTIF) unreliable       | X | X | X | X | *user* by using `enc28j60_readrxframescount()` instead of checking for `PKTIF`
PHY            | -                        | 7.  |  TPIN+/- automatic polarity detection and correction unreliable | X | X | X | X | *user* by avoiding incorrect RX polarity
PHY            | -                        | 8.  |  RBIAS resistor value differs between silicon revisions         | X | X |   |   | *user* when designing board
PHY            | -                        | 9.  |  Internal loopback in half-duplex unreliable                    | X | X | X | X | `enc28j60_init()`
PHY            | -                        | 10. |  Internal loopback in full-duplex unreliable                    | X | X | X | X | *user*
PHY LEDs       | -                        | 11. |  Combined Collision and Duplex Status mode unavailable          | X | X | X | X | *user*
Transmit logic | -                        | 12. |  Transmit abort may stall transmit logic                        | X | X | X | X | `enc28j60_aftertx()`
PHY            | -                        | 13. |  Received link pulses potentially cause collisions              |   |   | X | X | `enc28j60_shouldretransmit()`
Memory         | Ethernet Buffer          | 14. |  Even values in ERXRDPT may corrupt receive buffer              | X | X | X | X | `enc28j60_advancerxreadpointer()` or `enc28j60_finishreadframe()`
Transmit logic | -                        | 15. |  LATECOL Status bit unreliable                                  | X | X | X | X | `enc28j60_readtxstatus()`
PHY LEDs       | -                        | 16. |  LED auto-polarity detection unreliable                         | X | X | X | X | *user*
DMA            | -                        | 17. |  DMA checksum calculations will abort receive packets           | X | X | X | X | *user* by doing checksum in software
Receive Filter | -                        | 18. |  Pattern match filter allows reception of extra packets         | X | X | X | X | *user* by discarding unwanted packets
SPI            | -                        | 19. |  Reset command unavailable in Power Save mode                   | X | X | X | X | `enc28j60_softreset()`



# Example usage

First copy `enc28j60-struct.example.h` into `enc28j60-struct.h` and make changes
into the code, structures and macros if you wish. You have to choose whenever
your system supports advanced spi driver (that is a driver that can transceive
to/from two different buffers while keeping `SS` low all the time). In this
guide, we assume that you have advanced driver, but you can find code examples
even for the basic driver in `/examples/*_nonadv.c`.

After customizing the `enc28j60-struct.h`, implement the functions
defined there (`enc28j60_spi_transfer()` if you don't have advanced SPI driver,
`enc28j60_spi_advanced_transfer()` otherwise; `enc28j60_spi_transfer_join()`,
`enc28j60_spi_error_handler()`, `enc28j60_delay()` and `enc28j60_getticks()`).

## Initialization

Choose whenever to use full duplex (if available - otherwise half-duplex will be
used even if `is_full_duplex` is true) and whenever to discard frames with
invalid CRC and finally chose MAC address. Reset the device and initialize it
with chosen parameters.

```c
    enc28j60 eth0;
    /* Reset and initialize enc28j60 */
    enc28j60_init_struct init_struct = {
        .is_full_duplex = true,
        .discard_crc_errors = true
    };
    char mac[] = { 0x00, 0x00, 0x00, 0x01, 0x02, 0x03};
    memcpy(init_struct.mac_address, mac, 6);

    printf("Initializing\n");
    enc28j60_softreset(&eth0);
    enc28j60_init(&eth0, &init_struct);
    ...
```

Note that you can modify the `struct enc28j60` and add for example a pointer to
the SPI device you wish to use. Then the first lines will look like this:

```c
    enc28j60 eth0;
    eth0.spi_pointer = &spi1_dma;
    /* Reset and initialize enc28j60 */
    ... 
```

## Reception

It's possible to setup interrupts to handle reception of frames, however to keep
things simple we start with simple blocking architecture. First you can choose,
whenever you wish to accept broadcast and multicast packets, set where RX buffer
starts and ends, and then finally enable the receiver and wait for frames.

```c
    ...
    enc28j60_enablebroadcast(&eth0, true);
    enc28j60_enablemulticast(&eth0, true);
    // use whole memory as receive buffer
    enc28j60_init_rxbuffer(&eth0, 0, ENC28J60_MEMORY_SIZE-1);

    /* Enable receiver and start receiving packets */
    printf("Receiving...\n");
    enc28j60_enablerx(&eth0);
    static char frame[ENC28J60_MAX_FRAMELEN_ETH + sizeof(enc28j60_header)];

    while(1) {
        uint8_t cnt;
        do {
            // Loop until we have received any frame
            enc28j60_readrxframescount(&eth0, &cnt);
        } while(cnt == 0);

        // or call the following code in interrupt (EIR.PKTIF - it is reliable as source of interrupt, but unreliable when you read it from EIR register)
        enc28j60_readframe(&eth0, frame, sizeof(frame), NULL);
        enc28j60_finishreadframe(&eth0); 
        unsigned length = eth0.length;

        do_whatever_you_want_with_frame(frame, length);
    }

```

or, if you want to prototype faster, you can just use built-in function
`enc28j60_receiveframeblocking()` which either waits at most given number of
ticks, or waits infinitely if `0` is supplied as `timeout` argument until a
frame is received (if you don't want this function to block, simply check the
number of received frames yourself before calling this function):

```c
    ...
    while(1) {
        enc28j60_receiveframeblocking(&eth0, frame, sizeof(frame), 0);
        unsigned length = eth0.length;
        
        do_whatever_you_want_with_frame(frame, length);
    }
```

But note that this function blocks - if you don't want it to block, you can
simply check the amount of received frames, and if at least one was received,
the call to `enc28j60_receciveframeblocking()` won't wait, but immediately start
reading that frame from enc28j60's memory.


## Transmission

The transmission is simple as well, and the code below can be divided into two
pieces, one part of which gets executed somewhere in the application, while the
other part might get executed from interrupt.

```c
    ...
    char tx_frame[] = {...}; // ethernet frame with valid CRC at the end
    uint16_t start_ptr = ENC28J60_MEMORY_SIZE - 0x600;
    enc28j60_writeframe(&eth0, 0,
        start_ptr, tx_frame, sizeof(tx_frame), NULL
    );
    enc28j60_init_txbuffer(&eth0, start_ptr, sizeof(tx_frame));

    printf("Transmitting...\t");
    unsigned remaining_retries = 16;
    enc28j60_txstatus status;
    do {
        enc28j60_cleartxflags(&eth0);
        enc28j60_transmitframe(&eth0);
        enc28j60_jointx(&eth0); 
        // enc28j60_jointx() or run the following code from interrupt on EIR.TXIF or EIR.TXERIF flag
        enc28j60_aftertx(&eth0);
        enc28j60_readtxstatus(&eth0, start_ptr, sizeof(tx_frame), &status);
        enc28j60_decrementretries(&remaining_retries);
    } while(enc28j60_shouldretransmit(&status, remaining_retries));
    printf("done\n");
```

This code may seem non-trivial, but without the extra effort it would not
be compliant with errata #12, #13, #15, and the transceiver could freeze.
The last few lines can be however simplified by using built-in function
`enc28j60_transmitframeblocking()`:

```c
    printf("Transmitting...\t");
    enc28j60_transmitframeblocking(&eth0, start_ptr, sizeof(tx_frame));
    printf("done\n");
```

