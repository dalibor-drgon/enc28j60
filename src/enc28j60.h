// Copyright (c) 2020 Dalibor Drgon <dalibor.drgon@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)
/**
 * @file enc28j60.h
 * @author Dalibor Drgon <dalibor.drgon@gmail.com>
 * @brief Part of enc28j60 driver
 * @version 1.0
 * @date 2020-05-01
 * 
 * @copyright Copyright (c) 2020 Dalibor Drgon <dalibor.drgon@gmail.com>
 * This code is licensed under MIT license (see LICENSE.txt for details)
 */

#ifndef __ENC28J60_H
#define __ENC28J60_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENC28J60_ERRATA_INFO


/************************ enc28j60 registers start ***************************/

// Shared registers
#define ENC28J60_EIE              0x1B
#define ENC28J60_EIR              0x1C
#define ENC28J60_ESTAT            0x1D
#define ENC28J60_ECON2            0x1E
#define ENC28J60_ECON1            0x1F

// Bank #0
#define ENC28J60_ERDPTL           (0x00|0x00)
#define ENC28J60_ERDPTH           (0x01|0x00)
#define ENC28J60_EWRPTL           (0x02|0x00)
#define ENC28J60_EWRPTH           (0x03|0x00)
#define ENC28J60_ETXSTL           (0x04|0x00)
#define ENC28J60_ETXSTH           (0x05|0x00)
#define ENC28J60_ETXNDL           (0x06|0x00)
#define ENC28J60_ETXNDH           (0x07|0x00)
#define ENC28J60_ERXSTL           (0x08|0x00)
#define ENC28J60_ERXSTH           (0x09|0x00)
#define ENC28J60_ERXNDL           (0x0A|0x00)
#define ENC28J60_ERXNDH           (0x0B|0x00)
#define ENC28J60_ERXRDPTL         (0x0C|0x00)
#define ENC28J60_ERXRDPTH         (0x0D|0x00)
#define ENC28J60_ERXWRPTL         (0x0E|0x00)
#define ENC28J60_ERXWRPTH         (0x0F|0x00)
#define ENC28J60_EDMASTL          (0x10|0x00)
#define ENC28J60_EDMASTH          (0x11|0x00)
#define ENC28J60_EDMANDL          (0x12|0x00)
#define ENC28J60_EDMANDH          (0x13|0x00)
#define ENC28J60_EDMADSTL         (0x14|0x00)
#define ENC28J60_EDMADSTH         (0x15|0x00)
#define ENC28J60_EDMACSL          (0x16|0x00)
#define ENC28J60_EDMACSH          (0x17|0x00)

// Bank #1
#define ENC28J60_EHT0             (0x00|0x20)
#define ENC28J60_EHT1             (0x01|0x20)
#define ENC28J60_EHT2             (0x02|0x20)
#define ENC28J60_EHT3             (0x03|0x20)
#define ENC28J60_EHT4             (0x04|0x20)
#define ENC28J60_EHT5             (0x05|0x20)
#define ENC28J60_EHT6             (0x06|0x20)
#define ENC28J60_EHT7             (0x07|0x20)
#define ENC28J60_EPMM0            (0x08|0x20)
#define ENC28J60_EPMM1            (0x09|0x20)
#define ENC28J60_EPMM2            (0x0A|0x20)
#define ENC28J60_EPMM3            (0x0B|0x20)
#define ENC28J60_EPMM4            (0x0C|0x20)
#define ENC28J60_EPMM5            (0x0D|0x20)
#define ENC28J60_EPMM6            (0x0E|0x20)
#define ENC28J60_EPMM7            (0x0F|0x20)
#define ENC28J60_EPMCSL           (0x10|0x20)
#define ENC28J60_EPMCSH           (0x11|0x20)
#define ENC28J60_EPMOL            (0x14|0x20)
#define ENC28J60_EPMOH            (0x15|0x20)
#define ENC28J60_EWOLIE           (0x16|0x20)
#define ENC28J60_EWOLIR           (0x17|0x20)
#define ENC28J60_ERXFCON          (0x18|0x20)
#define ENC28J60_EPKTCNT          (0x19|0x20)

// Bank #2
#define ENC28J60_MACON1           (0x00|0x40|0x80)
#define ENC28J60_MACON2           (0x01|0x40|0x80)
#define ENC28J60_MACON3           (0x02|0x40|0x80)
#define ENC28J60_MACON4           (0x03|0x40|0x80)
#define ENC28J60_MABBIPG          (0x04|0x40|0x80)
#define ENC28J60_MAIPGL           (0x06|0x40|0x80)
#define ENC28J60_MAIPGH           (0x07|0x40|0x80)
#define ENC28J60_MACLCON1         (0x08|0x40|0x80)
#define ENC28J60_MACLCON2         (0x09|0x40|0x80)
#define ENC28J60_MAMXFLL          (0x0A|0x40|0x80)
#define ENC28J60_MAMXFLH          (0x0B|0x40|0x80)
#define ENC28J60_MAPHSUP          (0x0D|0x40|0x80)
#define ENC28J60_MICON            (0x11|0x40|0x80)
#define ENC28J60_MICMD            (0x12|0x40|0x80)
#define ENC28J60_MIREGADR         (0x14|0x40|0x80)
#define ENC28J60_MIWRL            (0x16|0x40|0x80)
#define ENC28J60_MIWRH            (0x17|0x40|0x80)
#define ENC28J60_MIRDL            (0x18|0x40|0x80)
#define ENC28J60_MIRDH            (0x19|0x40|0x80)

// Bank #3
#define ENC28J60_MAADR5           (0x00|0x60|0x80)
#define ENC28J60_MAADR6           (0x01|0x60|0x80)
#define ENC28J60_MAADR3           (0x02|0x60|0x80)
#define ENC28J60_MAADR4           (0x03|0x60|0x80)
#define ENC28J60_MAADR1           (0x04|0x60|0x80)
#define ENC28J60_MAADR2           (0x05|0x60|0x80)
#define ENC28J60_EBSTSD           (0x06|0x60)
#define ENC28J60_EBSTCON          (0x07|0x60)
#define ENC28J60_EBSTCSL          (0x08|0x60)
#define ENC28J60_EBSTCSH          (0x09|0x60)
#define ENC28J60_MISTAT           (0x0A|0x60|0x80)
#define ENC28J60_EREVID           (0x12|0x60)
#define ENC28J60_ECOCON           (0x15|0x60)
#define ENC28J60_EFLOCON          (0x17|0x60)
#define ENC28J60_EPAUSL           (0x18|0x60)
#define ENC28J60_EPAUSH           (0x19|0x60)

// PHY registers
#define ENC28J60_PHCON1           0x00
#define ENC28J60_PHSTAT1          0x01
#define ENC28J60_PHID1            0x02
#define ENC28J60_PHID2            0x03
#define ENC28J60_PHCON2           0x10
#define ENC28J60_PHSTAT2          0x11
#define ENC28J60_PHIE             0x12
#define ENC28J60_PHIR             0x13
#define ENC28J60_PHLCON           0x14

/************************ enc28j60 register flags start **********************/

// enc28j60 EIE Register flags
#define ENC28J60_EIE_INTIE        0x80
#define ENC28J60_EIE_PKTIE        0x40
#define ENC28J60_EIE_DMAIE        0x20
#define ENC28J60_EIE_LINKIE       0x10
#define ENC28J60_EIE_TXIE         0x08
#define ENC28J60_EIE_WOLIE        0x04
#define ENC28J60_EIE_TXERIE       0x02
#define ENC28J60_EIE_RXERIE       0x01

// enc28j60 EIR Register flags
#define ENC28J60_EIR_PKTIF        0x40
#define ENC28J60_EIR_DMAIF        0x20
#define ENC28J60_EIR_LINKIF       0x10
#define ENC28J60_EIR_TXIF         0x08
#define ENC28J60_EIR_WOLIF        0x04
#define ENC28J60_EIR_TXERIF       0x02
#define ENC28J60_EIR_RXERIF       0x01

// enc28j60 ESTAT Register flags
#define ENC28J60_ESTAT_INT        0x80
#define ENC28J60_ESTAT_LATECOL    0x10
#define ENC28J60_ESTAT_RXBUSY     0x04
#define ENC28J60_ESTAT_TXABRT     0x02
#define ENC28J60_ESTAT_CLKRDY     0x01

// enc28j60 ECON2 Register flags
#define ENC28J60_ECON2_AUTOINC    0x80
#define ENC28J60_ECON2_PKTDEC     0x40
#define ENC28J60_ECON2_PWRSV      0x20
#define ENC28J60_ECON2_VRPS       0x08

// enc28j60 ECON1 Register flags
#define ENC28J60_ECON1_TXRST      0x80
#define ENC28J60_ECON1_RXRST      0x40
#define ENC28J60_ECON1_DMAST      0x20
#define ENC28J60_ECON1_CSUMEN     0x10
#define ENC28J60_ECON1_TXRTS      0x08
#define ENC28J60_ECON1_RXEN       0x04
#define ENC28J60_ECON1_BSEL1      0x02
#define ENC28J60_ECON1_BSEL0      0x01

// enc28j60 ERXFCON Register flags
#define ENC28J60_ERXFCON_UCEN     0x80
#define ENC28J60_ERXFCON_ANDOR    0x40
#define ENC28J60_ERXFCON_CRCEN    0x20
#define ENC28J60_ERXFCON_PMEN     0x10
#define ENC28J60_ERXFCON_MPEN     0x08
#define ENC28J60_ERXFCON_HTEN     0x04
#define ENC28J60_ERXFCON_MCEN     0x02
#define ENC28J60_ERXFCON_BCEN     0x01

// enc28j60 MACON1 Register flags
#define ENC28J60_MACON1_LOOPBK    0x10
#define ENC28J60_MACON1_TXPAUS    0x08
#define ENC28J60_MACON1_RXPAUS    0x04
#define ENC28J60_MACON1_PASSALL   0x02
#define ENC28J60_MACON1_MARXEN    0x01

// enc28j60 MACON3 Register flags
#define ENC28J60_MACON3_PADCFG2   0x80
#define ENC28J60_MACON3_PADCFG1   0x40
#define ENC28J60_MACON3_PADCFG0   0x20
#define ENC28J60_MACON3_TXCRCEN   0x10
#define ENC28J60_MACON3_PHDRLEN   0x08
#define ENC28J60_MACON3_HFRMLEN   0x04
#define ENC28J60_MACON3_FRMLNEN   0x02
#define ENC28J60_MACON3_FULDPX    0x01

// EMC28J60 MACON4 Register flags
#define ENC28J60_MACON4_DEFER	  0x40
#define ENC28J60_MACON4_BPEN	  0x20
#define ENC28J60_MACON4_NOBKOFF	  0x10

// enc28j60 MICMD Register flags
#define ENC28J60_MICMD_MIISCAN    0x02
#define ENC28J60_MICMD_MIIRD      0x01

// enc28j60 MISTAT Register flags
#define ENC28J60_MISTAT_NVALID    0x04
#define ENC28J60_MISTAT_SCAN      0x02
#define ENC28J60_MISTAT_BUSY      0x01

// enc28j60 PHY PHCON1 Register flags
#define ENC28J60_PHCON1_PRST      0x8000
#define ENC28J60_PHCON1_PLOOPBK   0x4000
#define ENC28J60_PHCON1_PPWRSV    0x0800
#define ENC28J60_PHCON1_PDPXMD    0x0100

// enc28j60 PHY PHSTAT1 Register flags
#define ENC28J60_PHSTAT1_PFDPX    0x1000
#define ENC28J60_PHSTAT1_PHDPX    0x0800
#define ENC28J60_PHSTAT1_LLSTAT   0x0004
#define ENC28J60_PHSTAT1_JBSTAT   0x0002

// enc28j60 PHY PHCON2 Register flags
#define ENC28J60_PHCON2_FRCLINK   0x4000
#define ENC28J60_PHCON2_TXDIS     0x2000
#define ENC28J60_PHCON2_JABBER    0x0400
#define ENC28J60_PHCON2_HDLDIS    0x0100

// enc28j60 PHY PHSTAT2 Register flags
#define ENC28J60_PHSTAT2_TXSTAT   0x2000
#define ENC28J60_PHSTAT2_RXSTAT   0x1000
#define ENC28J60_PHSTAT2_COLSTAT  0x0800
#define ENC28J60_PHSTAT2_LSTAT    0x0400
#define ENC28J60_PHSTAT2_DPXSTAT  0x0200
#define ENC28J60_PHSTAT2_PLRITY   0x0020

/************************ enc28j60 rxstatus flags start **********************/

// enc28j60 RX Buffer Status (enc28_60.rxstatus after enc28j60_readframe())
#define ENC28J60_RXSTATUS_LONGDROP_EVENT       (1<<0)
#define ENC28J60_RXSTATUS_CARRIER_EVENT        (1<<2)
#define ENC28J60_RXSTATUS_CRC_ERROR            (1<<4)
#define ENC28J60_RXSTATUS_LENGTH_CHECK_ERROR   (1<<5)
#define ENC28J60_RXSTATUS_LENGTH_OUT_OF_RANGE  (1<<6)
#define ENC28J60_RXSTATUS_RECEIVE_OK           (1<<7)

#define ENC28J60_RXSTATUS_RECEIVE_MULTICAST    (1<<8)
#define ENC28J60_RXSTATUS_RECEIVE_BROADCAST    (1<<9)
#define ENC28J60_RXSTATUS_DRIBBLE_NIBBLE       (1<<10)
#define ENC28J60_RXSTATUS_RECEIVE_CONTROL_FRAME       (1<<11)
#define ENC28J60_RXSTATUS_RECEIVE_PAUSE_CONTROL_FRAME (1<<12)
#define ENC28J60_RXSTATUS_RECEIVE_UNKNOWN_OPCODE      (1<<13)
#define ENC28J60_RXSTATUS_RECEIVE_VLAN_TYPE    (1<<14)
// Custom code added to indicate that the frame length exceeded max_length and thus got trimmed. 
#define ENC28J60_RXSTATUS_TRIMMED              (1<<15)

/************************ enc28j60 rxstatus flags start **********************/

// enc28j60 TX Buffer Status (shifted bit positions, stored in
// enc28j60_txstatus.status)
#define ENC28J60_TXSTATUS_GET_COLLISION_COUNT(status) (status & 0xf)
#define ENC28J60_TXSTATUS_CRC_ERROR            (1<<4)
#define ENC28J60_TXSTATUS_LENGTH_CHECK_ERROR   (1<<5)
#define ENC28J60_TXSTATUS_LENGTH_OUT_OF_RANGE  (1<<6)
#define ENC28J60_TXSTATUS_DONE                 (1<<7)
#define ENC28J60_TXSTATUS_MULTICAST            (1<<8)
#define ENC28J60_TXSTATUS_BROADCAST            (1<<9)
#define ENC28J60_TXSTATUS_PACKET_DEFER         (1<<10)
#define ENC28J60_TXSTATUS_EXCESSIVE_DEFER      (1<<11)
#define ENC28J60_TXSTATUS_EXCESSIVE_COLLISION  (1<<12)
#define ENC28J60_TXSTATUS_LATE_COLLISION       (1<<13)
#define ENC28J60_TXSTATUS_GIANT                (1<<14)
#define ENC28J60_TXSTATUS_UNDERRUN             (1<<15)

#define ENC28J60_TXSTATUS_CONTROL_FRAME        (1<<16)
#define ENC28J60_TXSTATUS_PAUSE_CONTROL_FRAME  (1<<17)
#define ENC28J60_TXSTATUS_BACKPRESSURE_APPLIED (1<<18)
#define ENC28J60_TXSTATUS_VLAN_TAGGED_FRAME    (1<<19)

// enc28j60 TX Packet Control Byte flags
#define ENC28J60_PKTCTRL_PHUGEEN   0x08
#define ENC28J60_PKTCTRL_PPADEN    0x04
#define ENC28J60_PKTCTRL_PCRCEN    0x02
#define ENC28J60_PKTCTRL_POVERRIDE 0x01

// SPI operation codes
#define ENC28J60_OP_READ_CTRL_REG       0x00
#define ENC28J60_OP_READ_BUF_MEM        0x3A
#define ENC28J60_OP_WRITE_CTRL_REG      0x40
#define ENC28J60_OP_WRITE_BUF_MEM       0x7A
#define ENC28J60_OP_BIT_FIELD_SET       0x80
#define ENC28J60_OP_BIT_FIELD_CLR       0xA0
#define ENC28J60_OP_SOFT_RESET          0xFF

/************************ enc28j60 macros start ******************************/

/// Max frame length which the conroller will accept.
/// Both for RX and TX. This means that we can store up to 5 full frames.
#define ENC28J60_MAX_FRAMELEN 1500
/// Maximum frame length including ethernet header and checksum.
#define ENC28J60_MAX_FRAMELEN_ETH 1518

/// Size of the enc28j60's memory.
#define ENC28J60_MEMORY_SIZE 0x2000


/************************ enc28j60 structs start *****************************/

struct enc28j60;
typedef struct enc28j60 enc28j60;

/**
 * @brief Structure that has to prefix any variable length data when not using
 * advanced SPI driver (that is when ENC28J60_HAS_ADVANCED_DRIVER is not
 * defined).
 * 
 * The correct allocation and usage is as follows:
 * ```c
 * enc28j60 eth1;
 * //...
 * static char data[ENC28J60_MAX_FRAME_LENGTH_ETH + sizeof(enc28j60_header)];
 * enc28j60_receiveframeblocking(&eth1, data, sizeof(data));
 * printf("Received frame of length %u\n", eth1.length);
 * char *frame = &data[sizeof(enc28j60_header)];
 * printf("First byte is %02X, the last is %02X\n", frame[0], frame[eth1.length-1]);
 * ```
 */
typedef struct __attribute__((__packed__)) enc28j60_header {
	/// Opcode
	uint8_t opcode;
} enc28j60_header;

/**
 * @brief Ethernet header
 */
typedef struct __attribute__((__packed__)) enc28j60_ethernet_header {
	char dest[6];
	char src[6];
	uint16_t type;
	char data[0];
} enc28j60_ethernet_header;

/**
 * Structure that contains data parsed from status footer appended by enc28j60
 * after frame that was transmitted or which transmission was aborted.
 */
typedef struct enc28j60_txstatus {
	/// Status vector. Use ENC28J60_TXSTATUS_* flags for check
	uint32_t status;
	/// Total bytes transmitted on the wire for the current packet, including
	/// all bytes from collided attempts.
	uint16_t transmit_byte_count;
	/// Total bytes in frame not counting collided bytes.
	uint16_t total_bytes_transmitted;
} enc28j60_txstatus;

/// Callback used internally and externally.
/// Accepts pointer to enc28j60 and error code indicating result (0 means
/// success, anything else is treated as error).
typedef void (*enc28j60_spi_callback)(
	enc28j60 *instance, 
	int custom_error_code
);

/**
 * Structure used for initialization of the enc28j60.
 */
typedef struct enc28j60_init_struct {
	/// Six-bytes-long mac address
	char mac_address[6];
	/// Support full duplex?
	bool is_full_duplex;
	/// Discard frames with invalid CRC?
	bool discard_crc_errors;
} enc28j60_init_struct;

/************************ enc28j60 current action start **********************/

#define ENC28J60_ACTION_TX (0b00)
#define ENC28J60_ACTION_TX_REG (0b10)
#define ENC28J60_ACTION_RX (0b01)
#define ENC28J60_ACTION_RX_REG (0b11)
#define ENC28J60_ACTION_IS_RX(action) ((action) & 0b01)
#define ENC28J60_ACTION_IS_REG(action) ((action) & 0b10)
#define ENC28J60_ACTION_SET_RX_POINTER (0b100)

#ifdef __DOXYGEN__
#include "enc28j60-struct.example.h"
#else
#include "enc28j60-struct.h"
#endif

/************************ enc28j60 functions start ***************************/

/**
 * @brief Performs IO on enc28j60. Internally, this calls
 * enc28j60_spi_transfer() until it succeedes or fails with unrepairable error.
 * Because of this, the request has to be repeatable (has to be read/write to
 * register, cannot be read/write to buffer memory as the read and write
 * pointers get internally incremented) in case the transfer fails and is
 * repeated. 
 * 
 * @param ins Enc28j60 pointer
 * @param tx Non-null pointer to buffer to be transmitted
 * @param rx Pointer where the received data will be saved (can be null)
 * @param length Length of the `tx` and `rx` buffers.
 * @return int Zero on success or non-zero error in case of unrepairable error.
 */
int enc28j60_blockingio(enc28j60 *ins, const char *tx, char *rx, unsigned length);

/**
 * @brief Ensures that correct bank is currently selected.
 * 
 * @param ins Enc28j60 pointer
 * @param bank Register bank to be selected
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_ensurebank(enc28j60 *ins, uint8_t bank);

/**
 * @brief Performs given IO operation on given 8-bit register until it succeeds
 * or fails with unrepairable error.
 * 
 * @param ins Enc28j60 pointer
 * @param op Operation to be performed
 * @param reg Register on which the operation will be performed
 * @param val Value to be sent
 * @param out If not null, read value will be written here.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_io8(enc28j60 *ins, uint8_t op, uint8_t reg, uint8_t val, uint8_t *out);

/**
 * @brief Performs given IO operation on given register 16bit until it succeeds
 * or fails with unrepairable error.
 * 
 * @param ins Enc28j60 pointer
 * @param op Operation to be performed
 * @param reg Register on which the operation will be performed
 * @param val Value to be sent
 * @param out If not null, read value will be written here.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_io16(enc28j60 *ins, uint8_t op, uint8_t reg, uint16_t val, uint16_t *out);

#if 0
/**
 * @brief Performs given output operation on given 8-bit register one time and
 * either returns the error code (if callback is null) or returns zero and sends
 * the error code to the callback.
 * 
 * @param ins Enc28j60 pointer
 * @param op Operation to be performed
 * @param reg Register on which the operation will be performed
 * @param val Value to be sent
 * @param on_finish Callback to be called when operation finishes (or fails). If
 * null, this operation is synchronous, otherwise it may be asynchronous if the
 * underlying SPI driver supports it.
 * @return int Zero on succes, zero when callbacak is present or non-zero error
 * code otherwise. 
 */
int enc28j60_o8asynctry(enc28j60 *ins, uint8_t op, uint8_t reg, uint8_t val, enc28j60_spi_callback on_finish);

/**
 * @brief Performs given output operation on given 16-bit register one time and
 * either returns the error code (if callback is null) or returns zero and sends
 * the error code to the callback.
 * 
 * @param ins Enc28j60 pointer
 * @param op Operation to be performed
 * @param reg Register on which the operation will be performed
 * @param val Value to be sent
 * @param on_finish Callback to be called when operation finishes (or fails). If
 * null, this operation is synchronous, otherwise it may be asynchronous if the
 * underlying SPI driver supports it.
 * @return int Zero on succes, zero when callbacak is present or non-zero error
 * code otherwise. 
 */
int enc28j60_o16asynctry(enc28j60 *ins, uint8_t op, uint8_t reg, uint16_t val, enc28j60_spi_callback on_finish);
#endif

/**
 * @brief Read value of given 8-bit register and store it in `*content` if not
 * `content` is not null.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register from which to read.
 * @param content If not null, read value will be written here.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_read8bitreg(enc28j60 *ins, uint8_t reg, uint8_t *content);

/**
 * @brief Write given value `content` into given 8-bit register.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register to be modified
 * @param content New value of the register.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_write8bitreg(enc28j60 *ins, uint8_t reg, uint8_t content);

/**
 * @brief Clears bits given by `content` from given 8-bit register.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register to be modified
 * @param content Bits to be cleared
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_clear8bitreg(enc28j60 *ins, uint8_t reg, uint8_t content);

/**
 * @brief Sets bits given by `content` in given 8-bit register.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register to be modified
 * @param content Bits to be set 
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_set8bitreg(enc28j60 *ins, uint8_t reg, uint8_t content);

/**
 * @brief Read value of given 16-bit register and store it in `*content` if not
 * `content` is not null.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register from which to read.
 * @param content If not null, read value will be written here.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_read16bitreg(enc28j60 *ins, uint8_t reg, uint16_t *content);

/**
 * @brief Write given value `content` into given 16-bit register.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register to be modified
 * @param content New value of the register.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_write16bitreg(enc28j60 *ins, uint8_t reg, uint16_t content);

/**
 * @brief Clears bits given by `content` from given 16-bit register.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register to be modified
 * @param content Bits to be cleared
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_clear16bitreg(enc28j60 *ins, uint8_t reg, uint16_t content);

/**
 * @brief Sets bits given by `content` in given 16-bit register.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register to be modified
 * @param content Bits to be set 
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_set16bitreg(enc28j60 *ins, uint8_t reg, uint16_t content);

/**
 * @brief Read from PHY register.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register to be read
 * @param content Where to write the register value
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_readphyreg(enc28j60 *ins, uint8_t reg, uint16_t *content);

/**
 * @brief Write into PHY register.
 * 
 * @param ins Enc28j60 pointer
 * @param reg Register to be written
 * @param content New value of the register
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_writephyreg(enc28j60 *ins, uint8_t reg, uint16_t content);

/**
 * @brief Reads the revision number into `*rev`.
 * 
 * @param ins Enc28j60 pointer
 * @param rev If not null, revision number will be stored here.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_readrevision(enc28j60 *ins, uint8_t *rev);

/**
 * @brief Sends soft-reset and waits until the enc28j60 is operational again.
 * 
 * Implementation compliant with errata #2 and #19.
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_softreset(enc28j60 *ins);

/**
 * @brief Initializes the enc28j60.
 * 
 * Compliant with errata #9.
 * 
 * @param ins Enc28j60 pointer
 * @param mac_addr 6 bytes long MAC address to be set.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_init(enc28j60 *ins, enc28j60_init_struct *init);

/**
 * @brief Reads the currently selected bank into `ins->bank`.
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_init_bank(enc28j60 *ins);

/**
 * @brief Initializes the MAC address.
 * 
 * @param ins Enc28j60 pointer
 * @param mac_addr 6 bytes long MAC address to be set.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_init_mac(enc28j60 *ins, char mac_addr[6]);

/**
 * @brief Writes the EWRPTL:EWRPTH registers with given value.
 * 
 * @param ins Enc28j60 pointer
 * @param ptr Value to be written into EWRPTL:EWRPTH register
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_writewritepointer(enc28j60 *ins, uint16_t ptr);

/**
 * @brief Reads the content of EWRPTL:EWRPTH register into `*ptr` if `ptr` is
 * not null.
 * 
 * @param ins Enc28j60 pointer
 * @param ptr If not null, it's where the value of EWRPTL:EWRPTH registers will 
 * be written to.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_readwritepointer(enc28j60 *ins, uint16_t *ptr);

/**
 * @brief Writes the ERDPTL:ERDPTH registers with given value.
 * 
 * @param ins Enc28j60 pointer
 * @param ptr Value to be written into ERDPTL:ERDPTH register
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_writereadpointer(enc28j60 *ins, uint16_t ptr);

/**
 * @brief Reads the content of ERDPTL:ERDPTH register into `*ptr` if `ptr` is
 * not null.
 * 
 * @param ins Enc28j60 pointer
 * @param ptr If not null, it's where the value of ERDPTL:ERDPTH registers will 
 * be written to.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_readreadpointer(enc28j60 *ins, uint16_t *ptr);

/**
 * @brief Writes the ETXSTL:ETXSTH and ETXNDL:ETXNDH pointers.
 * 
 * NOTE: The last argument is length and not a pointer to end of the frame.
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param ins Enc28j60 pointer
 * @param start Where the frame starts in buffer memory
 * @param length How long the frame is (including the opcode if not using
 * advanced SPI driver)
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_init_txbuffer(enc28j60 *ins, uint16_t start, uint16_t length);

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
/**
 * @brief Writes entire frame into enc28j60's memory.
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param ins Enc28j60 pointer
 * @param start_ptr Where the frame starts in enc28j60's memory
 * @param frame The frame to be written.
 * @param length How long the frame is
 * @param on_finish Callback to be called when the frame is written to the
 * memory. If not null and if the underlying driver supports asynchronous
 * transfer, this function will be asynchronous and this callback will get
 * called once the transfer finishes with appropriate error code. Otherwise this
 * transfer will be blocking and return the appropriate error code.
 * @return int If on_finish callback is not null, zero is returned and error
 * code is passed to `on_finish` callback. Otherwise if given callback is null,
 * zero is returned on succes and non-zero error in case of unrepairable error.
 */
int enc28j60_write(enc28j60 *ins,
		uint16_t start_ptr, char *frame, uint16_t length, 
		enc28j60_spi_callback on_finish);
#else
/**
 * @brief Writes entire frame into enc28j60's memory.
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param ins Enc28j60 pointer
 * @param start_ptr Where the frame starts in enc28j60's memory
 * @param frame The frame to be written where the first byte is overwritten and
 * used as opcode.
 * @param length How long the frame is including the opcode
 * @param on_finish Callback to be called when the frame is written to the
 * memory. If not null and if the underlying driver supports asynchronous
 * transfer, this function will be asynchronous and this callback will get
 * called once the transfer finishes with appropriate error code. Otherwise this
 * transfer will be blocking and return the appropriate error code.
 * @return int If on_finish callback is not null, zero is returned and error
 * code is passed to `on_finish` callback. Otherwise if given callback is null,
 * zero is returned on succes and non-zero error in case of unrepairable error.
 */
int enc28j60_write(enc28j60 *ins,
		uint16_t start_ptr, enc28j60_header *frame, uint16_t length, 
		enc28j60_spi_callback on_finish);
#endif

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
/**
 * @brief Writes control byte and entire frame into enc28j60's memory.
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param ins Enc28j60 pointer
 * @param control The control byte written before the frame. See
 * ENC28J60_PKTCTRL_* macros.
 * @param start_ptr Where the frame starts in enc28j60's memory
 * @param frame The frame to be written.
 * @param length How long the frame is
 * @param on_finish Callback to be called when the frame is written to the
 * memory. If not null and if the underlying driver supports asynchronous
 * transfer, this function will be asynchronous and this callback will get
 * called once the transfer finishes with appropriate error code. Otherwise this
 * transfer will be blocking and return the appropriate error code.
 * @return int If on_finish callback is not null, zero is returned and error
 * code is passed to `on_finish` callback. Otherwise if given callback is null,
 * zero is returned on succes and non-zero error in case of unrepairable error.
 */
int enc28j60_writeframe(enc28j60 *ins, uint8_t control, 
		uint16_t start_ptr, char *frame, uint16_t length, 
		enc28j60_spi_callback on_finish);
#else
/**
 * @brief Writes control byte and entire frame into enc28j60's memory.
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param ins Enc28j60 pointer
 * @param control The control byte written before the frame. See
 * ENC28J60_PKTCTRL_* macros.
 * @param start_ptr Where the frame starts in enc28j60's memory
 * @param frame The frame to be written where the first byte is overwritten and
 * used as opcode.
 * @param length How long the frame is including the opcode
 * @param on_finish Callback to be called when the frame is written to the
 * memory. If not null and if the underlying driver supports asynchronous
 * transfer, this function will be asynchronous and this callback will get
 * called once the transfer finishes with appropriate error code. Otherwise this
 * transfer will be blocking and return the appropriate error code.
 * @return int If on_finish callback is not null, zero is returned and error
 * code is passed to `on_finish` callback. Otherwise if given callback is null,
 * zero is returned on succes and non-zero error in case of unrepairable error.
 */
int enc28j60_writeframe(enc28j60 *ins, uint8_t control, 
		uint16_t start_ptr, enc28j60_header *frame, uint16_t length, 
		enc28j60_spi_callback on_finish);
#endif

/**
 * @brief Checks whenever the enc28j60 still transmits a frame and writes the
 * corresponding status into `*status`
 * 
 * The transmission state should be checked before calling
 * enc28j60_transmitframe().
 * 
 * @param ins Enc28j60 pointer
 * @param status If not null, the status (true when still transmitting) will be
 * written here.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_istx(enc28j60 *ins, bool *status);

/**
 * @brief Waits until the transmission (if any in progress) finishes and then
 * returns (or in case of unrepairable error it may return sooner).
 * 
 * Internally just uses enc28j60_istx().
 * 
 * @see enc28j60_istx()
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_jointx(enc28j60 *ins);

/**
 * @brief Clears EIR.TXIF and EIR.TXERIF flags.
 * 
 * Should be called after a frame finishes transmission or before starting new
 * transmission.
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_cleartxflags(enc28j60 *ins);

/**
 * @brief Sets the ECON1.TXRTS flag in order to start transmission.
 * 
 * Before calling this make sure that there is no ongoing tranmission already
 * taking place (by calling eithet enc28j60_istx() or enc28j60_jointx()).
 * 
 * Example usage (using advanced SPI driver. If you are not using version with
 * advanced SPI driver, simply change the type of `tx_frame` from `char*` to
 * `enc28j60_header*`):
 * 
 * ```c
 * int writeandtransmitframeblocking(uint16_t tx_start, char *tx_frame, uint16_t tx_length) {
 *    enc28j60_writeframe(&eth1, 0, tx_start, tx_frame, tx_length, NULL);
 *    enc28j60_init_txbuffer(&eth1, tx_start, tx_length);
 *    unsigned remaining_retries = 16;
 *    enc28j60_txstatus status;
 *    do {
 *        enc28j60_cleartxflags(&eth1);
 *        enc28j60_transmitframe(&eth1);
 *        enc28j60_jointx(&eth1);
 *        enc28j60_aftertx(&eth1);
 *        enc28j60_readtxstatus(&eth1, tx_start, tx_length, &status);
 *        enc28j60_decrementretries(&remaining_retries);
 *    } while(enc28j60_shouldretransmit(&status, remaining_retries));
 * }
 * ```
 * 
 * @see enc28j60_cleartxflags()
 * @see enc28j60_istx()
 * @see enc28j60_jointx()
 * @see enc28j60_readtxstatus()
 * @see enc28j60_decrementretries()
 * @see enc28j60_shouldretransmit()
 * @see enc28j60_transmitframeblocking()
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_transmitframe(enc28j60 *ins);

/**
 * @brief Should be called when transmission finishes in order to avoid bug
 * described in errata #12.
 * 
 * NOTE: Make sure to call this before enc28j60_cleartxflags(). 
 * 
 * Example usage (using advanced SPI driver. If you are not using version with
 * advanced SPI driver, simply change the type of `tx_frame` from `char*` to
 * `enc28j60_header*`):
 * 
 * ```c
 * enc28j60 eth1;
 * 
 * int transmitframeblocking(uint16_t tx_start, char *tx_frame, uint16_t tx_length) {
 *    enc28j60_writeframe(&eth1, 0, tx_start, tx_frame, tx_length, NULL);
 *    enc28j60_init_txbuffer(&eth1, tx_start, tx_length);
 *    unsigned remaining_retries = 16;
 *    enc28j60_txstatus status;
 *    do {
 *        enc28j60_cleartxflags(&eth1);
 *        enc28j60_transmitframe(&eth1);
 *        enc28j60_jointx(&eth1);
 *        enc28j60_aftertx(&eth1);
 *        enc28j60_readtxstatus(&eth1, tx_start, tx_length, &status);
 *        enc28j60_decrementretries(&remaining_retries);
 *    } while(enc28j60_shouldretransmit(&status, remaining_retries));
 * }
 * ```
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_aftertx(enc28j60 *ins);

/**
 * @brief Reads the seven-byte-long status vector written after TXed frame.
 * Useful to determine whenever ENC28J60_TXSTATUS_LATE_COLLISION happened
 * because of errata #15.
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param ins Enc28j60 pointer
 * @param tx_start Where the frame starts in enc28j60's memory
 * @param tx_length How long the frame is (including the opcode if not using
 * advanced SPI driver)
 * @param status Where the parsed data will be written to.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_readtxstatus(enc28j60 *ins, uint16_t tx_start, uint16_t tx_length, enc28j60_txstatus *status);

/**
 * @brief Decrements `*remaining_tries` (if the pointer is not null) by one.
 * 
 * Added just for better readability?
 * 
 * @param remaining_tries Pointing to variable to be decremented by one.
 */
void enc28j60_decrementretries(unsigned *remaining_tries);

/**
 * @brief Checks for late collision and returns true if late collision happened
 * and remaining_tries is not zero. Otherwise returns false.
 * 
 * This function was added so that code that is compliant with advice for errata
 * #13 can be easily written.
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param status Read status used for status checking
 * @param remaining_tries How many retries are there remaing
 * @return true When late collision happened and remaing_tries != 0
 * @return false When late collision did not happen or remaining_tries == 0
 */
bool enc28j60_shouldretransmit(enc28j60_txstatus *status, unsigned remaining_tries);

/**
 * @brief Writes an ethernet frame of given length into enc28j60's memory
 * starting at given pointer and transmits it with given control flags.
 * Compliant with errata #12, #13 and #15.
 * 
 * The functionality of this function can be summarised by the following code
 * (not including the error checks for simplicity and readability and not
 * including enc28j60 pointer but using static enc28j60 instance instead):
 * 
 * ```c
 * enc28j60 eth1;
 * 
 * int transmitframeblocking(uint16_t tx_start, uint16_t tx_length) {
 *    unsigned remaining_retries = 16;
 *    enc28j60_txstatus status;
 *    do {
 *        enc28j60_cleartxflags(&eth1);
 *        enc28j60_transmitframe(&eth1);
 *        enc28j60_jointx(&eth1);
 *        enc28j60_aftertx(&eth1);
 *        enc28j60_readtxstatus(&eth1, tx_start, tx_length, &status);
 *        enc28j60_decrementretries(&remaining_retries);
 *    } while(enc28j60_shouldretransmit(&status, remaining_retries));
 * }
 * ```
 * 
 * Example usage (without error checks):
 * 
 * ```c
 *    enc28j60_writeframe(&eth1, 0, tx_start, tx_frame, tx_frame_length, NULL);
 *    enc28j60_init_txbuffer(&eth1, tx_start, tx_frame_length);
 *    enc28j60_transmitframeblocking(&eth1, tx_start, tx_frame_length);
 * ```
 * 
 * @see enc28j60_transmitframe()
 * 
 * @param ins Enc28j60 pointer
 * @param start_ptr Where the frame starts in enc28j60's memory
 * @param length How long the frame is (including the opcode if not using
 * advanced SPI driver)
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_transmitframeblocking(enc28j60 *ins,
		uint16_t start_ptr, uint16_t length);

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
/**
 * @brief Try to read from enc28j60's memory. If callback is present and
 * underlying SPI driver supports it, this request is asynchronous. In case of
 * any error, no retrying will be done and resulting error code will be
 * immediately returned (or passed via a callback) to the user.
 * 
 * Do not forget to set read pointer by calling enc28j60_writereadpointer()
 * 
 * @see enc28j60_writereadpointer()
 * 
 * @param ins Enc28j60 pointer
 * @param frame The data will be read there.
 * @param length Length of the data to read.
 * @param on_finish Callback, optional, may be null. If not null and the
 * underlying SPI driver supports asynchronous requests, the request will be
 * asynchronous, the resulting error code will be handled over to given callback
 * and zero will be returned. Otherwise if the SPI driver does not support
 * asynchronous operation, or this callback is null, we will operate in
 * synchronous mode and resulting error code will be returned.
 * @return int Zero on succes or non-zero error in case of error.
 */
int enc28j60_read_try(enc28j60 *ins, char *frame, uint16_t length, enc28j60_spi_callback on_finish);
#else
/**
 * @brief Try to read from enc28j60's memory. If callback is present and
 * underlying SPI driver supports it, this request is asynchronous. In case of
 * any error, no retrying will be done and resulting error code will be
 * immediately returned (or passed via a callback) to the user.
 * 
 * Do not forget to set read pointer by calling enc28j60_writereadpointer()
 * 
 * @see enc28j60_writereadpointer()
 * 
 * @param ins Enc28j60 pointer
 * @param frame The data will be read there.
 * @param length Length of the data to read including opcode byte.
 * @param on_finish Callback, optional, may be null. If not null and the
 * underlying SPI driver supports asynchronous requests, the request will be
 * asynchronous, the resulting error code will be handled over to given callback
 * and zero will be returned. Otherwise if the SPI driver does not support
 * asynchronous operation, or this callback is null, we will operate in
 * synchronous mode and resulting error code will be returned.
 * @return int Zero on succes or non-zero error in case of error.
 */
int enc28j60_read_try(enc28j60 *ins, enc28j60_header *frame, uint16_t length, enc28j60_spi_callback on_finish);
#endif

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
/**
 * @brief Try to read from enc28j60's memory. If callback is present and
 * underlying SPI driver supports it, this request is asynchronous. In case of
 * any repairable error, we will retry until either succeding or failing with
 * unrepairable error.
 * 
 * @param ins Enc28j60 pointer
 * @param ptr Where to start reading from (pointer to enc28j60's memory)
 * @param frame The data will be read there.
 * @param length Length of the data to read.
 * @param on_finish Callback, optional, may be null. If not null and the
 * underlying SPI driver supports asynchronous requests, the request will be
 * asynchronous, the resulting error code will be handled over to given callback
 * and zero will be returned. Otherwise if the SPI driver does not support
 * asynchronous operation, or this callback is null, we will operate in
 * synchronous mode and resulting error code will be returned.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_read(enc28j60 *ins, uint16_t ptr, char *rx, uint16_t length, enc28j60_spi_callback on_finish);
#else
/**
 * @brief Try to read from enc28j60's memory. If callback is present and
 * underlying SPI driver supports it, this request is asynchronous. In case of
 * any repairable error, we will retry until either succeding or failing with
 * unrepairable error.
 * 
 * @param ins Enc28j60 pointer
 * @param ptr Where to start reading from (pointer to enc28j60's memory)
 * @param frame The data will be read there.
 * @param length Length of the data to read including the opcode.
 * @param on_finish Callback, optional, may be null. If not null and the
 * underlying SPI driver supports asynchronous requests, the request will be
 * asynchronous, the resulting error code will be handled over to given callback
 * and zero will be returned. Otherwise if the SPI driver does not support
 * asynchronous operation, or this callback is null, we will operate in
 * synchronous mode and resulting error code will be returned.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_read(enc28j60 *ins, uint16_t ptr, enc28j60_header *frame, uint16_t length, enc28j60_spi_callback on_finish);
#endif

/**
 * @brief Reads the number of received frames waiting to be processed in the
 * buffer memory (reds the EPKTCNT register).
 * 
 * NOTE: Use this to check whenever there are any frames in the rx buffer.
 * 
 * @param ins Enc28j60 pointer
 * @param cnt If not null, the number of packets read will be written here.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_readrxframescount(enc28j60 *ins, uint8_t *cnt);

/**
 * @brief Claims space for RX buffer.
 * 
 * NOTE: To be compliant with errata #5, make sure that the buffer starts at
 * address 0000h.
 * 
 * @param ins Enc28j60 pointer
 * @param start Start (first byte pointer) of the RX buffer.
 * @param end End (last byte pointer) of the RX buffer.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_init_rxbuffer(enc28j60 *ins, uint16_t start, uint16_t end);

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
/**
 * @brief Reads six-bytes-long header and then reads the frame of given max
 * length into memory location pointed to by `frame`. Also updates
 * enc28j60.rxstatus (see ENC28J60_RXSTATUS_* flags for testing) and fills
 * enc28j60.length with the original length.
 * 
 * Before calling, make sure that there is frame in the buffer by checking the
 * value read from enc28j60_readrxframescount(), otherwise corruption of localy
 * stored pointers will happen and garbage will be read from enc28j60's memory.
 * 
 * @see enc28j60_receiveframeblocking()
 * @see enc28j60_enablerx()
 * @see enc28j60_finishreadframe()
 * 
 * @param ins Enc28j60 pointer
 * @param frame The frame will be read there.
 * @param max_length Max length of the frame.
 * @param on_finish Callback, optional, may be null. If not null and the
 * underlying SPI driver supports asynchronous requests, the request will be
 * asynchronous, the resulting error code will be handled over to given callback
 * and zero will be returned. Otherwise if the SPI driver does not support
 * asynchronous operation, or this callback is null, we will operate in
 * synchronous mode and resulting error code will be returned.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_readframe(enc28j60 *ins, char *frame, uint16_t max_length, enc28j60_spi_callback on_finish);
#else
/**
 * @brief Reads six-bytes-long header and then reads the frame of given max
 * length into memory location pointed to by `frame`. Also updates
 * enc28j60.rxstatus (see ENC28J60_RXSTATUS_* flags for testing) and fills
 * enc28j60.length with the original length.
 * 
 * Before calling, make sure that there is frame in the buffer by checking the
 * value read from enc28j60_readrxframescount(), otherwise corruption of localy
 * stored pointers will happen and garbage will be read from enc28j60's memory.
 * 
 * @see enc28j60_receiveframeblocking()
 * @see enc28j60_enablerx()
 * @see enc28j60_finishreadframe()
 * 
 * @param ins Enc28j60 pointer
 * @param frame The frame will be read there.
 * @param max_length Max length of the frame including the opcode.
 * @param on_finish Callback, optional, may be null. If not null and the
 * underlying SPI driver supports asynchronous requests, the request will be
 * asynchronous, the resulting error code will be handled over to given callback
 * and zero will be returned. Otherwise if the SPI driver does not support
 * asynchronous operation, or this callback is null, we will operate in
 * synchronous mode and resulting error code will be returned.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_readframe(enc28j60 *ins, enc28j60_header *frame, uint16_t max_length, enc28j60_spi_callback on_finish);
#endif

/**
 * @brief Writes the ERXRDPTL:ERXRDPTH registers-pointer with appropriate value
 * to free up space in the buffer.
 * 
 * Compliant with errata #14.
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_advancerxreadpointer(enc28j60 *ins);

/**
 * @brief Writes the ERXRDPTL:ERXRDPTH registers-pointer with appropriate value
 * to free up space in the buffer and decrements EPKTCNT register by one.
 * 
 * Should be called after frame is received and read from the buffer.
 * 
 * Internally uses enc28j60_advancerxreadpointer(), compliant with errata #14.
 * 
 * @see enc28j60_receiveframeblocking()
 * @see enc28j60_enablerx()
 * @see enc28j60_readframe()
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_finishreadframe(enc28j60 *ins);

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
/**
 * @brief Wait until there is frame available in the buffer, read it into local
 * memory, do cleanup and return. If timeout is not zero, the time spent waiting
 * will be limited to given number of ticks. Also updates
 * enc28j60.rxstatus (see ENC28J60_RXSTATUS_* flags for testing) and fills
 * enc28j60.length with the original length.
 * 
 * If you don't want this functioon to block, make sure to check number of
 * received frames by calling enc28j60_readrxframescount(). If there is at least
 * one frame in the memory stored already, this function won't block.
 * 
 * The functionality with `timeout=0` is as following (not including error
 * checks): 
 * ```c
 * void receiveframeblocking(enc28j60 *ins, char *frame, uint16_t frame_len) {
 *     uint8_t cnt;
 *     do {
 *         // Loop until we have received any frame
 *         enc28j60_readrxframescount(ins, &cnt);
 *     } while(cnt == 0);
 *    
 *     enc28j60_readframe(ins, frame, frame_len, NULL);
 *     enc28j60_finishreadframe(ins); 
 * }
 * ```
 * @see enc28j60_enablerx()
 * @see enc28j60_readframe()
 * @see enc28j60_finishreadframe()
 * 
 * @param ins Enc28j60 pointer
 * @param frame Where the frame will be written to.
 * @param max_length Maximum length of the frame
 * @param timeout Timeout - how many ticks to wait at max before returning. If
 * zero, the enc28j60 will wait infinitely long for a frame.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_receiveframeblocking(enc28j60 *ins, char *frame, uint16_t max_length, enc28j60_tick timeout);
#else
/**
 * @brief Wait until there is frame available in the buffer, read it into local
 * memory, do cleanup and return. If timeout is not zero, the time spent waiting
 * will be limited to given number of ticks. Also updates
 * enc28j60.rxstatus (see ENC28J60_RXSTATUS_* flags for testing) and fills
 * enc28j60.length with the original length.
 * 
 * If you don't want this functioon to block, make sure to check number of
 * received frames by calling enc28j60_readrxframescount(). If there is at least
 * one frame in the memory stored already, this function won't block.
 * 
 * The functionality with `timeout=0` is as following (not including error
 * checks): 
 * ```c
 * void receiveframeblocking(enc28j60 *ins, char *frame, uint16_t frame_len) {
 *     uint8_t cnt;
 *     do {
 *         // Loop until we have received any frame
 *         enc28j60_readrxframescount(ins, &cnt);
 *     } while(cnt == 0);
 *    
 *     enc28j60_readframe(ins, frame, frame_len, NULL);
 *     enc28j60_finishreadframe(ins); 
 * }
 * ```
 * @see enc28j60_enablerx()
 * @see enc28j60_readframe()
 * @see enc28j60_finishreadframe()
 * 
 * @param ins Enc28j60 pointer
 * @param frame Where the frame will be written to.
 * @param max_length Maximum length of the frame including the opcode.
 * @param timeout Timeout - how many ticks to wait at max before returning. If
 * zero, the enc28j60 will wait infinitely long for a frame.
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_receiveframeblocking(enc28j60 *ins, enc28j60_header *frame, uint16_t max_length, enc28j60_tick timeout);
#endif

/**
 * @brief Enable reception of frames. 
 * Sets ECON1.RXEN bit in order to enable reception of ethernet frames.
 * 
 * @see enc28j60_receiveframeblocking()
 * @see enc28j60_readframe()
 * @see enc28j60_finishreadframe()
 * @see enc28j60_disablerx()
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_enablerx(enc28j60 *ins);

/**
 * @brief Disable reception of frames.
 * Clear ECON1.RXEN bit and waits until ESTAT.RXBUSY is clearead.
 * 
 * @see enc28j60_enablerx()
 * 
 * @param ins Enc28j60 pointer
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_disablerx(enc28j60 *ins);

/**
 * @brief Discard frames that have invalid CRC checksum present.
 * 
 * @param ins Enc28j60 pointer
 * @param discard true to discard packets with invalid CRC, false to keep them
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_discardcrcerrors(enc28j60 *ins, bool discard);

/**
 * @brief Enable reception of frames with multicast destination address
 * 
 * @param ins Enc28j60 pointer
 * @param discard true to enable reception of frames with multicast destination,
 * false to ignore them
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_enablemulticast(enc28j60 *ins, bool enable);

/**
 * @brief Enable reception of frames with broadcast destination address
 * 
 * @param ins Enc28j60 pointer
 * @param discard true to enable reception of frames with broadcast destination,
 * false to ignore them
 * @return int Zero on succes or non-zero error in case of unrepairable error.
 */
int enc28j60_enablebroadcast(enc28j60 *ins, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* __ENC28J60_H */
