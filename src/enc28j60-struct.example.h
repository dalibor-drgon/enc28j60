// Copyright (c) 2020 Dalibor Drgon <dalibor.drgon@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)
/**
 * @file enc28j60-struct.example.h
 * @author Dalibor Drgon <dalibor.drgon@gmail.com>
 * @brief Part of enc28j60 driver
 * @version 1.0
 * @date 2020-05-01
 * 
 * @copyright Copyright (c) 2020 Dalibor Drgon <dalibor.drgon@gmail.com>
 * This code is licensed under MIT license (see LICENSE.txt for details)
 */

#ifndef __ENC28J60_H
#error "Don't include this file directly, only via enc28j60.h"
#endif

#ifndef __ENC28J60_STRUCT_H
#define __ENC28J60_STRUCT_H

#include <stdint.h>

/// Error code indicating that max number of retries were performed and we were
/// still unable to transmit the frame.
/// TODO: Change its value if such value for error code is already used.
#define ENC28J60_ERROR_TOO_MANY_COLLISIONS -0x900

/// Error code indicating that we timed out while waiting for a packet in
/// enc28j60_receiveframeblocking().
/// TODO: Change its value if such value for error code is already used.
#define ENC28J60_ERROR_RX_TIMED_OUT -0x901

// TODO: Define or comment this
#define ENC28J60_HAS_ADVANCED_DRIVER

/// TODO: Change it to uint32_t or uint8_t, depending on the timer resolution.
/// This data type has to cover 1 millisecond.
///
/// NOTE: The code (ENC28J60_CALC_REMAINING_TICKS macro) assumes that the timer
/// resets when it hits 0b1111...1111. If your timer does work differently, you
/// may need to modify ENC28J60_CALC_REMAINING_TICKS.
typedef uint16_t enc28j60_tick;

/// How many ticks are there per second.
///
/// NOTE: This should be at least 1 millisecond to quarantee best performance,
/// but even for less precise timers than that the functionality is preserved.
#define ENC28J60_TIMER_TICK_PER_SECOND 1000

/// Calculate how many ticks are remaining.
///
/// NOTE: The data type returned should be signed.
#define ENC28J60_CALC_REMAINING_TICKS(GOAL, CUR) ((int16_t) (GOAL) - (int16_t) (CUR))

/// Structure holding important informations about enc28j60 and about the latest
/// transmission performed.
///
/// TODO: Modify this structure so you can figure out which SPI port to use (or
/// hardcode it and keep this struct untouched, as you wish).
struct enc28j60 {
    /// Callback to be called when transfer finishes.
    enc28j60_spi_callback callback; 
    /// Used internally when asynchronous request is called and error happens to
    /// retry the request if needed.
    /// This is an argument of enc28j60_readframe or enc28j60_writeframe.
    char *frame;
    /// Used internally when asynchronous request is called and error happens to
    /// retry the request if needed.
    /// This is an argument of enc28j60_readframe or enc28j60_writeframe, or
    /// this can be less if the actual length of the received frame was less.
    unsigned length;
    /// Status read from buffer. See datasheet page 41 and 44 and
    /// ENC28J60_RXSTATUS_* macros for testing each bit.
    uint16_t rxstatus;
    /// How many retries were there before this call to spi_transfer.
    uint16_t retry_count;
    uint16_t retry_count_reg;

    /// Pointer where the currently-processed received frame starts.
    uint16_t cur_rx_frame;
    /// Pointer where the next received frame starts.
    uint16_t next_rx_frame;
    
    /// Pointer to where currently transmitted ethernet frame starts (pointing
    /// to the zero control frame)
    uint16_t current_pointer;

    /// Saved ERXSTL:ERXSTH register value
    uint16_t erxst;
    /// Saved ERXNDL:ERXNDH register value
    uint16_t erxnd;

    /// Low/Least significant 2 bits contain which bank is currently used (0-3).
    uint8_t bank;
    /// Indicates if current/previous operation is/was RX or TX
    uint8_t current_action;

    // struct spi_device *device;
    // int file_handle;
    // SPI_t *spi;
    // bool (*error_callback)(enc28j60 *instance, int error_code);
    // etc... depending on your implementation
};

#ifdef ENC28J60_HAS_ADVANCED_DRIVER
/// @brief This function performs (using DMA or blocking I/O) parallely
/// transmision from 'tx1' buffer and reception into 'rx1' buffer (if 'rx1' is
/// not null), and once this transmission is completed, performs transmission
/// from 'tx2' (if not null) into 'rx2' (if not null). If 'tx2' and 'rx2' is
/// null (that is, when 'length2' is zero), the arguments 'tx2' and 'rx2' should
/// be ignored and only repcetion from/into 'tx1'/'rx1' (if not null) should be
/// done. If the callback is null,
/// this function must block and return the error code, otherwise it may or may
/// not block and the error code should be passed to the callback (and 0
/// returned). 
///
/// NOTE: You must implement this function.
///
/// @param instance Pointer to enc28j60 struct (may contain hardware/software
/// specific fields to determine which SPI to use).
/// @param tx1 Pointer pointing to data for transmission. Will not be null.
/// @param rx1 Pointer pointing to memory where data sohuld be received. May be
/// null, in which case received data may be ignored. May also be same as 'tx'.
/// @param length1 Length of 'tx1' (and length of 'rx1' if 'rx1' is not null).
/// @param tx2 Pointer pointing to data for transmission. May be null.
/// @param rx2 Pointer pointing to memory where data sohuld be received. May be
/// null, in which case received data may be ignored. May also be same as 'tx'.
/// @param length2 Length of 'tx2' (if 'tx2' is not null) and length of 'rx2'
/// (if 'rx2' is not null). 
/// @param on_finish Callback which should be called when transfer is completed.
/// If callback is null, the function must block and return error code,
/// otherwise if callback is supplied the function may or may not block and
/// callback must be called (with error code as parameter) and zero must be
/// returned. 
///
/// @return int error code, or 0 on success (return 0 when callback is supplied)
int enc28j60_spi_advanced_transfer(
	enc28j60 *instance,
	const char* tx1, 
	char *rx1, 
	unsigned length1,
	const char* tx2, 
	char *rx2, 
	unsigned length2,
	enc28j60_spi_callback on_finish
);
#else
/// @brief This function performs (using DMA or blocking I/O) parallely
/// transmision 
/// from 'tx' buffer and reception into 'rx' buffer. If the callback is null, 
/// this function must block and return the error code, otherwise it may or may
/// not block and the error code should be passed to the callback (and 0
/// returned). 
///
/// NOTE: You must implement this function.
///
/// @param instance Pointer to enc28j60 struct (may contain hardware/software
/// specific fields to determine which SPI to use).
/// @param tx Pointer pointing to data for transmission. Will not be null.
/// @param rx Pointer pointing to memory where data sohuld be received. May be
/// null, in which case received data may be ignored. May also be same as 'tx'.
/// @param length Length of 'tx' (and length of 'rx' if 'rx' is not null).
/// @param on_finish Callback which should be called when transfer is completed.
/// If callback is null, the function must block and return error code,
/// otherwise if callback is supplied the function may or may not block and
/// callback must be called (with error code as parameter) and zero must be
/// returned. 
///
/// @return int error code, or 0 on success (return 0 when callback is supplied)
int enc28j60_spi_transfer(
	enc28j60 *instance,
	const char* tx, 
	char *rx, 
	unsigned length,
	enc28j60_spi_callback on_finish
);
#endif


/// This function should wait until SPI operation in progress (if any) finishes
/// and then return. 
///
/// NOTE: You must implement this function.
///
/// @return Error code or 0 on success.
int enc28j60_spi_transfer_join(
	enc28j60 *instance
);

/// @brief This function is called before callback when non-zero error code is
/// received from enc28j60_spi_transfer() or enc28j60_spi_advanced_transfer().
/// It tells whenever we should retry the transmission 
/// (by returning true), or simply fail and return the error code to the user
/// (by returning false).
///
/// What exactly will this function do is dependant on the user. It may as well
/// block infinitely and print error message on the screen, or always return
/// true to retry the transmission if it's safe to do so.
///
/// NOTE: You must implement this function.
///
/// @param instance Pointer to enc28j60 struct
/// @param error_code Non-zero error code received from enc28j60_spi_transfer()
/// or enc28j60_spi_advanced_transfer()
/// @param retry How many retries were there total (starting from 1).
///
/// @return bool true if tranmission should be retried, false if error should be
/// returned/passed to a callback.
bool enc28j60_spi_error_handler(
	enc28j60 *instance,
	int error_code,
	uint16_t retry
);

/// @brief Wait until given number of ticks passes.
///
/// @param ins Pointer to initialized enc28j60 struct.
/// @param ticks How many ticks this function should wait before returning.
/// @return int Error code or 0 on success.
int enc28j60_delay(enc28j60 *ins, enc28j60_tick ticks);

/// @brief Get current number of ticks.
///
/// @param ins Pointer to initialized enc28j60 struct.
/// @param ticks If not null, there should be stored the number of ticks.
/// @return int Error code or 0 on success.
int enc28j60_getticks(enc28j60 *ins, enc28j60_tick *ticks);



#endif
