/*
 * UART over CAN tunnel implementation
 *
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2025 Antti Lehikoinen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef UART_OVER_CAN_H
#define UART_OVER_CAN_H

#include <stdint.h>
#include <cstddef>
#include "canhardware.h"

// Configuration constants
#define CAN_BAUDRATE             CanHardware::Baud500  // 500kbps
#define CAN_ID_TX                0x700         // STM32 -> ESP32
#define CAN_ID_RX                0x701         // ESP32 -> STM32
#define UART_TX_CHUNK_MAX        7             // Max 7 bytes per CAN frame
#define RX_BUFFER_SIZE           512           // RX buffer size
#define INTERFRAME_DELAY_US      1000          // 1ms delay between frames

class UartOverCan : public CanCallback
{
public:
    UartOverCan(CanHardware* hw);
    ~UartOverCan();

    // Initialize the module
    void Init();

    // Send UART data over CAN
    void SendUartData(const uint8_t* data, size_t len);

    // Get received UART data
    int GetUartData(uint8_t* buffer, size_t max_len);

    // Check if data is available
    bool IsDataAvailable() const;

    // CanCallback interface
    virtual bool HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc);
    virtual void HandleClear();

private:
    CanHardware* canHardware;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    size_t rx_write_pos;
    size_t rx_read_pos;
    uint8_t expect_seq;
    bool initialized;

    // Internal functions
    void send_chunk(const uint8_t* data, uint8_t len, bool is_last);
    void reset_reassembly();
    void delay_us(uint32_t us);
};

#endif // UART_OVER_CAN_H
