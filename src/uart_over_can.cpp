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

#include "uart_over_can.h"
#include "delay.h"
#include <string.h>

//Some functions use the "register" keyword which C++ doesn't like
//We can safely ignore that as we don't even use those functions
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include <libopencm3/cm3/cortex.h>
#pragma GCC diagnostic pop

UartOverCan::UartOverCan(CanHardware* hw)
    : canHardware(hw), rx_write_pos(0), rx_read_pos(0), expect_seq(0), initialized(false)
{
}

UartOverCan::~UartOverCan()
{
}

void UartOverCan::Init()
{
    if (initialized) return;

    // Register for CAN messages
    canHardware->RegisterUserMessage(CAN_ID_RX);

    // Add ourselves as a callback
    canHardware->AddCallback(this);

    reset_reassembly();
    initialized = true;
}

bool UartOverCan::HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc)
{
    if (canId != CAN_ID_RX || dlc < 2) {
        return false; // Not for us
    }

    uint8_t* d = (uint8_t*)data;
    uint8_t seq = d[0];
    uint8_t lf = d[1];
    uint8_t n = lf & 0x0F;
    bool last = (lf & 0x80) != 0;

    // Validate frame
    if (n > 7 || (2 + n) > dlc) {
        reset_reassembly();
        return true;
    }

    // Check sequence number for reassembly
    if (rx_write_pos == 0) {
        expect_seq = seq; // Establish sync on first frame
    }

    if (seq != expect_seq) {
        reset_reassembly(); // Resync on sequence mismatch
        expect_seq = seq;
    }

    expect_seq++;

    // Check if we have space for this data
    size_t buffer_used = (rx_write_pos - rx_read_pos + RX_BUFFER_SIZE) % RX_BUFFER_SIZE;
    if (buffer_used + n > RX_BUFFER_SIZE) {
        reset_reassembly(); // Overflow, reset
        return true;
    }

    // Copy data to buffer
    for (uint8_t i = 0; i < n; i++) {
        rx_buffer[rx_write_pos] = d[2 + i];
        rx_write_pos = (rx_write_pos + 1) % RX_BUFFER_SIZE;
    }

    // If this is the last frame, we're done with this message
    if (last) {
        // Message complete - data is ready for reading
    }

    return true;
}

void UartOverCan::HandleClear()
{
    // Re-register our CAN ID after clear
    if (initialized) {
        canHardware->RegisterUserMessage(CAN_ID_RX);
    }
}

void UartOverCan::SendUartData(const uint8_t* data, size_t len)
{
    if (!initialized || !data || len == 0) return;

    while (len > 0) {
        uint8_t chunk = (len > UART_TX_CHUNK_MAX) ? UART_TX_CHUNK_MAX : (uint8_t)len;
        size_t remaining = len - chunk;
        bool is_last = (remaining == 0);

        send_chunk(data, chunk, is_last);

        data += chunk;
        len -= chunk;

        // Optional inter-frame delay for busy buses
        if (remaining > 0) {
            delay_us(INTERFRAME_DELAY_US);
        }
    }
}

int UartOverCan::GetUartData(uint8_t* buffer, size_t max_len)
{
    if (!buffer || max_len == 0) return 0;

    size_t bytes_copied = 0;

    while (bytes_copied < max_len && rx_read_pos != rx_write_pos) {
        buffer[bytes_copied++] = rx_buffer[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % RX_BUFFER_SIZE;
    }

    return bytes_copied;
}

bool UartOverCan::IsDataAvailable() const
{
    return rx_read_pos != rx_write_pos;
}

void UartOverCan::send_chunk(const uint8_t* data, uint8_t len, bool is_last)
{
    static uint8_t seq = 0;
    uint32_t frame_data[2] = {0};

    uint8_t* d = (uint8_t*)frame_data;
    d[0] = seq++;  // Sequence number
    d[1] = (len & 0x0F) | (is_last ? 0x80 : 0x00);  // LEN_FLAGS

    // Copy payload data
    for (uint8_t i = 0; i < len; i++) {
        d[2 + i] = data[i];
    }

    // Send frame (blocking for now, could be improved with timeout)
    canHardware->Send(CAN_ID_TX, frame_data, 2 + len);
}

void UartOverCan::reset_reassembly()
{
    // Reset read position to match write position, effectively clearing buffer
    rx_read_pos = rx_write_pos;
    expect_seq = 0;
}

void UartOverCan::delay_us(uint32_t us)
{
    uDelay(us);
}
