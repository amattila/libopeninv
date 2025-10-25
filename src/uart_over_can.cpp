#include "uart_over_can.h"
#include "canhardware.h"
#include <string.h>
#include "printf.h"

#define UART_CAN_BASE_ID 0x700
#define UART_CAN_TX_ID (UART_CAN_BASE_ID + 0)
#define UART_CAN_RX_ID (UART_CAN_BASE_ID + 1)

UartOverCan::UartOverCan(CanHardware* can)
    : m_can(can), m_lastRxTime(0), m_rxBufferPos(0), m_expectedSeq(0)
{
}

void UartOverCan::Init()
{
    m_can->AddCallback(this);
}

void UartOverCan::SendUartData(const uint8_t* data, uint32_t length)
{
    static uint8_t seq = 0;
    uint32_t sent = 0;

    while (sent < length)
    {
        uint8_t chunkSize = (length - sent) > 7 ? 7 : (uint8_t)(length - sent);
        bool isLast = (sent + chunkSize >= length);
        uint8_t canData[8] = {0};

        // SEQ + LEN_FLAGS format as per spec
        canData[0] = seq++;
        canData[1] = (chunkSize & 0x0F) | (isLast ? 0x80 : 0x00);
        memcpy(&canData[2], &data[sent], chunkSize);

        m_can->Send(UART_CAN_TX_ID, canData, 2 + chunkSize);
        sent += chunkSize;
    }
}

int UartOverCan::GetUartData(uint8_t* buffer, uint32_t maxLength)
{
    if (m_rxBufferPos == 0)
        return 0;

    uint32_t bytesToCopy = m_rxBufferPos < maxLength ? m_rxBufferPos : maxLength;
    memcpy(buffer, m_rxBuffer, bytesToCopy);

    // Shift remaining data
    if (m_rxBufferPos > bytesToCopy)
    {
        memcpy(m_rxBuffer, &m_rxBuffer[bytesToCopy], m_rxBufferPos - bytesToCopy);
    }

    m_rxBufferPos -= bytesToCopy;
    return bytesToCopy;
}

bool UartOverCan::HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc)
{
    if (canId == UART_CAN_RX_ID && dlc >= 2)
    {
        uint8_t* canData = (uint8_t*)data;
        uint8_t seq = canData[0];
        uint8_t lf = canData[1];
        uint8_t n = lf & 0x0F;
        bool last = (lf & 0x80) != 0;

        // Validate frame as per spec
        if (n > 7 || (2 + n) > dlc)
        {
            m_rxBufferPos = 0; // Reset on invalid frame
            return true;
        }

        // Resync logic: reset if unexpected sequence
        if (m_rxBufferPos == 0)
        {
            m_expectedSeq = seq;
        }
        if (seq != m_expectedSeq)
        {
            m_rxBufferPos = 0; // Resync
            m_expectedSeq = seq;
        }
        m_expectedSeq++;

        // Check buffer overflow
        if (m_rxBufferPos + n > sizeof(m_rxBuffer))
        {
            m_rxBufferPos = 0; // Reset on overflow
            return true;
        }

        // Copy data
        memcpy(&m_rxBuffer[m_rxBufferPos], &canData[2], n);
        m_rxBufferPos += n;

        // If this is the last frame, message is complete
        if (last)
        {
            // Data is ready for reading via GetUartData()
        }
    }
    return true; // Continue processing other callbacks
}

void UartOverCan::HandleClear()
{
    m_rxBufferPos = 0;
}