#include "uart_over_can.h"
#include "canhardware.h"
#include <string.h>
#include "printf.h"

#define UART_CAN_BASE_ID 0x700
#define UART_CAN_TX_ID (UART_CAN_BASE_ID + 0)
#define UART_CAN_RX_ID (UART_CAN_BASE_ID + 1)

UartOverCan::UartOverCan(CanHardware* can)
    : m_can(can), m_lastRxTime(0), m_rxBufferPos(0), m_txBufferPos(0)
{
}

void UartOverCan::Init()
{
    m_can->AddCallback(this);
}

void UartOverCan::SendUartData(const uint8_t* data, uint32_t length)
{
    uint32_t sent = 0;
    while (sent < length)
    {
        uint32_t chunkSize = (length - sent) > 7 ? 7 : (length - sent);
        uint8_t canData[8] = {0};

        // First byte is sequence number, rest is data
        canData[0] = (sent / 7) & 0xFF;
        memcpy(&canData[1], &data[sent], chunkSize);

        m_can->Send(UART_CAN_TX_ID, canData, 8);
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
        // Convert uint32_t data to uint8_t array
        uint8_t* canData = (uint8_t*)data;

        // Skip sequence number, copy data
        uint32_t dataLen = dlc - 1;
        if (m_rxBufferPos + dataLen < sizeof(m_rxBuffer))
        {
            memcpy(&m_rxBuffer[m_rxBufferPos], &canData[1], dataLen);
            m_rxBufferPos += dataLen;
        }
    }
    return true; // Continue processing other callbacks
}

void UartOverCan::HandleClear()
{
    m_rxBufferPos = 0;
}