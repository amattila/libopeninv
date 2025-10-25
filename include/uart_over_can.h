#ifndef UART_OVER_CAN_H
#define UART_OVER_CAN_H

#include "canhardware.h"

class UartOverCan : public CanCallback
{
public:
    UartOverCan(CanHardware* can);
    void Init();
    void SendUartData(const uint8_t* data, uint32_t length);
    int GetUartData(uint8_t* buffer, uint32_t maxLength);

    // CanCallback interface
    bool HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc);
    void HandleClear();

private:
    CanHardware* m_can;
    uint32_t m_lastRxTime;
    uint8_t m_rxBuffer[256];
    uint32_t m_rxBufferPos;
    uint8_t m_txBuffer[256];
    uint32_t m_txBufferPos;
};

#endif // UART_OVER_CAN_H