#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include <string.h>                     // For memcpy and memset
#include "definitions.h"                // SYS function prototypes
#include "AppRS485Handler.h"
#include "timers.h"

#define UART8 8
#define UART9 9

#define MODBUS_MAX_LEN         256
#define MODBUS_RX_TIMEOUT_MS   10   // Adjust as needed (baud rate dependent)

volatile uint8_t modbusRxBuffer8[MODBUS_MAX_LEN];
volatile size_t modbusRxIndex8 = 0;
volatile bool modbusFrameReady8 = false;

volatile uint8_t modbusRxBuffer9[MODBUS_MAX_LEN];
volatile size_t modbusRxIndex9 = 0;
volatile bool modbusFrameReady9 = false;

#if HEV_IO_Aggregator
    TCP_SOCKET sUart8ServerSocket = INVALID_SOCKET;
    TCP_SOCKET sUart9ServerSocket = INVALID_SOCKET;
#endif

#if Two_Wheeler_IO_Aggregator
    UDP_SOCKET sUart8ServerSocket = INVALID_SOCKET;
    UDP_SOCKET sUart9ServerSocket = INVALID_SOCKET;
#endif    
void vUart9HandlerServerTask(void *pvParameters);
void vUart8HandlerServerTask(void *pvParameters);
QueueHandle_t xUart8QueueHandler;
QueueHandle_t xUart9QueueHandler;

typedef struct
{
    uint8_t buffer[256];
    uint16_t index;
    uint16_t length;
} ModbusMessage_t;

/**
 * @brief Calculates the CRC-16 for a Modbus message.
 *
 * This function implements the CRC-16 calculation using the Modbus polynomial 0xA001.
 * The algorithm follows bitwise processing to compute the CRC.
 *
 * @param[in]  buffer Pointer to the input data buffer.
 * @param[in]  length Length of the input data buffer.
 * @return     Computed CRC-16 value.
 */
uint16_t CalculateModbusCRC(const uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFFU; /* Initial CRC value */
    uint16_t pos;
    uint8_t i;
    
    if (buffer == NULL)  /* MISRA Compliance: Check for null pointer */
    {
        return 0U;
    }

    for (pos = 0U; pos < length; pos++)
    {
        crc ^= (uint16_t)buffer[pos];

        for (i = 0U; i < 8U; i++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

/**
 * @brief Validates if the given data is a valid Modbus request.
 * @param[in] data Pointer to the data buffer.
 * @param[in] length Length of the data buffer.
 * @return true if valid, false otherwise.
 */
bool ValidateModbusRequest(const uint8_t *data, uint16_t length, uint8_t channelId) {
    if (data == NULL || length < 4) return false;

    uint16_t calculatedCRC = CalculateModbusCRC(data, length - 2);
    uint16_t receivedCRC = (data[length - 1] << 8) | data[length - 2];

    if (calculatedCRC != receivedCRC) {
        SYS_CONSOLE_PRINT("Invalid Modbus request (CH%u), discarding: ", channelId);
        for (uint16_t i = 0; i < length; i++) {
            SYS_CONSOLE_PRINT("%02X ", data[i]);
        }
        SYS_CONSOLE_PRINT("\r\n");

        if (channelId == 1) ReportError(ERR_RS485_1_INVALID_CRC);
        else if (channelId == 2) ReportError(ERR_RS485_2_INVALID_CRC);

        return false;
    }

    return true;
}

/**
 * @brief Sets RS485 transceiver to transmit mode.
 *
 * This function configures the RS485 transceiver direction to transmit
 * based on the provided UART number.
 *
 * @param[in] uartNumber The UART number (8 or 9) corresponding to the transceiver.
 */
void RS485_SetTransmitMode(uint8_t uartNumber) {
    if (uartNumber == 8) {
        RS485_EN1_Set(); // Set direction to Transmit 
    } else if (uartNumber == 9) {
        RS485_EN2_Set(); // Set direction to Transmit
    }
}

/**
 * @brief Sets RS485 transceiver to receive mode.
 *
 * This function configures the RS485 transceiver direction to receive
 * based on the provided UART number.
 *
 * @param[in] uartNumber The UART number (8 or 9) corresponding to the transceiver.
 */
void RS485_SetReceiveMode(uint8_t uartNumber) {
    if (uartNumber == 8) {
        RS485_EN1_Clear(); // Set direction to Receive
    } else if (uartNumber == 9) {
        RS485_EN2_Clear(); // Set direction to Receive
    }
}

/**
 * @brief Transmits data over RS485.
 *
 * This function transmits data using the specified UART number,
 * ensuring that the transmission is completed before switching back to receive mode.
 * It uses a semaphore to ensure exclusive access to the UART resource.
 *
 * @param[in] data Pointer to the data buffer to be transmitted.
 * @param[in] length Length of the data to be transmitted.
 */
void RS485_TransmitData_UART8(uint8_t *data, uint16_t length) {
    if (length == 0 || !ValidateModbusRequest(data, length, 1)) return;
    taskENTER_CRITICAL();
    RS485_SetTransmitMode(UART8);
    if (!SERCOM8_USART_Write(data, length))
    {
        SYS_CONSOLE_PRINT("RS485 channel 1: Write error (no response)\r\n");
        ReportError(ERR_RS485_1_NO_RESPONSE);
    }    
    while (!SERCOM8_USART_TransmitComplete());
//    RS485_SetReceiveMode(UART8);
    taskEXIT_CRITICAL();
    vTaskDelay(7);
    RS485_SetReceiveMode(UART8);
}

/**
 * @brief Transmits data over RS485.
 *
 * This function transmits data using the specified UART number,
 * ensuring that the transmission is completed before switching back to receive mode.
 * It uses a semaphore to ensure exclusive access to the UART resource.
 *
 * @param[in] data Pointer to the data buffer to be transmitted.
 * @param[in] length Length of the data to be transmitted.
 */
void RS485_TransmitData_UART9(uint8_t *data, uint16_t length) {
    if (length == 0 || !ValidateModbusRequest(data, length, 2)) return;
    taskENTER_CRITICAL();
    RS485_SetTransmitMode(UART9);
    if (!SERCOM9_USART_Write(data, length))
    {
        SYS_CONSOLE_PRINT("RS485 channel 2: Write error (no response)\r\n");
        ReportError(ERR_RS485_2_NO_RESPONSE);
    }
    while (!SERCOM9_USART_TransmitComplete());
//    RS485_SetReceiveMode(UART9);
    taskEXIT_CRITICAL();
    vTaskDelay(7);
    RS485_SetReceiveMode(UART9);    
}

/*
 * @brief   UART8 Server Task Handler
 * @details This function continuously monitors the UART8 server socket.
 *          It reads incoming data from the socket and transmits it over RS485.
 *          If the connection is lost, it attempts to reconnect.
 *
 * @param   pvParameters - Pointer to task parameters (unused in this function).
 *
 * @return  None (This function runs indefinitely in a loop).
 */

void vUart8HandlerServerTask(void *pvParameters)
{
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);

#if HEV_IO_Aggregator    
    // Move buffers out of loop to avoid repeated stack usage
    uint8_t u8UART8Buffer[128] = {0};
    ModbusMessage_t modbusMessage;
    TickType_t lastReconnectAttempt = 0;
#endif

    while (true)
    {
        #if HEV_IO_Aggregator
                // Handle data from queue (Modbus TX to client)
                if (xQueueReceive(xUart8QueueHandler, &modbusMessage, pdMS_TO_TICKS(50)) == pdPASS)
                {
                    if (TCPIP_TCP_IsConnected(sUart8ServerSocket))
                    {
                        SYS_CONSOLE_PRINT("Write %d bytes on UART8 Server\n\r", modbusMessage.length);
                        TCPIP_TCP_ArrayPut(sUart8ServerSocket, modbusMessage.buffer, modbusMessage.length);
                    }

                    // Clear message buffer safely
                    memset(modbusMessage.buffer, 0, sizeof(modbusMessage.buffer));
                    modbusMessage.index = 0U;
                    modbusMessage.length = 0U;
                }
                else
                {
                    // No queue data? Check for TCP receive
                    if (TCPIP_TCP_IsConnected(sUart8ServerSocket))
                    {
                        int16_t bytesRead = TCPIP_TCP_ArrayGet(sUart8ServerSocket, u8UART8Buffer, sizeof(u8UART8Buffer));
                        if (bytesRead > 0)
                        {
                            SYS_CONSOLE_PRINT("Got Data on RS485_1 Server Handler - Data Send to UART8\r\n");
                            RS485_TransmitData_UART8(u8UART8Buffer, bytesRead);
                        }
                        if (!TCPIP_TCP_IsConnected(sUart8ServerSocket) || TCPIP_TCP_WasDisconnected(sUart8ServerSocket)) {
                            SYS_CONSOLE_PRINT("\r\nTCP RS485_1 Connection Closed\r\n");
                            TCPIP_TCP_Close(sUart8ServerSocket);
                            sUart8ServerSocket = INVALID_SOCKET;
                        }                         
                    }
                }

                // Smart reconnect handling every 3 seconds
                if (sUart8ServerSocket == INVALID_SOCKET)
                {
                    TickType_t currentTime = xTaskGetTickCount();
                    if ((currentTime - lastReconnectAttempt) > pdMS_TO_TICKS(3000))
                    {
                        lastReconnectAttempt = currentTime;

                        sUart8ServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, writeData.rs485Ports[0], 0);
                        if (sUart8ServerSocket == INVALID_SOCKET)
                        {
                            SYS_CONSOLE_PRINT("Failed to reopen RS485_1 socket, retrying...\r\n");
                        }
                        else
                        {
                            SYS_CONSOLE_PRINT("RS485_1 socket reopened successfully\r\n");
                        }
                    }
                }

        #elif Two_Wheeler_IO_Aggregator
            int16_t availableBytes = TCPIP_UDP_GetIsReady(sUart8ServerSocket);
            if (availableBytes > 0) 
            {
                uint8_t u8Uart8Buffer[64];  // Adjust buffer size if needed
                memset(u8Uart8Buffer, 0, sizeof(u8Uart8Buffer));
                int bytesReceived = TCPIP_UDP_ArrayGet(sUart8ServerSocket, u8Uart8Buffer, sizeof(u8Uart8Buffer));
                if (bytesReceived > 0) 
                {
                    SYS_CONSOLE_PRINT("Received UDP Data (Length: %d): ", bytesReceived);
                    for (int i = 0; i < bytesReceived; i++) 
                    {
                        SYS_CONSOLE_PRINT("%02X ", u8Uart8Buffer[i]);
                    }
                    SYS_CONSOLE_PRINT("\r\n");
                    RS485_TransmitData_UART8(u8Uart8Buffer, bytesReceived);
                }
                TCPIP_UDP_Discard(sUart8ServerSocket);  // Free the RX buffer
            }                
            // Ensure reconnection logic is handled correctly
            if (sUart8ServerSocket == INVALID_SOCKET) {
                sUart8ServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, UART8_SERVER_PORT, 0);

                if (sUart8ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen RS485_1 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            static ModbusMessage_t modbusMessage; // Ensure ModbusMessage_t is defined

            if (xQueueReceive(xUart8QueueHandler, &modbusMessage, (TickType_t) 10) == pdPASS)
            {
                if (TCPIP_UDP_PutIsReady(sUart8ServerSocket) >= 0)
                {
                    SYS_CONSOLE_PRINT("Write %d bytes on UART8 UDP Server\n\r", modbusMessage.length);
                    TCPIP_UDP_ArrayPut(sUart8ServerSocket, modbusMessage.buffer, modbusMessage.length);
                    (void)TCPIP_UDP_Flush(sUart8ServerSocket);
                }

                // Clear message buffer safely
                memset(modbusMessage.buffer, 0, sizeof(modbusMessage.buffer));
                modbusMessage.index = 0U;
                modbusMessage.length = 0U;
            }             
        #endif
    }
    vTaskDelay(10);
}

/*
 * @brief   UART9 Server Task Handler
 * @details This function continuously monitors the UART9 server socket.
 *          It reads incoming data from the socket and transmits it over RS485.
 *          If the connection is lost, it attempts to reconnect.
 *
 * @param   pvParameters - Pointer to task parameters (unused in this function).
 *
 * @return  None (This function runs indefinitely in a loop).
 */
void vUart9HandlerServerTask(void *pvParameters)
{
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);
#if HEV_IO_Aggregator
    uint8_t u8UART9Buffer[128] = {0};    
    ModbusMessage_t modbusMessage;
    TickType_t lastReconnectAttempt = 0;
#endif    

    while (true)
    {
        #if HEV_IO_Aggregator
                // Handle data from queue (Modbus TX to client)
                if (xQueueReceive(xUart9QueueHandler, &modbusMessage, pdMS_TO_TICKS(50)) == pdPASS)
                {
                    if (TCPIP_TCP_IsConnected(sUart9ServerSocket))
                    {
                        SYS_CONSOLE_PRINT("Write %d bytes on UART9 Server\n\r", modbusMessage.length);
                        TCPIP_TCP_ArrayPut(sUart9ServerSocket, modbusMessage.buffer, modbusMessage.length);
                    }

                    // Clear message buffer safely
                    memset(modbusMessage.buffer, 0, sizeof(modbusMessage.buffer));
                    modbusMessage.index = 0U;
                    modbusMessage.length = 0U;
                }
                else
                {
                    // No queue data ? check for TCP receive
                    if (TCPIP_TCP_IsConnected(sUart9ServerSocket))
                    {
                        int16_t bytesRead = TCPIP_TCP_ArrayGet(sUart9ServerSocket, u8UART9Buffer, sizeof(u8UART9Buffer));
                        if (bytesRead > 0)
                        {
                            SYS_CONSOLE_PRINT("Got Data on RS485_2 Server Handler - Data Send to UART9\r\n");
                            RS485_TransmitData_UART9(u8UART9Buffer, bytesRead);
                        }
                        if (!TCPIP_TCP_IsConnected(sUart9ServerSocket) || TCPIP_TCP_WasDisconnected(sUart9ServerSocket)) {
                            SYS_CONSOLE_PRINT("\r\nTCP RS485_2 Connection Closed\r\n");
                            TCPIP_TCP_Close(sUart9ServerSocket);
                            sUart9ServerSocket = INVALID_SOCKET;
                        }                         
                    }
                }

                // Smart reconnect handling every 3 seconds
                if (sUart9ServerSocket == INVALID_SOCKET)
                {
                    TickType_t currentTime = xTaskGetTickCount();
                    if ((currentTime - lastReconnectAttempt) > pdMS_TO_TICKS(3000))
                    {
                        lastReconnectAttempt = currentTime;

                        sUart9ServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, writeData.rs485Ports[1], 0);
                        if (sUart9ServerSocket == INVALID_SOCKET)
                        {
                            SYS_CONSOLE_PRINT("Failed to reopen RS485_2 socket, retrying...\r\n");
                        }
                        else
                        {
                            SYS_CONSOLE_PRINT("RS485_2 socket reopened successfully\r\n");
                        }
                    }
                }

        #elif Two_Wheeler_IO_Aggregator
            int16_t availableBytesUART9 = TCPIP_UDP_GetIsReady(sUart9ServerSocket);
            if (availableBytesUART9 > 0) 
            {
                uint8_t u8Uart9Buffer[64];  // Adjust buffer size if needed
                memset(u8Uart9Buffer, 0, sizeof(u8Uart9Buffer));
                int bytesReceived = TCPIP_UDP_ArrayGet(sUart9ServerSocket, u8Uart9Buffer, sizeof(u8Uart9Buffer));
                if (bytesReceived > 0) 
                {
                    SYS_CONSOLE_PRINT("Received UDP Data on UART9 (Length: %d): ", bytesReceived);
                    for (int i = 0; i < bytesReceived; i++) 
                    {
                        SYS_CONSOLE_PRINT("%02X ", u8Uart9Buffer[i]);
                    }
                    SYS_CONSOLE_PRINT("\r\n");
                    RS485_TransmitData_UART9(u8Uart9Buffer, bytesReceived);
                }
                TCPIP_UDP_Discard(sUart9ServerSocket);  // Free the RX buffer
            }

            // Ensure reconnection logic is handled correctly
            if (sUart9ServerSocket == INVALID_SOCKET) {
                sUart9ServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, UART9_SERVER_PORT, 0);

                if (sUart9ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen RS485_2 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }

            static ModbusMessage_t modbusMessageUART9; // Ensure ModbusMessage_t is defined

            if (xQueueReceive(xUart9QueueHandler, &modbusMessageUART9, (TickType_t) 10) == pdPASS)
            {
                if (TCPIP_UDP_PutIsReady(sUart9ServerSocket) >= 0)
                {
                    SYS_CONSOLE_PRINT("Write %d bytes on UART9 UDP Server\n\r", modbusMessageUART9.length);
                    TCPIP_UDP_ArrayPut(sUart9ServerSocket, modbusMessageUART9.buffer, modbusMessageUART9.length);
                    (void)TCPIP_UDP_Flush(sUart9ServerSocket);
                }

                // Clear message buffer safely
                memset(modbusMessageUART9.buffer, 0, sizeof(modbusMessageUART9.buffer));
                modbusMessageUART9.index = 0U;
                modbusMessageUART9.length = 0U;
            } 
        #endif
    }
    vTaskDelay(10);
}


void SERCOM8_USART_ReadCallback(uintptr_t context) {
    uint8_t byte;
    if (!SERCOM8_USART_ReadIsBusy()) {
        if (!SERCOM8_USART_Read(&byte, 1))
        {
            SYS_CONSOLE_PRINT("RS485 channel 1: Read error (overrun or not ready)\r\n");
            ReportError(ERR_RS485_1_OVERRUN);
        }
        if (modbusRxIndex8 < MODBUS_MAX_LEN) {
            modbusRxBuffer8[modbusRxIndex8++] = byte;
        }
        TCC0_TimerStop();
        TCC0_TimerStart();
    }
}

void SERCOM9_USART_ReadCallback(uintptr_t context) {
    uint8_t byte;
    if (!SERCOM9_USART_ReadIsBusy()) {
        if (!SERCOM9_USART_Read(&byte, 1))
        {
            SYS_CONSOLE_PRINT("RS485 channel 2: Read error (overrun or not ready)\r\n");
            ReportError(ERR_RS485_2_OVERRUN);
        }
        if (modbusRxIndex9 < MODBUS_MAX_LEN) {
            modbusRxBuffer9[modbusRxIndex9++] = byte;
        }
        TCC1_TimerStop();
        TCC1_TimerStart();
    }
}

void TCC0_Callback_InterruptHandler(uint32_t status, uintptr_t context) {
    __DMB();
    TCC0_TimerStop();
    if (modbusRxIndex8 > 0) {
        modbusFrameReady8 = true;
    }
}

void TCC1_Callback_InterruptHandler(uint32_t status, uintptr_t context) {
    __DMB();
    TCC1_TimerStop();
    if (modbusRxIndex9 > 0) {
        modbusFrameReady9 = true;
    }
}

void vUartRxHandlerTask(void *pvParameters) {
    while (1) {
        // ---------- UART8 Handler ----------
        bool frameReady8 = false;
        taskENTER_CRITICAL();
        if (modbusFrameReady8) {
            modbusFrameReady8 = false;
            frameReady8 = true;
        }
        taskEXIT_CRITICAL();

        if (frameReady8) {
            uint16_t crc8 = CalculateModbusCRC((const uint8_t *)modbusRxBuffer8, modbusRxIndex8 - 2U);
            uint16_t frameCrc8 = (uint16_t)modbusRxBuffer8[modbusRxIndex8 - 2U] |
                                 ((uint16_t)modbusRxBuffer8[modbusRxIndex8 - 1U] << 8U);

            if (crc8 == frameCrc8) {
                SYS_CONSOLE_PRINT("RS485_1 Frame (%d bytes): ", modbusRxIndex8);
                for (size_t i = 0U; i < modbusRxIndex8; i++) {
                    SYS_CONSOLE_PRINT("%02X ", modbusRxBuffer8[i]);
                }
                SYS_CONSOLE_PRINT("\r\n");

                ModbusMessage_t msg;
                memcpy(msg.buffer, (const void *)modbusRxBuffer8, modbusRxIndex8);
                msg.index = modbusRxIndex8;
                msg.length = modbusRxIndex8;

                if (xQueueSend(xUart8QueueHandler, &msg, 10U) != pdPASS) {
                    SYS_CONSOLE_PRINT("Failed to send message in UART8 queue\r\n");
                }
            } else {
                ReportError(ERR_RS485_1_INVALID_CRC);
                SYS_CONSOLE_PRINT("RS485_1 CRC Invalid\r\n");
            }

            for (size_t i = 0U; i < MODBUS_MAX_LEN; i++) {
                modbusRxBuffer8[i] = 0U;
            }
            modbusRxIndex8 = 0U;
            uint8_t dummy;
            SERCOM8_USART_Read(&dummy, 1U);
        }

        // ---------- UART9 Handler ----------
        bool frameReady9 = false;
        taskENTER_CRITICAL();
        if (modbusFrameReady9) {
            modbusFrameReady9 = false;
            frameReady9 = true;
        }
        taskEXIT_CRITICAL();

        if (frameReady9) {
            uint16_t crc9 = CalculateModbusCRC((const uint8_t *)modbusRxBuffer9, modbusRxIndex9 - 2U);
            uint16_t frameCrc9 = (uint16_t)modbusRxBuffer9[modbusRxIndex9 - 2U] |
                                 ((uint16_t)modbusRxBuffer9[modbusRxIndex9 - 1U] << 8U);

            if (crc9 == frameCrc9) {
                SYS_CONSOLE_PRINT("RS485_2 Frame (%d bytes): ", modbusRxIndex9);
                for (size_t i = 0U; i < modbusRxIndex9; i++) {
                    SYS_CONSOLE_PRINT("%02X ", modbusRxBuffer9[i]);
                }
                SYS_CONSOLE_PRINT("\r\n");

                ModbusMessage_t msg;
                memcpy(msg.buffer, (const void *)modbusRxBuffer9, modbusRxIndex9);
                msg.index = modbusRxIndex9;
                msg.length = modbusRxIndex9;

                if (xQueueSend(xUart9QueueHandler, &msg, 10U) != pdPASS) {
                    SYS_CONSOLE_PRINT("Failed to send message in UART9 queue\r\n");
                }
            } else {
                ReportError(ERR_RS485_2_INVALID_CRC);
                SYS_CONSOLE_PRINT("RS485_2 CRC Invalid\r\n");
            }

            for (size_t i = 0U; i < MODBUS_MAX_LEN; i++) {
                modbusRxBuffer9[i] = 0U;
            }
            modbusRxIndex9 = 0U;
            uint8_t dummy;
            SERCOM9_USART_Read(&dummy, 1U);
        }

        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}

/*
 * @brief   Initializes RS485 Handlers
 * @details This function registers USART read callbacks for SERCOM8 and SERCOM9
 *          and creates tasks for UART8 and UART9 server handlers.
 *
 * @param   void
 * @return  void
 */
void vRS485HandlerInit(void)
{
    static uint8_t dummy = 0U;

    /* Register SERCOM8 USART read callback */
    (void)SERCOM8_USART_ReadCallbackRegister(SERCOM8_USART_ReadCallback, (uintptr_t)0);
    (void)SERCOM8_USART_Read(&dummy, 1U);

    /* Register TCC0 timer callback */
    (void)TCC0_TimerCallbackRegister(TCC0_Callback_InterruptHandler, (uintptr_t)0);

#if HEV_IO_Aggregator
    /* Register SERCOM9 USART read callback */
    (void)SERCOM9_USART_ReadCallbackRegister(SERCOM9_USART_ReadCallback, (uintptr_t)0);
    (void)SERCOM9_USART_Read(&dummy, 1U);

    /* Register TCC1 timer callback */
    (void)TCC1_TimerCallbackRegister(TCC1_Callback_InterruptHandler, (uintptr_t)0);
#endif

    /* Create UART RX Handler Task */
    xTaskCreate(
        vUartRxHandlerTask,
        "UART_Modbus_Task",
        UART_RX_HANDLER_HEAP_DEPTH,
        NULL,
        UART_RX_HANDLER_TASK_PRIORITY,
        NULL
    );
    xUart8QueueHandler = xQueueCreate((UBaseType_t)10, sizeof(ModbusMessage_t));
#if HEV_IO_Aggregator
        xUart9QueueHandler = xQueueCreate((UBaseType_t)10, sizeof(ModbusMessage_t));
        if ((xUart8QueueHandler != NULL) && (xUart9QueueHandler != NULL))
        {
            (void)SYS_CONSOLE_PRINT("UARTRX Queue create success\r\n");
        }
#endif
        /* Create UART8 Handler Server Task */
        (void)xTaskCreate(
            vUart8HandlerServerTask,
            "UART8_Handler_Task",
            UART_SERVER_HANDLER_HEAP_DEPTH,
            NULL,
            UART_SERVER_HANDLER_TASK_PRIORITY,
            NULL
        );

#if HEV_IO_Aggregator
        /* Create UART9 Handler Server Task */
        (void)xTaskCreate(
            vUart9HandlerServerTask,
            "UART9_Handler_Task",
            UART_SERVER_HANDLER_HEAP_DEPTH,
            NULL,
            UART_SERVER_HANDLER_TASK_PRIORITY,
            NULL
        );
#endif
}

/* =========================================================================
 * Function Definitions
 * ========================================================================= */

void RS485_Output_1_Set(void)   { }
void RS485_Output_1_Clear(void) { }

void RS485_Output_2_Set(void)   { }
void RS485_Output_2_Clear(void) { }

void RS485_Output_3_Set(void)   { }
void RS485_Output_3_Clear(void) { }

void RS485_Output_4_Set(void)   { }
void RS485_Output_4_Clear(void) { }

void RS485_Output_5_Set(void)   { }
void RS485_Output_5_Clear(void) { }

void RS485_Output_6_Set(void)   { }
void RS485_Output_6_Clear(void) { }

void RS485_Output_7_Set(void)   { }
void RS485_Output_7_Clear(void) { }

void RS485_Output_8_Set(void)   { }
void RS485_Output_8_Clear(void) { }

void RS485_Output_9_Set(void)   { }
void RS485_Output_9_Clear(void) { }

void RS485_Output_10_Set(void)   { }
void RS485_Output_10_Clear(void) { }

void RS485_Output_11_Set(void)   { }
void RS485_Output_11_Clear(void) { }

void RS485_Output_12_Set(void)   { }
void RS485_Output_12_Clear(void) { }

void RS485_Output_13_Set(void)   { }
void RS485_Output_13_Clear(void) { }

void RS485_Output_14_Set(void)   { }
void RS485_Output_14_Clear(void) { }

void RS485_Output_15_Set(void)   { }
void RS485_Output_15_Clear(void) { }

void RS485_Output_16_Set(void)   { }
void RS485_Output_16_Clear(void) { }

void RS485_Output_17_Set(void)   { }
void RS485_Output_17_Clear(void) { }

void RS485_Output_18_Set(void)   { }
void RS485_Output_18_Clear(void) { }

uint8_t RS485_Output_1_Get(void)  { return 0U; }
uint8_t RS485_Output_2_Get(void)  { return 0U; }
uint8_t RS485_Output_3_Get(void)  { return 0U; }
uint8_t RS485_Output_4_Get(void)  { return 0U; }
uint8_t RS485_Output_5_Get(void)  { return 0U; }
uint8_t RS485_Output_6_Get(void)  { return 0U; }
uint8_t RS485_Output_7_Get(void)  { return 0U; }
uint8_t RS485_Output_8_Get(void)  { return 0U; }
uint8_t RS485_Output_9_Get(void)  { return 0U; }
uint8_t RS485_Output_10_Get(void) { return 0U; }
uint8_t RS485_Output_11_Get(void) { return 0U; }
uint8_t RS485_Output_12_Get(void) { return 0U; }
uint8_t RS485_Output_13_Get(void) { return 0U; }
uint8_t RS485_Output_14_Get(void) { return 0U; }
uint8_t RS485_Output_15_Get(void) { return 0U; }
uint8_t RS485_Output_16_Get(void) { return 0U; }
uint8_t RS485_Output_17_Get(void) { return 0U; }
uint8_t RS485_Output_18_Get(void) { return 0U; }