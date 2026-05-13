/*******************************************************************************
 * @file    AppUartHandler.h
 * @brief   UART Handler Application Header File
 * @details This file provides function prototypes and macros for UART handling.
 * 
 * @company BACANCY SYSTEMS PVT. LTD. 
 *******************************************************************************/

#ifndef APP_UART_HANDLER_H
#define APP_UART_HANDLER_H

/* ************************************************************************** */
/* Section: Included Files */
/* ************************************************************************** */
#include <stdint.h>   /* Fixed-width integer types */
#include <stdbool.h>  /* Boolean type definitions */
#include <stdlib.h>   /* Standard library functions */

/* ************************************************************************** */
/* Section: Macro Definitions */
/* ************************************************************************** */
#define UART_RX_HANDLER_HEAP_DEPTH      512  /* Task heap depth for RX handler */
#define UART_RX_HANDLER_TASK_PRIORITY   4//6  /* RX handler task priority */

#define UART_SERVER_HANDLER_HEAP_DEPTH  512  /* Task heap depth for server handler */
#define UART_SERVER_HANDLER_TASK_PRIORITY 4//5  /* Server handler task priority */


#define MODBUS_CRC16_POLY              (0xA001U)  /* CRC-16 Polynomial for Modbus */

#define USART_BUFFER_SIZE              (128U)  /* USART buffer size */
#define UART_QUEUE_SIZE                (10U)   /* Queue size for UART data */

/* UART Server Port Definitions */
#if HEV_IO_Aggregator
    extern uint32_t uartBaud[2];
    extern uint8_t dataBits[2];
    extern char parity[2];
    extern uint8_t stopBits[2];
    extern flash_data_t writeData;
    extern TCP_SOCKET sUart8ServerSocket;
    extern TCP_SOCKET sUart9ServerSocket;     
#endif
    
#if Two_Wheeler_IO_Aggregator 
    #define UART8_SERVER_PORT     (30001)
    #define UART9_SERVER_PORT     (30002)
#endif

/* ************************************************************************** */
/* Section: Function Prototypes */
/* ************************************************************************** */

/**
 * @brief   Task function for UART8 server handling.
 * @param   pvParameters - Pointer to task parameters.
 * @return  None
 */
void vUart8HandlerServerTask(void *pvParameters);

/**
 * @brief   Task function for UART9 server handling.
 * @param   pvParameters - Pointer to task parameters.
 * @return  None
 */
void vUart9HandlerServerTask(void *pvParameters);

/**
 * @brief   Initializes RS485 Handlers.
 * @details Registers USART callbacks and creates UART8 & UART9 tasks.
 * @return  None
 */
void vRS485HandlerInit(void);

/**
 * @brief  Transmit a data buffer over UART8.
 *
 * @param[in]  u8data   Pointer to the buffer containing data to transmit.
 * @param[in]  u8len    Number of bytes to transmit.
 *
 * @return  0 if the transmission is successful; 1 if a failure occurs.
 */
uint8_t u8Uart8Write(uint8_t *u8data, uint8_t u8len);

/**
 * @brief  Transmit a data buffer over UART9.
 *
 * @param[in]  u8data   Pointer to the buffer containing data to transmit.
 * @param[in]  u8len    Number of bytes to transmit.
 *
 * @return  0 if the transmission is successful; 1 if a failure occurs.
 */
uint8_t u8Uart9Write(uint8_t *u8data, uint8_t u8len);

/**
 * @brief  Transmit RS485 data over UART8 interface.
 *
 * @param[in]  data     Pointer to the buffer containing RS485 data.
 * @param[in]  length   Length of the data in bytes.
 */
void RS485_TransmitData_UART8(uint8_t *data, uint16_t length);

/**
 * @brief  Transmit RS485 data over UART9 interface.
 *
 * @param[in]  data     Pointer to the buffer containing RS485 data.
 * @param[in]  length   Length of the data in bytes.
 */
void RS485_TransmitData_UART9(uint8_t *data, uint16_t length);

void RS485_Output_1_Set(void);
void RS485_Output_1_Clear(void);

void RS485_Output_2_Set(void);
void RS485_Output_2_Clear(void);

void RS485_Output_3_Set(void);
void RS485_Output_3_Clear(void);

void RS485_Output_4_Set(void);
void RS485_Output_4_Clear(void);

void RS485_Output_5_Set(void);
void RS485_Output_5_Clear(void);

void RS485_Output_6_Set(void);
void RS485_Output_6_Clear(void);

void RS485_Output_7_Set(void);
void RS485_Output_7_Clear(void);

void RS485_Output_8_Set(void);
void RS485_Output_8_Clear(void);

void RS485_Output_9_Set(void);
void RS485_Output_9_Clear(void);

void RS485_Output_10_Set(void);
void RS485_Output_10_Clear(void);

void RS485_Output_11_Set(void);
void RS485_Output_11_Clear(void);

void RS485_Output_12_Set(void);
void RS485_Output_12_Clear(void);

void RS485_Output_13_Set(void);
void RS485_Output_13_Clear(void);

void RS485_Output_14_Set(void);
void RS485_Output_14_Clear(void);

void RS485_Output_15_Set(void);
void RS485_Output_15_Clear(void);

void RS485_Output_16_Set(void);
void RS485_Output_16_Clear(void);

void RS485_Output_17_Set(void);
void RS485_Output_17_Clear(void);

void RS485_Output_18_Set(void);
void RS485_Output_18_Clear(void);

uint8_t RS485_Output_1_Get(void);
uint8_t RS485_Output_2_Get(void);
uint8_t RS485_Output_3_Get(void);
uint8_t RS485_Output_4_Get(void);
uint8_t RS485_Output_5_Get(void);
uint8_t RS485_Output_6_Get(void);
uint8_t RS485_Output_7_Get(void);
uint8_t RS485_Output_8_Get(void);
uint8_t RS485_Output_9_Get(void);
uint8_t RS485_Output_10_Get(void);
uint8_t RS485_Output_11_Get(void);
uint8_t RS485_Output_12_Get(void);
uint8_t RS485_Output_13_Get(void);
uint8_t RS485_Output_14_Get(void);
uint8_t RS485_Output_15_Get(void);
uint8_t RS485_Output_16_Get(void);
uint8_t RS485_Output_17_Get(void);
uint8_t RS485_Output_18_Get(void);
#endif /* APP_UART_HANDLER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/
