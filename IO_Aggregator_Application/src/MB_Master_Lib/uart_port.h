/*******************************************************************************
 * @file    uart_port.h
 * @brief   UART / RS485 Hardware Abstraction Layer (HAL) - Header
 * @details Abstracts all direct hardware access (SERCOM8, SERCOM9, direction
 *          GPIOs, timer callbacks) behind a clean port interface.
 *          The Modbus stack calls ONLY these APIs; no SERCOM symbols appear
 *          above this layer.
 *
 *          Two independent channels are supported:
 *            UART_PORT_CH1  →  SERCOM8 / RS485 bus 1
 *            UART_PORT_CH2  →  SERCOM9 / RS485 bus 2
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

#ifndef UART_PORT_H
#define UART_PORT_H

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* =========================================================================
 * Type Definitions
 * ========================================================================= */

/**
 * @brief  Channel identifiers for the two RS485 buses.
 */
typedef enum
{
    UART_PORT_CH1 = 0U,    /**< SERCOM8 — RS485 Bus 1 */
    UART_PORT_CH2 = 1U,    /**< SERCOM9 — RS485 Bus 2 */
    UART_PORT_CH_MAX       /**< Sentinel — number of channels */
} UartPort_Channel_t;

/**
 * @brief  Return codes for UART port operations.
 */
typedef enum
{
    UART_PORT_OK         = 0U,   /**< Operation succeeded */
    UART_PORT_ERR_PARAM  = 1U,   /**< Invalid parameter   */
    UART_PORT_ERR_BUSY   = 2U,   /**< Peripheral busy     */
    UART_PORT_ERR_HW     = 3U,   /**< Hardware fault      */
    UART_PORT_ERR_OVERRUN = 4U   /**< RX buffer overrun   */
} UartPort_Status_t;

/**
 * @brief  Prototype for the byte-received callback.
 *
 * @details Called from ISR context each time a byte arrives on the given
 *          channel.  Implementation must be ISR-safe (no blocking calls).
 *
 * @param[in]  eChannel   Channel on which the byte arrived.
 * @param[in]  u8Byte     The received byte value.
 */
typedef void (*UartPort_RxByteCallback_t)(UartPort_Channel_t eChannel,
                                           uint8_t            u8Byte);

/**
 * @brief  Prototype for the inter-frame silence (T3.5) callback.
 *
 * @details Called from timer-ISR context after the Modbus inter-frame gap
 *          has elapsed with no new bytes, indicating end-of-frame.
 *
 * @param[in]  eChannel   Channel whose silence timer fired.
 */
typedef void (*UartPort_FrameEndCallback_t)(UartPort_Channel_t eChannel);

/* =========================================================================
 * Configuration Constants
 * ========================================================================= */

/** Number of RS485 channels supported by this port layer */
#define UART_PORT_CHANNEL_COUNT    (2U)

/** Post-transmit DE pin hold time in milliseconds before switching to RX */
#define UART_PORT_TX_HOLD_MS       (7U)

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialise the UART port layer for a given channel.
 *
 * @details Registers SERCOM RX callbacks, primes the first Read(), registers
 *          the T3.5 timer callback, and leaves the RS485 transceiver in
 *          receive mode.  Must be called once per channel before any other
 *          port function.
 *
 * @param[in]  eChannel          Channel to initialise.
 * @param[in]  pfnRxByte         Callback invoked per received byte (ISR context).
 * @param[in]  pfnFrameEnd       Callback invoked on inter-frame silence (ISR context).
 *
 * @return  UART_PORT_OK on success, UART_PORT_ERR_PARAM if arguments invalid.
 */
UartPort_Status_t UartPort_Init(UartPort_Channel_t         eChannel,
                                 UartPort_RxByteCallback_t  pfnRxByte,
                                 UartPort_FrameEndCallback_t pfnFrameEnd);

/**
 * @brief  Transmit a block of bytes on the specified channel.
 *
 * @details Switches RS485 transceiver to TX, initiates SERCOM write, blocks
 *          until transmit complete, then delays UART_PORT_TX_HOLD_MS before
 *          switching back to RX.
 *          Must NOT be called from ISR context.
 *
 * @param[in]  eChannel   Target channel.
 * @param[in]  pu8Data    Pointer to data buffer.
 * @param[in]  u16Length  Number of bytes to send.
 *
 * @return  UART_PORT_OK on success; UART_PORT_ERR_PARAM or UART_PORT_ERR_HW on failure.
 */
UartPort_Status_t UartPort_Transmit(UartPort_Channel_t  eChannel,
                                     const uint8_t      *pu8Data,
                                     uint16_t            u16Length);

/**
 * @brief  Set the RS485 transceiver to transmit (DE high) mode.
 *
 * @param[in]  eChannel  Target channel.
 */
void UartPort_SetTxMode(UartPort_Channel_t eChannel);

/**
 * @brief  Set the RS485 transceiver to receive (DE low) mode.
 *
 * @param[in]  eChannel  Target channel.
 */
void UartPort_SetRxMode(UartPort_Channel_t eChannel);

/**
 * @brief  Re-arm the hardware RX read for the next byte on a channel.
 *
 * @details Called internally after consuming a byte from the SERCOM FIFO
 *          when Read() is byte-at-a-time.  Exposed here so the ISR glue
 *          code can call it without touching the Modbus layer.
 *
 * @param[in]  eChannel  Target channel.
 */
void UartPort_ArmRx(UartPort_Channel_t eChannel);

#endif /* UART_PORT_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/
