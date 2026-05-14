/*******************************************************************************
 * @file    uart_port.c
 * @brief   UART / RS485 Hardware Abstraction Layer (HAL) - Implementation
 * @details Implements all direct SERCOM8/SERCOM9 access, RS485 direction
 *          control, and timer-based inter-frame detection.
 *
 *          Hardware map (Two-Wheeler IO Aggregator variant):
 *            CH1 → SERCOM8, TCC0, RS485_EN1 GPIO
 *            CH2 → SERCOM9, TCC1, RS485_EN2 GPIO
 *
 *          Porting to a different MCU requires ONLY changes inside this file.
 *          The Modbus stack and application layer are hardware-independent.
 *
 * @company BACANCY SYSTEMS PVT. LTD.
 *******************************************************************************/

/* =========================================================================
 * Included Files
 * ========================================================================= */
#include "uart_port.h"
#include "definitions.h"          /* SERCOM, TCC, GPIO HAL from Harmony/ASF  */
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* =========================================================================
 * Private Types
 * ========================================================================= */

/**
 * @brief  Per-channel runtime state held by the port layer.
 */
typedef struct
{
    UartPort_RxByteCallback_t   pfnRxByte;      /**< Registered RX-byte callback  */
    UartPort_FrameEndCallback_t pfnFrameEnd;    /**< Registered frame-end callback */
    bool                        bInitialised;   /**< Channel is ready for use      */
} UartPort_ChannelCtx_t;

/* =========================================================================
 * Private Data
 * ========================================================================= */

/** Runtime state for each channel */
static UartPort_ChannelCtx_t s_axCtx[UART_PORT_CHANNEL_COUNT];

/** Single-byte staging registers used by SERCOM byte-at-a-time reads */
static uint8_t s_u8RxStageCh1;
static uint8_t s_u8RxStageCh2;

/* =========================================================================
 * Private ISR / Callback Implementations
 * =========================================================================
 * These functions are registered with the SERCOM and timer HAL.
 * They must be fast (no blocking, no FreeRTOS API that requires scheduler).
 * ========================================================================= */

/**
 * @brief  SERCOM8 byte-received ISR handler.
 */
static void prv_SERCOM8_RxCallback(uintptr_t context)
{
    (void)context;

    if (SERCOM8_USART_ReadIsBusy())
    {
        return;     /* Byte not ready yet — defensive guard */
    }

    if (!SERCOM8_USART_Read(&s_u8RxStageCh1, 1U))
    {
        /* Hardware overrun / framing error — re-arm and return */
        (void)SERCOM8_USART_Read(&s_u8RxStageCh1, 1U);
        return;
    }

    /* Restart the inter-frame silence timer on every received byte */
    TCC0_TimerStop();
    TCC0_TimerStart();

    /* Notify the Modbus RX parser */
    if (s_axCtx[UART_PORT_CH1].pfnRxByte != NULL)
    {
        s_axCtx[UART_PORT_CH1].pfnRxByte(UART_PORT_CH1, s_u8RxStageCh1);
    }

    /* Re-arm for the next byte */
    (void)SERCOM8_USART_Read(&s_u8RxStageCh1, 1U);
}

/**
 * @brief  SERCOM9 byte-received ISR handler.
 */
static void prv_SERCOM9_RxCallback(uintptr_t context)
{
    (void)context;

    if (SERCOM9_USART_ReadIsBusy())
    {
        return;
    }

    if (!SERCOM9_USART_Read(&s_u8RxStageCh2, 1U))
    {
        (void)SERCOM9_USART_Read(&s_u8RxStageCh2, 1U);
        return;
    }

    TCC1_TimerStop();
    TCC1_TimerStart();

    if (s_axCtx[UART_PORT_CH2].pfnRxByte != NULL)
    {
        s_axCtx[UART_PORT_CH2].pfnRxByte(UART_PORT_CH2, s_u8RxStageCh2);
    }

    (void)SERCOM9_USART_Read(&s_u8RxStageCh2, 1U);
}

/**
 * @brief  TCC0 inter-frame silence timer ISR — CH1 frame end.
 */
static void prv_TCC0_FrameEndCallback(uint32_t u32Status, uintptr_t context)
{
    (void)u32Status;
    (void)context;

    __DMB();    /* Data memory barrier — ensure ISR writes are visible */
    TCC0_TimerStop();

    if (s_axCtx[UART_PORT_CH1].pfnFrameEnd != NULL)
    {
        s_axCtx[UART_PORT_CH1].pfnFrameEnd(UART_PORT_CH1);
    }
}

/**
 * @brief  TCC1 inter-frame silence timer ISR — CH2 frame end.
 */
static void prv_TCC1_FrameEndCallback(uint32_t u32Status, uintptr_t context)
{
    (void)u32Status;
    (void)context;

    __DMB();
    TCC1_TimerStop();

    if (s_axCtx[UART_PORT_CH2].pfnFrameEnd != NULL)
    {
        s_axCtx[UART_PORT_CH2].pfnFrameEnd(UART_PORT_CH2);
    }
}

/* =========================================================================
 * Public Function Implementations
 * ========================================================================= */

UartPort_Status_t UartPort_Init(UartPort_Channel_t          eChannel,
                                 UartPort_RxByteCallback_t   pfnRxByte,
                                 UartPort_FrameEndCallback_t pfnFrameEnd)
{
    if ((eChannel >= UART_PORT_CH_MAX) ||
        (pfnRxByte  == NULL)           ||
        (pfnFrameEnd == NULL))
    {
        return UART_PORT_ERR_PARAM;
    }

    s_axCtx[eChannel].pfnRxByte   = pfnRxByte;
    s_axCtx[eChannel].pfnFrameEnd = pfnFrameEnd;
    s_axCtx[eChannel].bInitialised = true;

    if (eChannel == UART_PORT_CH1)
    {
        /* Leave transceiver in RX mode at startup */
        RS485_EN1_Clear();

        /* Register SERCOM8 callback and prime the first Read */
        (void)SERCOM8_USART_ReadCallbackRegister(prv_SERCOM8_RxCallback, (uintptr_t)0U);
        (void)SERCOM8_USART_Read(&s_u8RxStageCh1, 1U);

        /* Register TCC0 silence timer callback */
        (void)TCC0_TimerCallbackRegister(prv_TCC0_FrameEndCallback, (uintptr_t)0U);
    }
    else    /* UART_PORT_CH2 */
    {
        RS485_EN2_Clear();

        (void)SERCOM9_USART_ReadCallbackRegister(prv_SERCOM9_RxCallback, (uintptr_t)0U);
        (void)SERCOM9_USART_Read(&s_u8RxStageCh2, 1U);

        (void)TCC1_TimerCallbackRegister(prv_TCC1_FrameEndCallback, (uintptr_t)0U);
    }

    return UART_PORT_OK;
}

/* ------------------------------------------------------------------------- */

void UartPort_SetTxMode(UartPort_Channel_t eChannel)
{
    if (eChannel == UART_PORT_CH1)
    {
        RS485_EN1_Set();
    }
    else
    {
        RS485_EN2_Set();
    }
}

/* ------------------------------------------------------------------------- */

void UartPort_SetRxMode(UartPort_Channel_t eChannel)
{
    if (eChannel == UART_PORT_CH1)
    {
        RS485_EN1_Clear();
    }
    else
    {
        RS485_EN2_Clear();
    }
}

/* ------------------------------------------------------------------------- */

void UartPort_ArmRx(UartPort_Channel_t eChannel)
{
    /* Re-arm is already done inside the ISR callbacks.
     * This stub is provided for any external caller that needs to
     * flush and re-arm after an error condition. */
    uint8_t u8Dummy = 0U;

    if (eChannel == UART_PORT_CH1)
    {
        (void)SERCOM8_USART_Read(&u8Dummy, 1U);
    }
    else
    {
        (void)SERCOM9_USART_Read(&u8Dummy, 1U);
    }
}

/* ------------------------------------------------------------------------- */

UartPort_Status_t UartPort_Transmit(UartPort_Channel_t  eChannel,
                                     const uint8_t      *pu8Data,
                                     uint16_t            u16Length)
{
    bool bWriteOk;

    if ((eChannel >= UART_PORT_CH_MAX) ||
        (pu8Data  == NULL)             ||
        (u16Length == 0U))
    {
        return UART_PORT_ERR_PARAM;
    }

    /* Acquire critical section to protect direction switching and SERCOM write.
     * taskENTER_CRITICAL is safe here because the transmit-complete spin is
     * very short (frame duration at the baud rate). */
    taskENTER_CRITICAL();
    UartPort_SetTxMode(eChannel);

    if (eChannel == UART_PORT_CH1)
    {
        bWriteOk = SERCOM8_USART_Write((void *)pu8Data, (size_t)u16Length);
        if (bWriteOk)
        {
            while (!SERCOM8_USART_TransmitComplete()) { /* spin */ }
        }
    }
    else
    {
        bWriteOk = SERCOM9_USART_Write((void *)pu8Data, (size_t)u16Length);
        if (bWriteOk)
        {
            while (!SERCOM9_USART_TransmitComplete()) { /* spin */ }
        }
    }

    taskEXIT_CRITICAL();

    if (!bWriteOk)
    {
        UartPort_SetRxMode(eChannel);   /* Ensure bus is released on error */
        return UART_PORT_ERR_HW;
    }

    /* Hold DE asserted for a short time so the last stop bit clears the bus
     * before the transceiver switches to RX.  vTaskDelay here is acceptable
     * because we are NOT inside a critical section at this point. */
    vTaskDelay(pdMS_TO_TICKS(UART_PORT_TX_HOLD_MS));
    UartPort_SetRxMode(eChannel);

    return UART_PORT_OK;
}

/*******************************************************************************
 * End of File
 *******************************************************************************/
